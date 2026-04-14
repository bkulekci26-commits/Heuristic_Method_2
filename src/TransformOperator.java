import java.util.*;

/**
 * Transfer operator adapted from Aguayo et al. (2025) VRP-T methodology
 * for the Capacitated Team Orienteering Problem with Transfers and Synchronization.
 *
 * Aguayo's key insight: don't add transfers to packed routes as post-optimization.
 * Instead, evaluate complete transfer moves (including the new customer insertion
 * that exploits the freed capacity) as atomic operations.
 *
 * Three strategies, applied iteratively until no improvement:
 *
 *   1. DIRECT_INSERT: Find unserved customers that fit in existing routes.
 *   2. SINGLE_SWAP: Replace a low-profit customer with a high-profit one.
 *   3. CROSS_TRANSFER (Archetype A): Route k2 drops off load at a node j
 *      that k1 already visits. k1 picks up this load, freeing capacity.
 *      k1 uses freed capacity to insert a new profitable customer u.
 *   4. REVERSE_TRANSFER (Archetype B): Route k2 already serves node j and
 *      drops off extra load there. k1 detours to j with a pickup-only stop.
 *      k1's initialLoad decreases, freeing capacity to insert new customer u.
 *      THIS TYPE IS UNIQUE TO CTOP-T — impossible in SD-CTOP.
 *
 * Key difference from VRP-T: In orienteering, transfers are valuable only if they
 * enable serving ADDITIONAL customers (profit gain), not for distance savings.
 */
public class TransformOperator {

    // ══════════════════════════════════════════════════════════════════════════
    //  MOVE TYPES
    // ══════════════════════════════════════════════════════════════════════════

    private enum MoveType { DIRECT_INSERT, SINGLE_SWAP, CROSS_TRANSFER, REVERSE_TRANSFER }

    /** Represents a candidate move to be evaluated */
    private static class Move {
        MoveType type;
        double netProfit;

        // DIRECT_INSERT: insert unserved node u into route k
        Node insertNode;
        int insertRouteIdx;
        int insertPos;

        // SINGLE_SWAP: remove node s from route, insert node u
        Node removeNode;
        int removeRouteIdx;

        // CROSS_TRANSFER (Archetype A): j ∈ k1. k2 drops at j, k1 picks up at j.
        //   k1 = serverRouteIdx (serves j + picks up), k2 = donorRouteIdx (drops off)
        //   u inserted in k1 using freed capacity
        int serverRouteIdx;     // k1: the route that serves transferNode and picks up
        int donorRouteIdx;      // k2: the route that drops off load
        int donorInsertPos;     // where k2 inserts the dropoff stop
        Node transferNode;      // the node where transfer happens
        double transferQty;     // how much is transferred
        int newCustInsertPos;   // where u is inserted in k1

        // REVERSE_TRANSFER (Archetype B): j ∈ k2. k2 serves+drops at j, k1 detours to pick up.
        //   k1 = pickerRouteIdx (picks up only), k2 = serverRouteIdx (serves j + drops)
        //   u inserted in k1 using picked-up capacity
        int pickerInsertPos;    // where pickup-only stop goes in k1
    }

    // ══════════════════════════════════════════════════════════════════════════
    //  MAIN ENTRY POINT
    // ══════════════════════════════════════════════════════════════════════════

    /** Maximum time (ms) allowed for the entire optimize() call */
    private static final long TIME_BUDGET_MS = 5000; // 5 seconds max

    /**
     * Optimize the solution by finding profitable transfer opportunities.
     * Returns the number of improving moves applied.
     */
    public int optimize(Solution solution) {
        Solution backup = new Solution(solution);
        int totalMoves = 0;
        int directInserts = 0, swaps = 0, transfersA = 0, transfersB = 0;
        boolean improved = true;
        long startTime = System.currentTimeMillis();

        while (improved) {
            improved = false;
            long elapsed = System.currentTimeMillis() - startTime;
            if (elapsed > TIME_BUDGET_MS) {
                System.out.printf("[Transform] Time budget exceeded (%.1fs), stopping%n",
                        elapsed / 1000.0);
                break;
            }

            // Find the best move across all strategies
            Move bestDirect = findBestDirectInsert(solution);
            Move bestSwap = findBestSingleSwap(solution);
            Move bestTransferA = findBestCrossTransfer(solution, startTime);
            Move bestTransferB = findBestReverseTransfer(solution, startTime);

            // Pick the best overall
            Move best = null;
            if (bestDirect != null && (best == null || bestDirect.netProfit > best.netProfit))
                best = bestDirect;
            if (bestSwap != null && (best == null || bestSwap.netProfit > best.netProfit))
                best = bestSwap;
            if (bestTransferA != null && (best == null || bestTransferA.netProfit > best.netProfit))
                best = bestTransferA;
            if (bestTransferB != null && (best == null || bestTransferB.netProfit > best.netProfit))
                best = bestTransferB;

            if (best != null && best.netProfit > 1e-6) {
                applyMove(solution, best);
                totalMoves++;
                improved = true;

                switch (best.type) {
                    case DIRECT_INSERT: directInserts++; break;
                    case SINGLE_SWAP: swaps++; break;
                    case CROSS_TRANSFER: transfersA++; break;
                    case REVERSE_TRANSFER: transfersB++; break;
                }
            }
        }

        // Validate final solution
        for (Route r : solution.getRoutes()) r.evaluate();
        if (!solution.isFeasible()) {
            Solution.FeasibilityReport fr = solution.checkFeasibility();
            System.out.println("[Transform] Post-apply infeasible, rolling back:");
            for (String v : fr.getViolations()) System.out.println("  → " + v);
            solution.restoreFrom(backup);
            for (Route r : solution.getRoutes()) r.evaluate();
            return 0;
        }

        if (totalMoves > 0) {
            double delta = solution.getTotalProfit() - backup.getTotalProfit();
            System.out.printf("[Transform] Applied %d moves (insert=%d, swap=%d, transferA=%d, transferB=%d), " +
                            "profit %.0f → %.0f (%+.0f)%n",
                    totalMoves, directInserts, swaps, transfersA, transfersB,
                    backup.getTotalProfit(), solution.getTotalProfit(), delta);
        }

        return totalMoves;
    }

    // ══════════════════════════════════════════════════════════════════════════
    //  STRATEGY 1: DIRECT INSERT
    // ══════════════════════════════════════════════════════════════════════════

    private Move findBestDirectInsert(Solution sol) {
        List<Node> unserved = sol.getUnservedNodes();
        List<Route> routes = sol.getRoutes();
        Move best = null;

        for (Node u : unserved) {
            for (int ri = 0; ri < routes.size(); ri++) {
                Route route = routes.get(ri);
                for (int pos = 0; pos <= route.size(); pos++) {
                    if (route.canInsert(pos, RouteStop.serve(u))) {
                        double profit = u.getProfit();
                        if (best == null || profit > best.netProfit) {
                            best = new Move();
                            best.type = MoveType.DIRECT_INSERT;
                            best.netProfit = profit;
                            best.insertNode = u;
                            best.insertRouteIdx = ri;
                            best.insertPos = pos;
                        }
                    }
                }
            }
        }
        return best;
    }

    // ══════════════════════════════════════════════════════════════════════════
    //  STRATEGY 2: SINGLE-VEHICLE SWAP
    // ══════════════════════════════════════════════════════════════════════════

    private Move findBestSingleSwap(Solution sol) {
        List<Node> unserved = sol.getUnservedNodes();
        List<Route> routes = sol.getRoutes();
        Instance inst = sol.getInstance();
        Move best = null;

        for (int ri = 0; ri < routes.size(); ri++) {
            Route route = routes.get(ri);

            for (int si = 0; si < route.size(); si++) {
                RouteStop stop = route.getStops().get(si);
                if (!stop.isServed()) continue;
                Node removeNode = stop.getNode();
                double removedProfit = removeNode.getProfit();

                // Try replacing with each unserved node
                for (Node u : unserved) {
                    double netProfit = u.getProfit() - removedProfit;
                    if (netProfit <= 0) continue;

                    // Test on a copy
                    Route copy = new Route(route);
                    copy.removeStop(si);
                    int bestPos = findBestFeasiblePos(copy, u, inst);
                    if (bestPos >= 0) {
                        if (best == null || netProfit > best.netProfit) {
                            best = new Move();
                            best.type = MoveType.SINGLE_SWAP;
                            best.netProfit = netProfit;
                            best.insertNode = u;
                            best.removeNode = removeNode;
                            best.insertRouteIdx = ri;
                            best.removeRouteIdx = ri;
                            best.insertPos = bestPos;
                        }
                    }
                }
            }
        }
        return best;
    }

    // ══════════════════════════════════════════════════════════════════════════
    //  STRATEGY 3: CROSS-VEHICLE TRANSFER (Aguayo-adapted)
    // ══════════════════════════════════════════════════════════════════════════

    /**
     * Aguayo-adapted cross-vehicle transfer for orienteering.
     *
     * For each route k1 (server/picker) and served customer j in k1:
     *   For each route k2 (donor/dropper) ≠ k1:
     *     - Can k2 visit j? (find time-feasible position for dropoff)
     *     - k2 drops transferQty at j, k1 picks up → k1 has freed capacity
     *     - Can k1 insert an unserved node u using freed + existing capacity?
     *     - Check sync: dropper(k2) arrival - picker(k1) arrival ≤ W
     *     - If all feasible and profitable: record this move
     *
     * Load model insight:
     *   When k1 picks up transferQty at position sj, k1's initialLoad decreases
     *   by transferQty. This frees capacity on ALL arcs from depot to j.
     *   Node u can be inserted at ANY position (not just before j) — the route
     *   copy feasibility check handles all arc load constraints correctly.
     */
    private Move findBestCrossTransfer(Solution sol, long startTime) {
        Instance inst = sol.getInstance();
        List<Route> routes = sol.getRoutes();
        List<Node> unserved = sol.getUnservedNodes();
        double W = inst.getSyncWindow();
        double Q = inst.getMaxCapacity();
        int K = routes.size();

        Move best = null;
        int syncChecked = 0, syncFailed = 0, timeFailed = 0, capFailed = 0;

        // Sort unserved by profit descending for early pruning
        unserved.sort((a, b) -> Double.compare(b.getProfit(), a.getProfit()));

        for (int i1 = 0; i1 < K; i1++) {
            if (System.currentTimeMillis() - startTime > TIME_BUDGET_MS) break;
            Route k1 = routes.get(i1);
            k1.evaluate();

            for (int sj = 0; sj < k1.size(); sj++) {
                RouteStop stopJ = k1.getStops().get(sj);
                if (!stopJ.isServed()) continue;
                Node nodeJ = stopJ.getNode();

                for (int i2 = 0; i2 < K; i2++) {
                    if (i1 == i2) continue;
                    Route k2 = routes.get(i2);
                    k2.evaluate();

                    // Skip if k2 already visits nodeJ
                    if (k2.visitsNode(nodeJ.getId())) continue;

                    // Can k2 visit nodeJ? Find best time-feasible position for dropoff
                    int posK2 = findBestFeasiblePosForDropoff(k2, nodeJ, inst);
                    if (posK2 < 0) { timeFailed++; continue; }

                    // How much spare capacity does k2 have?
                    double k2SpareCap = k2.getRemainingCapacity();
                    if (k2SpareCap < 1) { capFailed++; continue; }

                    // FIX #3: Transfer quantity = k2's full spare capacity
                    // (NOT capped at nodeJ.demand — transfer is just a meeting point)
                    double transferQty = Math.floor(k2SpareCap); // integer units

                    // Pre-compute k2 dropoff copy (shared across all u attempts)
                    Route k2Copy = new Route(k2);
                    k2Copy.insertStop(posK2, RouteStop.dropoff(nodeJ, transferQty));
                    k2Copy.evaluate();
                    if (!k2Copy.isFeasible()) { timeFailed++; continue; }
                    double dropperArr = k2Copy.getArrivalTimeAtNode(nodeJ.getId());

                    // Existing slack in k1 (capacity that's already free)
                    double k1ExistingSlack = k1.getRemainingCapacity();

                    // Now try inserting unserved customers in k1 using freed + existing capacity
                    for (Node u : unserved) {
                        // FIX #1: No demand pre-filter — route copy check handles feasibility.
                        // Only skip if demand clearly impossible (exceeds total available).
                        if (u.getDemand() > transferQty + k1ExistingSlack + 1) continue;
                        if (best != null && u.getProfit() <= best.netProfit) continue; // pruning

                        // FIX #2: Try ALL positions for u in k1 (not just before j)
                        // Freed capacity from pickup at j helps arcs before j.
                        // Existing slack may allow insertion after j too.
                        for (int posU = 0; posU <= k1.size(); posU++) {

                            // Build modified k1: insert u, add pickup at j
                            Route k1Copy = new Route(k1);
                            k1Copy.insertStop(posU, RouteStop.serve(u));

                            // j's position shifts if u was inserted before or at j
                            int newPosJ = (posU <= sj) ? sj + 1 : sj;

                            // Replace j's stop with serve+pickup
                            k1Copy.getStops().set(newPosJ,
                                    new RouteStop(nodeJ, true, true, false,
                                            transferQty, 0, nodeJ.getDemand()));
                            k1Copy.evaluate();

                            // Check k1 feasibility (time + capacity)
                            if (!k1Copy.isFeasible()) {
                                timeFailed++;
                                continue;
                            }

                            // Check sync: dropper(k2) arrival - picker(k1) arrival ≤ W
                            double pickerArr = k1Copy.getArrivalTimeAtNode(nodeJ.getId());
                            double syncGap = dropperArr - pickerArr;
                            syncChecked++;

                            if (syncGap > W + 1e-6) {
                                syncFailed++;
                                continue;
                            }

                            // Net profit: profit(u) — no customer lost
                            double netProfit = u.getProfit();

                            if (best == null || netProfit > best.netProfit) {
                                best = new Move();
                                best.type = MoveType.CROSS_TRANSFER;
                                best.netProfit = netProfit;
                                best.insertNode = u;
                                best.serverRouteIdx = i1;
                                best.donorRouteIdx = i2;
                                best.donorInsertPos = posK2;
                                best.transferNode = nodeJ;
                                best.transferQty = transferQty;
                                best.newCustInsertPos = posU;
                            }
                        }
                    }
                }
            }
        }

        System.out.printf("[Transform] Cross-transfer search: %d sync-checked, %d sync-failed, " +
                        "%d time-failed, %d cap-failed, best=%s%n", syncChecked, syncFailed, timeFailed, capFailed,
                best != null ? String.format("net=+%.0f", best.netProfit) : "none");

        return best;
    }

    // ══════════════════════════════════════════════════════════════════════════
    //  STRATEGY 4: REVERSE TRANSFER (Archetype B — unique to CTOP-T)
    // ══════════════════════════════════════════════════════════════════════════

    /**
     * Archetype B: Dropper serves AND drops extra load; picker detours to collect.
     *
     * This is the transfer type that is ONLY possible in CTOP-T, not in SD-CTOP.
     * In SD-CTOP, both visiting vehicles must serve the customer's demand.
     * In CTOP-T, one vehicle visits purely as a transfer point.
     *
     * For each route k2 (server/dropper) and served customer j in k2:
     *   k2 has spare capacity → can carry extra load from depot and drop it at j
     *   For each route k1 (picker) ≠ k2:
     *     k1 detours to j, inserts a PICKUP-ONLY stop
     *     k1's initialLoad decreases → capacity freed on arcs before j
     *     k1 inserts new unserved customer u using the freed capacity
     *
     * Result:
     *   k1: depot → ... → u[S] → ... → j[P:tq] → ... → depot
     *   k2: depot → ... → j[S+D:tq] → ... → depot
     */
    private Move findBestReverseTransfer(Solution sol, long startTime) {
        Instance inst = sol.getInstance();
        List<Route> routes = sol.getRoutes();
        List<Node> unserved = sol.getUnservedNodes();
        double W = inst.getSyncWindow();
        int K = routes.size();

        Move best = null;
        int syncChecked = 0, syncFailed = 0, timeFailed = 0, capFailed = 0;

        unserved.sort((a, b) -> Double.compare(b.getProfit(), a.getProfit()));

        for (int i2 = 0; i2 < K; i2++) {
            if (System.currentTimeMillis() - startTime > TIME_BUDGET_MS) break;
            Route k2 = routes.get(i2);   // server+dropper: has j, will add dropoff
            k2.evaluate();
            double k2SpareCap = k2.getRemainingCapacity();
            if (k2SpareCap < 1) { capFailed++; continue; }
            double transferQty = Math.floor(k2SpareCap);

            for (int sj = 0; sj < k2.size(); sj++) {
                RouteStop stopJ = k2.getStops().get(sj);
                if (!stopJ.isServed()) continue;
                Node nodeJ = stopJ.getNode();

                // Pre-build k2 copy: change j to serve+dropoff
                Route k2Copy = new Route(k2);
                k2Copy.getStops().set(sj,
                        RouteStop.serveAndDropoff(nodeJ, transferQty));
                k2Copy.evaluate();
                if (!k2Copy.isFeasible()) { capFailed++; continue; }
                double dropperArr = k2Copy.getArrivalTimeAtNode(nodeJ.getId());

                for (int i1 = 0; i1 < K; i1++) {
                    if (i1 == i2) continue;
                    if (System.currentTimeMillis() - startTime > TIME_BUDGET_MS) break;
                    Route k1 = routes.get(i1);   // picker: detours to j
                    k1.evaluate();

                    if (k1.visitsNode(nodeJ.getId())) continue;

                    // Try each position for pickup-only stop at j in k1
                    for (int posJ = 0; posJ <= k1.size(); posJ++) {

                        // Quick time check for pickup detour
                        Node depot = inst.getDepot();
                        List<RouteStop> stops = k1.getStops();
                        Node prev = (posJ == 0) ? depot : stops.get(posJ - 1).getNode();
                        Node next = (posJ == stops.size()) ? depot : stops.get(posJ).getNode();
                        double detour = inst.getDistance(prev, nodeJ) + inst.getDistance(nodeJ, next)
                                - inst.getDistance(prev, next);
                        if (k1.getTotalTime() + detour > inst.getMaxRouteDuration() + 1e-6) {
                            timeFailed++;
                            continue;
                        }

                        // Existing slack in k1
                        double k1Slack = k1.getRemainingCapacity();

                        for (Node u : unserved) {
                            if (u.getDemand() > transferQty + k1Slack + 1) continue;
                            if (best != null && u.getProfit() <= best.netProfit) continue;

                            // Try each position for u in k1
                            for (int posU = 0; posU <= k1.size(); posU++) {

                                // Build modified k1: insert both pickup(j) and serve(u)
                                Route k1Copy = new Route(k1);

                                // Insert in order: first the one at lower position
                                if (posU <= posJ) {
                                    k1Copy.insertStop(posU, RouteStop.serve(u));
                                    // posJ shifted right by 1
                                    k1Copy.insertStop(posJ + 1, RouteStop.pickup(nodeJ, transferQty));
                                } else {
                                    k1Copy.insertStop(posJ, RouteStop.pickup(nodeJ, transferQty));
                                    // posU shifted right by 1
                                    k1Copy.insertStop(posU + 1, RouteStop.serve(u));
                                }
                                k1Copy.evaluate();

                                if (!k1Copy.isFeasible()) {
                                    timeFailed++;
                                    continue;
                                }

                                // Check sync: dropper(k2) - picker(k1) ≤ W
                                double pickerArr = k1Copy.getArrivalTimeAtNode(nodeJ.getId());
                                double syncGap = dropperArr - pickerArr;
                                syncChecked++;

                                if (syncGap > W + 1e-6) {
                                    syncFailed++;
                                    continue;
                                }

                                double netProfit = u.getProfit();
                                if (best == null || netProfit > best.netProfit) {
                                    best = new Move();
                                    best.type = MoveType.REVERSE_TRANSFER;
                                    best.netProfit = netProfit;
                                    best.insertNode = u;
                                    best.serverRouteIdx = i2;   // k2 serves j
                                    best.donorRouteIdx = i1;    // k1 picks up (reuse field)
                                    best.transferNode = nodeJ;
                                    best.transferQty = transferQty;
                                    best.newCustInsertPos = posU;  // u position in k1
                                    best.pickerInsertPos = posJ;   // pickup position in k1
                                }
                            }
                        }
                    }
                }
            }
        }

        System.out.printf("[Transform] Reverse-transfer search: %d sync-checked, %d sync-failed, " +
                        "%d time-failed, %d cap-failed, best=%s%n", syncChecked, syncFailed, timeFailed, capFailed,
                best != null ? String.format("net=+%.0f", best.netProfit) : "none");

        return best;
    }

    // ══════════════════════════════════════════════════════════════════════════
    //  MOVE APPLICATION
    // ══════════════════════════════════════════════════════════════════════════

    private void applyMove(Solution sol, Move move) {
        List<Route> routes = sol.getRoutes();

        switch (move.type) {
            case DIRECT_INSERT: {
                Route route = routes.get(move.insertRouteIdx);
                route.insertStop(move.insertPos, RouteStop.serve(move.insertNode));
                route.evaluate();
                System.out.printf("[Transform] DIRECT_INSERT: node %d (p=%.0f) in v%d%n",
                        move.insertNode.getId(), move.insertNode.getProfit(),
                        route.getVehicleId());
                break;
            }

            case SINGLE_SWAP: {
                Route route = routes.get(move.insertRouteIdx);
                int removeIdx = route.findStopIndex(move.removeNode.getId());
                route.removeStop(removeIdx);
                int insertPos = findBestFeasiblePos(route, move.insertNode, sol.getInstance());
                route.insertStop(insertPos, RouteStop.serve(move.insertNode));
                route.evaluate();
                System.out.printf("[Transform] SINGLE_SWAP: -%d (p=%.0f) +%d (p=%.0f) in v%d%n",
                        move.removeNode.getId(), move.removeNode.getProfit(),
                        move.insertNode.getId(), move.insertNode.getProfit(),
                        route.getVehicleId());
                break;
            }

            case CROSS_TRANSFER: {
                Route k1 = routes.get(move.serverRouteIdx);
                Route k2 = routes.get(move.donorRouteIdx);
                Node nodeJ = move.transferNode;
                double tq = move.transferQty;

                // Insert new customer u in k1
                k1.insertStop(move.newCustInsertPos, RouteStop.serve(move.insertNode));
                // Find j's position (may have shifted)
                int posJ = k1.findStopIndex(nodeJ.getId());
                // Replace j's stop with serve+pickup
                k1.getStops().set(posJ,
                        new RouteStop(nodeJ, true, true, false, tq, 0, nodeJ.getDemand()));
                k1.evaluate();

                // Insert dropoff in k2
                k2.insertStop(move.donorInsertPos, RouteStop.dropoff(nodeJ, tq));
                k2.evaluate();

                // Register transfer: k2 gives (drops off), k1 receives (picks up)
                sol.addTransfer(new Transfer(
                        nodeJ.getId(), k2.getVehicleId(), k1.getVehicleId(), tq));

                System.out.printf("[Transform] CROSS_TRANSFER(A): +node %d (p=%.0f) in v%d, " +
                                "transfer %.0f at node %d: v%d drops → v%d picks%n",
                        move.insertNode.getId(), move.insertNode.getProfit(),
                        k1.getVehicleId(), tq, nodeJ.getId(),
                        k2.getVehicleId(), k1.getVehicleId());
                break;
            }

            case REVERSE_TRANSFER: {
                // Archetype B: k2 serves j + drops extra, k1 detours to pick up
                Route k2 = routes.get(move.serverRouteIdx);   // serves j
                Route k1 = routes.get(move.donorRouteIdx);    // picks up
                Node nodeJ = move.transferNode;
                double tq = move.transferQty;

                // Modify k2: change j from serve to serve+dropoff
                int posJinK2 = k2.findStopIndex(nodeJ.getId());
                k2.getStops().set(posJinK2, RouteStop.serveAndDropoff(nodeJ, tq));
                k2.evaluate();

                // Modify k1: insert pickup-only at j AND serve(u)
                int posJ = move.pickerInsertPos;
                int posU = move.newCustInsertPos;

                if (posU <= posJ) {
                    k1.insertStop(posU, RouteStop.serve(move.insertNode));
                    k1.insertStop(posJ + 1, RouteStop.pickup(nodeJ, tq));
                } else {
                    k1.insertStop(posJ, RouteStop.pickup(nodeJ, tq));
                    k1.insertStop(posU + 1, RouteStop.serve(move.insertNode));
                }
                k1.evaluate();

                // Register transfer: k2 gives (drops off), k1 receives (picks up)
                sol.addTransfer(new Transfer(
                        nodeJ.getId(), k2.getVehicleId(), k1.getVehicleId(), tq));

                System.out.printf("[Transform] REVERSE_TRANSFER(B): +node %d (p=%.0f) in v%d, " +
                                "v%d serves+drops %.0f at node %d → v%d picks up%n",
                        move.insertNode.getId(), move.insertNode.getProfit(),
                        k1.getVehicleId(),
                        k2.getVehicleId(), tq, nodeJ.getId(),
                        k1.getVehicleId());
                break;
            }
        }
    }

    // ══════════════════════════════════════════════════════════════════════════
    //  HELPERS
    // ══════════════════════════════════════════════════════════════════════════

    /**
     * Find the best feasible position to insert a serve stop for node u.
     * Returns -1 if no feasible position exists.
     */
    private int findBestFeasiblePos(Route route, Node node, Instance inst) {
        double bestTime = Double.MAX_VALUE;
        int bestPos = -1;
        for (int p = 0; p <= route.size(); p++) {
            RouteStop s = RouteStop.serve(node);
            route.insertStop(p, s);
            route.evaluate();
            if (route.isFeasible() && route.getTotalTime() < bestTime) {
                bestTime = route.getTotalTime();
                bestPos = p;
            }
            route.removeStop(p);
            route.evaluate();
        }
        return bestPos;
    }

    /**
     * Find the best feasible position to insert a dropoff stop.
     * Returns -1 if no time-feasible position exists.
     * Uses a dummy dropoff with qty=0 to test time feasibility only
     * (capacity is checked separately with actual transfer quantity).
     */
    private int findBestFeasiblePosForDropoff(Route route, Node node, Instance inst) {
        double bestTime = Double.MAX_VALUE;
        int bestPos = -1;
        // Use a lightweight check: just time feasibility with zero dropoff
        for (int p = 0; p <= route.size(); p++) {
            Node depot = inst.getDepot();
            List<RouteStop> stops = route.getStops();
            Node prev = (p == 0) ? depot : stops.get(p - 1).getNode();
            Node next = (p == stops.size()) ? depot : stops.get(p).getNode();
            double detour = inst.getDistance(prev, node) + inst.getDistance(node, next)
                    - inst.getDistance(prev, next);
            double newTime = route.getTotalTime() + detour;
            if (newTime <= inst.getMaxRouteDuration() + 1e-6 && newTime < bestTime) {
                bestTime = newTime;
                bestPos = p;
            }
        }
        return bestPos;
    }
}