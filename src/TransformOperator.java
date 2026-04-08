import java.util.*;

/**
 * Transfer creation heuristic for CTOP-T-Sync, adapted from
 * Aguayo et al. (2025) "The vehicle routing problem with transfers".
 *
 * Instead of a post-optimization MIP, this operator directly searches for
 * profitable transfer opportunities by testing on actual route copies.
 *
 * Transfer archetype:
 *   Vehicle k1 removes low-profit node s, gains spare depot capacity.
 *   k1 drops spare load at transfer node j (already in k1's route).
 *   Vehicle k2 detours to j, picks up the load.
 *   k2 now has extra capacity downstream → inserts high-profit unserved node u.
 *   Net profit = profit(u) - profit(s) must be positive.
 *
 * Sync constraint (directional): t_dropper(k1) - t_picker(k2) ≤ W
 */
public class TransformOperator {

    /**
     * Attempt to find and apply a profitable transfer.
     * Returns the number of transfers created (0 or 1+).
     */
    public int optimize(Solution solution) {
        Instance inst = solution.getInstance();
        List<Route> routes = solution.getRoutes();
        for (Route r : routes) r.evaluate();

        int K = routes.size();
        double Q = inst.getMaxCapacity();
        double Tmax = inst.getMaxRouteDuration();
        double W = inst.getSyncWindow();

        // Collect unserved nodes sorted by profit/demand ratio (best first)
        List<Node> unserved = solution.getUnservedNodes();
        unserved.sort((a, b) -> Double.compare(
                b.getProfit() / Math.max(b.getDemand(), 1.0),
                a.getProfit() / Math.max(a.getDemand(), 1.0)));

        // ── Search for the best single transfer move ──
        TransferMove bestMove = null;

        // Strategy 1: Direct insertion without transfer (ALNS may have missed some)
        bestMove = searchDirectInsertions(solution, unserved, inst);

        // Strategy 2: Single-vehicle swap (remove low, insert high)
        TransferMove swapMove = searchSingleVehicleSwaps(solution, unserved, inst);
        if (swapMove != null && (bestMove == null || swapMove.netProfit > bestMove.netProfit)) {
            bestMove = swapMove;
        }

        // Strategy 3: Cross-vehicle transfer (the Aguayo-style mechanism)
        TransferMove transferMove = searchCrossVehicleTransfers(solution, unserved, inst);
        if (transferMove != null && (bestMove == null || transferMove.netProfit > bestMove.netProfit)) {
            bestMove = transferMove;
        }

        // ── Apply the best move found ──
        if (bestMove != null) {
            return applyMove(solution, bestMove, inst);
        }

        return 0;
    }

    // ══════════════════════════════════════════════════════════════
    // STRATEGY 1: Direct insertions (no transfer needed)
    // ══════════════════════════════════════════════════════════════

    private TransferMove searchDirectInsertions(Solution solution, List<Node> unserved, Instance inst) {
        TransferMove best = null;
        for (Node u : unserved) {
            for (Route route : solution.getRoutes()) {
                Route copy = new Route(route);
                int pos = findBestFeasiblePos(copy, u, inst);
                if (pos >= 0) {
                    TransferMove m = new TransferMove();
                    m.type = MoveType.DIRECT_INSERT;
                    m.insertNode = u;
                    m.insertVehicle = route.getVehicleId();
                    m.insertPos = pos;
                    m.netProfit = u.getProfit();
                    if (best == null || m.netProfit > best.netProfit) best = m;
                }
            }
        }
        return best;
    }

    // ══════════════════════════════════════════════════════════════
    // STRATEGY 2: Single-vehicle swap (remove s, insert u in same route)
    // ══════════════════════════════════════════════════════════════

    private TransferMove searchSingleVehicleSwaps(Solution solution, List<Node> unserved, Instance inst) {
        TransferMove best = null;

        for (Route route : solution.getRoutes()) {
            for (int si = 0; si < route.size(); si++) {
                RouteStop stop = route.getStops().get(si);
                if (!stop.isServed()) continue;
                Node sNode = stop.getNode();

                // Create a copy with s removed
                Route copy = new Route(route);
                copy.removeStop(si);
                copy.evaluate();

                for (Node u : unserved) {
                    if (u.getProfit() <= sNode.getProfit()) continue; // must improve

                    int pos = findBestFeasiblePos(copy, u, inst);
                    if (pos >= 0) {
                        double net = u.getProfit() - sNode.getProfit();
                        if (best == null || net > best.netProfit) {
                            best = new TransferMove();
                            best.type = MoveType.SINGLE_SWAP;
                            best.removeNode = sNode;
                            best.removeVehicle = route.getVehicleId();
                            best.removeIdx = si;
                            best.insertNode = u;
                            best.insertVehicle = route.getVehicleId();
                            best.insertPos = pos;
                            best.netProfit = net;
                        }
                    }
                }
            }
        }
        return best;
    }

    // ══════════════════════════════════════════════════════════════
    // STRATEGY 3: Cross-vehicle transfer (Aguayo Transform)
    // ══════════════════════════════════════════════════════════════

    /**
     * Search for profitable cross-vehicle transfers:
     *   - Remove node s from k_drop (dropper), freeing depot capacity
     *   - k_drop drops freed load at transfer node j (already in k_drop's route)
     *   - k_pick detours to j, picks up the load
     *   - k_pick inserts unserved node u using the gained capacity
     *   - Net profit = profit(u) - profit(s)
     */
    private TransferMove searchCrossVehicleTransfers(Solution solution, List<Node> unserved, Instance inst) {
        List<Route> routes = solution.getRoutes();
        int K = routes.size();
        double Q = inst.getMaxCapacity();
        double W = inst.getSyncWindow();
        double Tmax = inst.getMaxRouteDuration();

        TransferMove best = null;
        int totalCombinations = 0;
        int syncFailures = 0;
        int capacityFailures = 0;
        int timeFailures = 0;

        // For each dropper route k_drop
        for (int kd = 0; kd < K; kd++) {
            Route kDrop = routes.get(kd);

            // For each removable node s in k_drop
            for (int si = 0; si < kDrop.size(); si++) {
                RouteStop sStop = kDrop.getStops().get(si);
                if (!sStop.isServed()) continue;
                Node sNode = sStop.getNode();
                double freedDemand = sNode.getDemand();

                // Create k_drop copy with s removed
                Route kDropCopy = new Route(kDrop);
                kDropCopy.removeStop(si);
                kDropCopy.evaluate();

                double dropperSpare = Q - kDropCopy.getInitialLoad(); // freed capacity at depot
                if (dropperSpare < 1) continue;

                double dropQty = Math.min(freedDemand, dropperSpare);

                // For each transfer point j in k_drop's remaining route
                for (int ji = 0; ji < kDropCopy.size(); ji++) {
                    Node jNode = kDropCopy.getStops().get(ji).getNode();

                    // k_drop drops dropQty at j → check time feasibility
                    // (drop doesn't add time if j is already in route, just adds load)
                    // Check capacity: k_drop needs to carry dropQty extra from depot to j
                    // arcLoads[0..ji] all increase by dropQty
                    boolean dropFeasible = true;
                    Route kDropWithDrop = new Route(kDropCopy);
                    RouteStop origStop = kDropWithDrop.getStops().get(ji);
                    kDropWithDrop.getStops().set(ji,
                            origStop.isServed()
                                    ? RouteStop.serveAndDropoff(jNode, dropQty)
                                    : RouteStop.dropoff(jNode, dropQty));
                    kDropWithDrop.evaluate();
                    if (!kDropWithDrop.isFeasible()) continue;

                    double dropperArrival = kDropWithDrop.getArrivalTimeAtNode(jNode.getId());

                    // For each picker route k_pick
                    for (int kp = 0; kp < K; kp++) {
                        if (kp == kd) continue;
                        Route kPick = routes.get(kp);

                        // Try ALL feasible pickup positions, checking sync for each
                        for (int pp = 0; pp <= kPick.size(); pp++) {
                            Route kPickCopy = new Route(kPick);
                            kPickCopy.insertStop(pp, RouteStop.pickup(jNode, dropQty));
                            kPickCopy.evaluate();
                            if (!kPickCopy.isFeasible()) { timeFailures++; continue; }

                            // Check sync: picker must not wait more than W for dropper
                            totalCombinations++;
                            double pickerArrival = kPickCopy.getArrivalTimeAtNode(jNode.getId());
                            if (dropperArrival - pickerArrival > W + 1e-6) { syncFailures++; continue; }

                            // Sync passes! Now try inserting unserved node u
                            for (Node u : unserved) {
                                if (u.getDemand() > dropQty) continue;

                                double net = u.getProfit() - sNode.getProfit();
                                if (net <= 0) continue;
                                if (best != null && net <= best.netProfit) continue;

                                int uPos = findBestFeasiblePos(kPickCopy, u, inst);
                                if (uPos >= 0) {
                                    best = new TransferMove();
                                    best.type = MoveType.CROSS_TRANSFER;
                                    best.removeNode = sNode;
                                    best.removeVehicle = kDrop.getVehicleId();
                                    best.removeIdx = si;
                                    best.transferNode = jNode;
                                    best.dropQty = dropQty;
                                    best.dropperVehicle = kDrop.getVehicleId();
                                    best.pickerVehicle = kPick.getVehicleId();
                                    best.insertNode = u;
                                    best.insertVehicle = kPick.getVehicleId();
                                    best.insertPos = uPos;
                                    best.pickupPos = pp;
                                    best.netProfit = net;
                                }
                            }
                        }
                    }
                }
            }
        }

        System.out.printf("[Transform] Cross-transfer search: %d sync-checked, %d sync-failed, %d time-failed, best=%s%n",
                totalCombinations, syncFailures, timeFailures,
                best != null ? String.format("net=+%.0f", best.netProfit) : "none");

        return best;
    }

    // ══════════════════════════════════════════════════════════════
    // APPLY MOVE
    // ══════════════════════════════════════════════════════════════

    private int applyMove(Solution solution, TransferMove move, Instance inst) {
        Solution backup = new Solution(solution);
        List<Route> routes = solution.getRoutes();

        System.out.printf("[Transform] Applying %s move (net=+%.0f)%n", move.type, move.netProfit);

        switch (move.type) {
            case DIRECT_INSERT: {
                Route route = getRouteById(routes, move.insertVehicle);
                int pos = findBestFeasiblePos(route, move.insertNode, inst);
                if (pos >= 0) {
                    route.insertStop(pos, RouteStop.serve(move.insertNode));
                    route.evaluate();
                    System.out.printf("[Transform] Inserted node %d (p=%.0f) in v%d%n",
                            move.insertNode.getId(), move.insertNode.getProfit(), move.insertVehicle);
                }
                break;
            }

            case SINGLE_SWAP: {
                Route route = getRouteById(routes, move.removeVehicle);
                // Remove
                int rmIdx = route.findStopIndex(move.removeNode.getId());
                if (rmIdx >= 0) {
                    route.removeStop(rmIdx);
                    route.evaluate();
                    System.out.printf("[Transform] Removed node %d (p=%.0f) from v%d%n",
                            move.removeNode.getId(), move.removeNode.getProfit(), move.removeVehicle);
                }
                // Insert
                int pos = findBestFeasiblePos(route, move.insertNode, inst);
                if (pos >= 0) {
                    route.insertStop(pos, RouteStop.serve(move.insertNode));
                    route.evaluate();
                    System.out.printf("[Transform] Inserted node %d (p=%.0f) in v%d%n",
                            move.insertNode.getId(), move.insertNode.getProfit(), move.insertVehicle);
                }
                break;
            }

            case CROSS_TRANSFER: {
                Route kDrop = getRouteById(routes, move.dropperVehicle);
                Route kPick = getRouteById(routes, move.pickerVehicle);

                // 1. Remove s from dropper
                int rmIdx = kDrop.findStopIndex(move.removeNode.getId());
                if (rmIdx >= 0) {
                    kDrop.removeStop(rmIdx);
                    kDrop.evaluate();
                    System.out.printf("[Transform] Removed node %d (p=%.0f) from v%d%n",
                            move.removeNode.getId(), move.removeNode.getProfit(), move.dropperVehicle);
                }

                // 2. Add dropoff at transfer node j in dropper's route
                int jIdx = kDrop.findStopIndex(move.transferNode.getId());
                if (jIdx >= 0) {
                    RouteStop orig = kDrop.getStops().get(jIdx);
                    double existDrop = orig.isDropoff() ? orig.getDropoffQty() : 0;
                    kDrop.getStops().set(jIdx, orig.isServed()
                            ? RouteStop.serveAndDropoff(move.transferNode, existDrop + move.dropQty)
                            : RouteStop.dropoff(move.transferNode, existDrop + move.dropQty));
                    kDrop.evaluate();
                }

                // 3. Add pickup at transfer node j in picker's route (sync-aware)
                double dropArrival = kDrop.getArrivalTimeAtNode(move.transferNode.getId());
                int pickPos = findSyncAwarePickupPos(kPick, move.transferNode, move.dropQty,
                        dropArrival, inst.getSyncWindow(), inst);
                if (pickPos >= 0) {
                    kPick.insertStop(pickPos, RouteStop.pickup(move.transferNode, move.dropQty));
                    kPick.evaluate();
                }

                // 4. Insert unserved node u in picker's route
                int uPos = findBestFeasiblePos(kPick, move.insertNode, inst);
                if (uPos >= 0) {
                    kPick.insertStop(uPos, RouteStop.serve(move.insertNode));
                    kPick.evaluate();
                    System.out.printf("[Transform] Inserted node %d (p=%.0f) in v%d%n",
                            move.insertNode.getId(), move.insertNode.getProfit(), move.pickerVehicle);
                }

                // 5. Register transfer
                solution.addTransfer(new Transfer(
                        move.transferNode.getId(),
                        move.dropperVehicle,
                        move.pickerVehicle,
                        move.dropQty));

                System.out.printf("[Transform] Transfer: v%d drops %.0f at node %d → v%d picks up%n",
                        move.dropperVehicle, move.dropQty, move.transferNode.getId(), move.pickerVehicle);
                break;
            }
        }

        // Validate
        for (Route r : routes) r.evaluate();
        if (!solution.isFeasible() || solution.getTotalProfit() <= backup.getTotalProfit()) {
            if (!solution.isFeasible()) {
                Solution.FeasibilityReport fr = solution.checkFeasibility();
                System.out.println("[Transform] Post-apply infeasible:");
                for (String v : fr.getViolations()) System.out.println("  → " + v);
            }
            System.out.printf("[Transform] Rollback (profit %.0f ≤ %.0f)%n",
                    solution.getTotalProfit(), backup.getTotalProfit());
            solution.restoreFrom(backup);
            for (Route r : solution.getRoutes()) r.evaluate();
            return 0;
        }

        System.out.printf("[Transform] SUCCESS: profit %.0f → %.0f (+%.0f)%n",
                backup.getTotalProfit(), solution.getTotalProfit(),
                solution.getTotalProfit() - backup.getTotalProfit());
        return 1;
    }

    // ══════════════════════════════════════════════════════════════
    // HELPERS
    // ══════════════════════════════════════════════════════════════

    /** Find best feasible position to insert a serve stop */
    private int findBestFeasiblePos(Route route, Node node, Instance inst) {
        double bestTime = Double.MAX_VALUE;
        int bestP = -1;
        for (int p = 0; p <= route.size(); p++) {
            route.insertStop(p, RouteStop.serve(node));
            route.evaluate();
            if (route.isFeasible() && route.getTotalTime() < bestTime) {
                bestTime = route.getTotalTime();
                bestP = p;
            }
            route.removeStop(p);
            route.evaluate();
        }
        return bestP;
    }

    /** Find best feasible position for a pickup stop that also satisfies sync */
    private int findSyncAwarePickupPos(Route route, Node node, double qty,
                                       double dropperArrival, double W, Instance inst) {
        double bestTime = Double.MAX_VALUE;
        int bestP = -1;
        for (int p = 0; p <= route.size(); p++) {
            route.insertStop(p, RouteStop.pickup(node, qty));
            route.evaluate();
            if (route.isFeasible()) {
                double pickerArrival = route.getArrivalTimeAtNode(node.getId());
                // Sync: dropper can arrive at most W after picker
                if (dropperArrival - pickerArrival <= W + 1e-6) {
                    if (route.getTotalTime() < bestTime) {
                        bestTime = route.getTotalTime();
                        bestP = p;
                    }
                }
            }
            route.removeStop(p);
            route.evaluate();
        }
        return bestP;
    }

    /** Find best feasible position for a pickup stop */
    private int findBestPickupPos(Route route, Node node, double qty, Instance inst) {
        double bestTime = Double.MAX_VALUE;
        int bestP = -1;
        for (int p = 0; p <= route.size(); p++) {
            route.insertStop(p, RouteStop.pickup(node, qty));
            route.evaluate();
            if (route.isFeasible() && route.getTotalTime() < bestTime) {
                bestTime = route.getTotalTime();
                bestP = p;
            }
            route.removeStop(p);
            route.evaluate();
        }
        return bestP;
    }

    private Route getRouteById(List<Route> routes, int vehicleId) {
        for (Route r : routes) if (r.getVehicleId() == vehicleId) return r;
        throw new IllegalArgumentException("Vehicle " + vehicleId + " not found");
    }

    // ══════════════════════════════════════════════════════════════
    // DATA STRUCTURES
    // ══════════════════════════════════════════════════════════════

    private enum MoveType { DIRECT_INSERT, SINGLE_SWAP, CROSS_TRANSFER }

    private static class TransferMove {
        MoveType type;
        Node removeNode;
        int removeVehicle;
        int removeIdx;
        Node insertNode;
        int insertVehicle;
        int insertPos;
        // Transfer-specific
        Node transferNode;
        double dropQty;
        int dropperVehicle;
        int pickerVehicle;
        int pickupPos;
        double netProfit;
    }
}