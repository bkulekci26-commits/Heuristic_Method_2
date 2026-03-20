import com.gurobi.gurobi.*;
import java.util.*;

/**
 * Transfer Post-Optimization MIP v3 for CTOP-T-Sync.
 *
 * Flow-based formulation inspired by the LOADS model (Cortes & Suzuki, 2020).
 *
 * Key features:
 *   - DEMAND SHARING: at transfer nodes, multiple vehicles can deliver
 *     portions of the node's demand (like a mini cross-dock)
 *   - MULTI-VEHICLE TRANSFERS: multiple vehicles can drop/pick at same node
 *   - REMOVAL + INSERTION: can remove low-profit nodes to free capacity
 *   - DIRECTIONAL SYNC: dropper can arrive at most W after picker;
 *     if dropper arrives first, load waits indefinitely
 *
 * Formulation:
 *   max  Σ profit_u × z_u  -  Σ profit_s × remove_s
 *   s.t. demand sharing, capacity, time, conservation, sync constraints
 */
public class TransferMIP {

    private static final double EPSILON = 1e-4;

    public int optimize(Solution solution) {
        Instance inst = solution.getInstance();
        List<Route> routes = solution.getRoutes();
        for (Route r : routes) r.evaluate();

        List<Node> unserved = solution.getUnservedNodes();
        if (unserved.isEmpty()) return 0;

        int K = routes.size();
        double Q = inst.getMaxCapacity();
        double Tmax = inst.getMaxRouteDuration();
        double W = inst.getSyncWindow();

        // ── Route metrics ──
        double[] loadK = new double[K];
        double[] timeK = new double[K];
        for (int k = 0; k < K; k++) {
            loadK[k] = routes.get(k).getInitialLoad();
            timeK[k] = routes.get(k).getTotalTime();
        }

        // ── Served nodes ──
        List<int[]> served = new ArrayList<>(); // [routeIdx, stopIdx, nodeId]
        List<Double> servedProfit = new ArrayList<>();
        List<Double> servedDemand = new ArrayList<>();
        List<Double> servedRemovalSaving = new ArrayList<>();

        for (int k = 0; k < K; k++) {
            Route route = routes.get(k);
            for (int s = 0; s < route.size(); s++) {
                RouteStop stop = route.getStops().get(s);
                if (stop.isServed()) {
                    served.add(new int[]{k, s, stop.getNode().getId()});
                    servedProfit.add(stop.getNode().getProfit());
                    servedDemand.add(stop.getNode().getDemand());
                    servedRemovalSaving.add(computeRemovalSaving(route, s, inst));
                }
            }
        }
        int S = served.size();

        // ── Arrival times at served nodes ──
        double[][] arrivalAt = new double[S][K];
        boolean[][] visits = new boolean[S][K]; // original visit
        for (int si = 0; si < S; si++) {
            int nodeId = served.get(si)[2];
            for (int k = 0; k < K; k++) {
                int idx = routes.get(k).findStopIndex(nodeId);
                if (idx >= 0) {
                    visits[si][k] = true;
                    arrivalAt[si][k] = routes.get(k).getArrivalTime(idx + 1);
                }
            }
        }

        // ── Detour data for non-visiting vehicles ──
        double[][] detourCostToServed = new double[S][K];
        double[][] detourArrivalAtServed = new double[S][K];
        boolean[][] detourFeasible = new boolean[S][K];

        for (int si = 0; si < S; si++) {
            Node transferNode = inst.getNodeById(served.get(si)[2]);
            for (int k = 0; k < K; k++) {
                if (visits[si][k]) {
                    detourCostToServed[si][k] = 0;
                    detourArrivalAtServed[si][k] = arrivalAt[si][k];
                    detourFeasible[si][k] = true;
                } else {
                    double bestDetour = Double.MAX_VALUE;
                    double bestArr = 0;
                    Route route = routes.get(k);
                    for (int pos = 0; pos <= route.size(); pos++) {
                        double det = computeDetourTime(route, pos, transferNode, inst);
                        if (det < bestDetour) {
                            bestDetour = det;
                            bestArr = estimateArrival(route, pos, transferNode, inst);
                        }
                    }
                    detourCostToServed[si][k] = bestDetour;
                    detourArrivalAtServed[si][k] = bestArr;
                    detourFeasible[si][k] = (timeK[k] + bestDetour <= Tmax + EPSILON);
                }
            }
        }

        // ── Unserved insertion data ──
        int U = unserved.size();
        double[][] insertDetour = new double[U][K];
        for (int ui = 0; ui < U; ui++) {
            Node u = unserved.get(ui);
            for (int k = 0; k < K; k++) {
                double best = Double.MAX_VALUE;
                for (int p = 0; p <= routes.get(k).size(); p++) {
                    double d = computeDetourTime(routes.get(k), p, u, inst);
                    if (d < best) best = d;
                }
                insertDetour[ui][k] = best;
            }
        }

        // ── Diagnostics ──
        int feasiblePairs = 0;
        for (int si = 0; si < S; si++) {
            int origK = served.get(si)[0];
            for (int k2 = 0; k2 < K; k2++) {
                if (k2 == origK) continue;
                if (detourFeasible[si][k2]) {
                    double gTime = arrivalAt[si][origK];
                    double rTime = detourArrivalAtServed[si][k2];
                    if (gTime - rTime <= W + EPSILON) feasiblePairs++;
                }
            }
        }
        System.out.printf("[TPO-MIP] K=%d S=%d U=%d feasible_transfer_pairs=%d%n",
                K, S, U, feasiblePairs);
        for (int k = 0; k < K; k++)
            System.out.printf("[TPO-MIP]   v%d: load=%.0f/%.0f time=%.1f/%.1f%n",
                    k, loadK[k], Q, timeK[k], Tmax);

        int newCustomers = 0;

        try {
            GRBEnv env = new GRBEnv(true);
            env.set(GRB.IntParam.LogToConsole, 0);
            env.set(GRB.DoubleParam.TimeLimit, 60.0);
            env.start();
            GRBModel model = new GRBModel(env);
            model.set(GRB.IntAttr.ModelSense, GRB.MAXIMIZE);

            // ═══ VARIABLES ═══

            // z_u: serve unserved customer u
            GRBVar[] z = new GRBVar[U];
            for (int ui = 0; ui < U; ui++)
                z[ui] = model.addVar(0, 1, unserved.get(ui).getProfit(), GRB.BINARY, "z" + ui);

            // remove_s: remove served customer s
            GRBVar[] rm = new GRBVar[S];
            for (int si = 0; si < S; si++)
                rm[si] = model.addVar(0, 1, -servedProfit.get(si), GRB.BINARY, "rm" + si);

            // assign_uk: assign unserved u to vehicle k
            GRBVar[][] asgn = new GRBVar[U][K];
            for (int ui = 0; ui < U; ui++)
                for (int k = 0; k < K; k++)
                    asgn[ui][k] = model.addVar(0, 1, 0, GRB.BINARY, "a" + ui + "_" + k);

            // detour_sk: vehicle k detours to served node s (for vehicles not originally visiting)
            GRBVar[][] detour = new GRBVar[S][K];
            for (int si = 0; si < S; si++)
                for (int k = 0; k < K; k++) {
                    double ub = (!visits[si][k] && detourFeasible[si][k]) ? 1 : 0;
                    detour[si][k] = model.addVar(0, ub, 0, GRB.BINARY, "dt" + si + "_" + k);
                }

            // share_sk: portion of demand(s) delivered by vehicle k (DEMAND SHARING)
            GRBVar[][] share = new GRBVar[S][K];
            for (int si = 0; si < S; si++)
                for (int k = 0; k < K; k++) {
                    double ub = (visits[si][k] || detourFeasible[si][k]) ? servedDemand.get(si) : 0;
                    share[si][k] = model.addVar(0, ub, 0, GRB.CONTINUOUS, "sh" + si + "_" + k);
                }

            // drop_sk: load dropped by vehicle k at node s
            GRBVar[][] drop = new GRBVar[S][K];
            for (int si = 0; si < S; si++)
                for (int k = 0; k < K; k++) {
                    double ub = visits[si][k] ? Q : 0; // only original visitor can drop
                    drop[si][k] = model.addVar(0, ub, 0, GRB.CONTINUOUS, "dr" + si + "_" + k);
                }

            // pick_sk: load picked up by vehicle k at node s
            GRBVar[][] pick = new GRBVar[S][K];
            for (int si = 0; si < S; si++)
                for (int k = 0; k < K; k++) {
                    double ub = (visits[si][k] || detourFeasible[si][k]) ? Q : 0;
                    pick[si][k] = model.addVar(0, ub, 0, GRB.CONTINUOUS, "pk" + si + "_" + k);
                }

            // ═══ CONSTRAINTS ═══

            // (C1) Assignment: Σ_k asgn_uk = z_u
            for (int ui = 0; ui < U; ui++) {
                GRBLinExpr e = new GRBLinExpr();
                for (int k = 0; k < K; k++) e.addTerm(1, asgn[ui][k]);
                model.addConstr(e, GRB.EQUAL, z[ui], "asgn" + ui);
            }

            // (C2) Demand sharing: Σ_k share_sk = demand_s × (1 - rm_s)
            for (int si = 0; si < S; si++) {
                GRBLinExpr lhs = new GRBLinExpr();
                for (int k = 0; k < K; k++) lhs.addTerm(1, share[si][k]);
                GRBLinExpr rhs = new GRBLinExpr();
                rhs.addConstant(servedDemand.get(si));
                rhs.addTerm(-servedDemand.get(si), rm[si]);
                model.addConstr(lhs, GRB.EQUAL, rhs, "demSh" + si);
            }

            // (C3) Can only deliver if visiting: share_sk ≤ demand_s × canVisit_sk
            for (int si = 0; si < S; si++)
                for (int k = 0; k < K; k++) {
                    if (visits[si][k]) {
                        // Original visitor: share ≤ demand (already set by UB)
                    } else if (detourFeasible[si][k]) {
                        // Detouring: share ≤ demand × detour_sk
                        GRBLinExpr rhs = new GRBLinExpr();
                        rhs.addTerm(servedDemand.get(si), detour[si][k]);
                        model.addConstr(share[si][k], GRB.LESS_EQUAL, rhs, "shLink" + si + "_" + k);
                    }
                    // else: UB=0 already prevents sharing
                }

            // (C4) Pick only if visiting: pick_sk ≤ Q × (visits OR detour)
            for (int si = 0; si < S; si++)
                for (int k = 0; k < K; k++) {
                    if (!visits[si][k] && detourFeasible[si][k]) {
                        GRBLinExpr rhs = new GRBLinExpr();
                        rhs.addTerm(Q, detour[si][k]);
                        model.addConstr(pick[si][k], GRB.LESS_EQUAL, rhs, "pkLink" + si + "_" + k);
                    }
                }

            // (C5) Capacity: load from depot for vehicle k
            // departure_load = Σ share_sk + Σ demand_u × asgn_uk + Σ drop_sk - Σ pick_sk ≤ Q
            // This accounts for: deliveries + drops carried from depot, minus pickups gained mid-route
            for (int k = 0; k < K; k++) {
                GRBLinExpr e = new GRBLinExpr();
                for (int si = 0; si < S; si++) {
                    e.addTerm(1, share[si][k]);
                    e.addTerm(1, drop[si][k]);
                    e.addTerm(-1, pick[si][k]);
                }
                for (int ui = 0; ui < U; ui++)
                    e.addTerm(unserved.get(ui).getDemand(), asgn[ui][k]);
                model.addConstr(e, GRB.LESS_EQUAL, Q, "cap" + k);
            }

            // (C6) Time: time_k + detour costs + insertion costs - removal savings ≤ Tmax
            for (int k = 0; k < K; k++) {
                GRBLinExpr e = new GRBLinExpr();
                e.addConstant(timeK[k]);
                for (int si = 0; si < S; si++) {
                    if (served.get(si)[0] == k)
                        e.addTerm(-servedRemovalSaving.get(si), rm[si]);
                    if (!visits[si][k] && detourFeasible[si][k])
                        e.addTerm(detourCostToServed[si][k], detour[si][k]);
                }
                for (int ui = 0; ui < U; ui++)
                    e.addTerm(insertDetour[ui][k], asgn[ui][k]);
                model.addConstr(e, GRB.LESS_EQUAL, Tmax, "time" + k);
            }

            // (C7) Transfer conservation: Σ_k pick_sk = Σ_k drop_sk at each node
            for (int si = 0; si < S; si++) {
                GRBLinExpr picks = new GRBLinExpr();
                GRBLinExpr drops = new GRBLinExpr();
                for (int k = 0; k < K; k++) {
                    picks.addTerm(1, pick[si][k]);
                    drops.addTerm(1, drop[si][k]);
                }
                model.addConstr(picks, GRB.EQUAL, drops, "conserve" + si);
            }

            // (C8) Can't drop at removed nodes
            for (int si = 0; si < S; si++) {
                int origK = served.get(si)[0];
                GRBLinExpr rhs = new GRBLinExpr();
                rhs.addConstant(Q);
                rhs.addTerm(-Q, rm[si]);
                model.addConstr(drop[si][origK], GRB.LESS_EQUAL, rhs, "noDrRm" + si);
            }

            // (C9) A vehicle can't both drop AND pick at the same node
            for (int si = 0; si < S; si++)
                for (int k = 0; k < K; k++) {
                    boolean canDrop = visits[si][k];
                    boolean canPick = visits[si][k] || detourFeasible[si][k];
                    if (canDrop && canPick) {
                        GRBVar isDr = model.addVar(0, 1, 0, GRB.BINARY, "idr" + si + "_" + k);
                        GRBVar isPk = model.addVar(0, 1, 0, GRB.BINARY, "ipk" + si + "_" + k);
                        GRBLinExpr dl = new GRBLinExpr(); dl.addTerm(Q, isDr);
                        model.addConstr(drop[si][k], GRB.LESS_EQUAL, dl, "dl" + si + "_" + k);
                        GRBLinExpr pl = new GRBLinExpr(); pl.addTerm(Q, isPk);
                        model.addConstr(pick[si][k], GRB.LESS_EQUAL, pl, "pl" + si + "_" + k);
                        GRBLinExpr mx = new GRBLinExpr(); mx.addTerm(1, isDr); mx.addTerm(1, isPk);
                        model.addConstr(mx, GRB.LESS_EQUAL, 1, "mx" + si + "_" + k);
                    }
                }

            // (C10) Sync: block infeasible (dropper,picker) pairs
            // Dropper can arrive at most W AFTER picker (if dropper is late, picker waits ≤ W)
            // Load waits indefinitely if dropper arrives first
            for (int si = 0; si < S; si++)
                for (int k1 = 0; k1 < K; k1++) {
                    if (!visits[si][k1]) continue; // only original visitor can drop
                    double dropTime = arrivalAt[si][k1];
                    for (int k2 = 0; k2 < K; k2++) {
                        if (k2 == k1) continue;
                        if (!visits[si][k2] && !detourFeasible[si][k2]) continue;
                        double pickTime = visits[si][k2]
                                ? arrivalAt[si][k2]
                                : detourArrivalAtServed[si][k2];
                        // Sync: dropTime - pickTime ≤ W
                        if (dropTime - pickTime > W + EPSILON) {
                            // Block this pair: drop[si][k1] and pick[si][k2] can't both be > 0
                            GRBLinExpr se = new GRBLinExpr();
                            se.addTerm(1.0 / Q, drop[si][k1]);
                            se.addTerm(1.0 / Q, pick[si][k2]);
                            model.addConstr(se, GRB.LESS_EQUAL, 1, "sync" + si + "_" + k1 + "_" + k2);
                        }
                    }
                }

            // (C11) Limits
            for (int k = 0; k < K; k++) {
                GRBLinExpr rmE = new GRBLinExpr();
                for (int si = 0; si < S; si++) if (served.get(si)[0] == k) rmE.addTerm(1, rm[si]);
                model.addConstr(rmE, GRB.LESS_EQUAL, 2, "maxRm" + k);

                GRBLinExpr insE = new GRBLinExpr();
                for (int ui = 0; ui < U; ui++) insE.addTerm(1, asgn[ui][k]);
                model.addConstr(insE, GRB.LESS_EQUAL, 2, "maxIns" + k);

                GRBLinExpr dtE = new GRBLinExpr();
                for (int si = 0; si < S; si++)
                    if (!visits[si][k] && detourFeasible[si][k]) dtE.addTerm(1, detour[si][k]);
                model.addConstr(dtE, GRB.LESS_EQUAL, 2, "maxDt" + k);
            }

            // (C12) Net profit must be positive
            {
                GRBLinExpr net = new GRBLinExpr();
                for (int ui = 0; ui < U; ui++) net.addTerm(unserved.get(ui).getProfit(), z[ui]);
                for (int si = 0; si < S; si++) net.addTerm(-servedProfit.get(si), rm[si]);
                model.addConstr(net, GRB.GREATER_EQUAL, 1, "netPos");
            }

            // ═══ SOLVE ═══
            model.optimize();

            int status = model.get(GRB.IntAttr.Status);
            if (status == GRB.OPTIMAL || status == GRB.SUBOPTIMAL) {
                double objVal = model.get(GRB.DoubleAttr.ObjVal);
                System.out.printf("[TPO-MIP] Objective: %.1f%n", objVal);

                if (objVal > EPSILON) {
                    // ── Extract results ──
                    List<int[]> removals = new ArrayList<>();
                    for (int si = 0; si < S; si++)
                        if (rm[si].get(GRB.DoubleAttr.X) > 0.5) {
                            removals.add(served.get(si));
                            System.out.printf("[TPO-MIP] Remove node %d (p=%.0f,d=%.0f) from v%d%n",
                                    served.get(si)[2], servedProfit.get(si), servedDemand.get(si), served.get(si)[0]);
                        }

                    List<int[]> insertions = new ArrayList<>();
                    for (int ui = 0; ui < U; ui++)
                        if (z[ui].get(GRB.DoubleAttr.X) > 0.5)
                            for (int k = 0; k < K; k++)
                                if (asgn[ui][k].get(GRB.DoubleAttr.X) > 0.5) {
                                    insertions.add(new int[]{ui, k});
                                    System.out.printf("[TPO-MIP] Insert node %d (p=%.0f,d=%.0f) in v%d%n",
                                            unserved.get(ui).getId(), unserved.get(ui).getProfit(),
                                            unserved.get(ui).getDemand(), k);
                                    break;
                                }

                    // Extract transfers
                    List<double[]> transfers = new ArrayList<>();
                    for (int si = 0; si < S; si++)
                        for (int k1 = 0; k1 < K; k1++) {
                            double dropVal = drop[si][k1].get(GRB.DoubleAttr.X);
                            if (dropVal > EPSILON)
                                for (int k2 = 0; k2 < K; k2++) {
                                    if (k2 == k1) continue;
                                    double pickVal = pick[si][k2].get(GRB.DoubleAttr.X);
                                    if (pickVal > EPSILON) {
                                        transfers.add(new double[]{si, k1, k2, Math.min(dropVal, pickVal)});
                                        System.out.printf("[TPO-MIP] Transfer: v%d drops %.0f at node %d → v%d picks up%n",
                                                k1, Math.min(dropVal, pickVal), served.get(si)[2], k2);
                                    }
                                }
                        }

                    // Extract demand sharing info
                    for (int si = 0; si < S; si++) {
                        if (rm[si].get(GRB.DoubleAttr.X) > 0.5) continue;
                        int origK = served.get(si)[0];
                        for (int k = 0; k < K; k++) {
                            double shareVal = share[si][k].get(GRB.DoubleAttr.X);
                            if (shareVal > EPSILON && k != origK) {
                                System.out.printf("[TPO-MIP] Demand sharing: node %d demand=%.0f → v%d delivers %.0f, v%d delivers %.0f%n",
                                        served.get(si)[2], servedDemand.get(si), origK,
                                        share[si][origK].get(GRB.DoubleAttr.X), k, shareVal);
                            }
                        }
                    }

                    // ── Apply to solution ──

                    // Step 1: Removals (backwards by index)
                    removals.sort((a, b) -> a[0] != b[0] ? b[0] - a[0] : b[1] - a[1]);
                    for (int[] r : removals) {
                        Route route = routes.get(r[0]);
                        int idx = route.findStopIndex(r[2]);
                        if (idx >= 0) { route.removeStop(idx); route.evaluate(); }
                    }

                    // Step 2: Insertions
                    for (int[] ins : insertions) {
                        Node u = unserved.get(ins[0]);
                        Route route = routes.get(ins[1]);
                        int pos = findBestPos(route, u, inst);
                        if (pos >= 0) {
                            route.insertStop(pos, RouteStop.serve(u));
                            route.evaluate();
                            newCustomers++;
                        }
                    }

                    // Step 3: Transfer modifications
                    for (double[] trf : transfers) {
                        int si = (int) trf[0];
                        int giverK = (int) trf[1];
                        int recvK = (int) trf[2];
                        double qty = trf[3];
                        int nid = served.get(si)[2];
                        Node tn = inst.getNodeById(nid);

                        // Giver: add dropoff
                        Route giver = routes.get(giverK);
                        int gi = giver.findStopIndex(nid);
                        if (gi >= 0) {
                            RouteStop old = giver.getStops().get(gi);
                            giver.getStops().set(gi, old.isServed()
                                    ? RouteStop.serveAndDropoff(tn, qty) : RouteStop.dropoff(tn, qty));
                            giver.evaluate();
                        }

                        // Receiver: add pickup (may need to insert detour stop)
                        Route recv = routes.get(recvK);
                        int ri = recv.findStopIndex(nid);
                        if (ri >= 0) {
                            RouteStop old = recv.getStops().get(ri);
                            recv.getStops().set(ri, old.isServed()
                                    ? RouteStop.serveAndPickup(tn, qty) : RouteStop.pickup(tn, qty));
                        } else {
                            int pos = findBestPos(recv, tn, inst);
                            if (pos >= 0) recv.insertStop(pos, RouteStop.pickup(tn, qty));
                        }
                        recv.evaluate();

                        solution.addTransfer(new Transfer(nid,
                                giver.getVehicleId(), recv.getVehicleId(), qty));
                    }

                    for (Route r : routes) r.evaluate();
                    System.out.printf("[TPO-MIP] Applied: %d removals, %d insertions, %d transfers, net=+%.0f%n",
                            removals.size(), newCustomers, transfers.size(), objVal);
                } else {
                    System.out.println("[TPO-MIP] No profitable configuration");
                }
            } else if (status == GRB.INFEASIBLE) {
                System.out.println("[TPO-MIP] Infeasible");
                model.computeIIS();
                for (GRBConstr c : model.getConstrs())
                    if (c.get(GRB.IntAttr.IISConstr) > 0)
                        System.out.println("  IIS: " + c.get(GRB.StringAttr.ConstrName));
            } else {
                System.out.printf("[TPO-MIP] Status: %d%n", status);
            }

            model.dispose();
            env.dispose();
        } catch (GRBException e) {
            System.err.println("[TPO-MIP] Error: " + e.getMessage());
            e.printStackTrace();
        }
        return newCustomers;
    }

    private double computeDetourTime(Route route, int pos, Node node, Instance inst) {
        Node depot = inst.getDepot();
        List<RouteStop> stops = route.getStops();
        Node prev = (pos == 0) ? depot : (stops.isEmpty() ? depot : stops.get(Math.min(pos - 1, stops.size() - 1)).getNode());
        Node next = (pos >= stops.size()) ? depot : stops.get(pos).getNode();
        if (stops.isEmpty()) { prev = depot; next = depot; }
        else if (pos == 0) { prev = depot; next = stops.get(0).getNode(); }
        else if (pos >= stops.size()) { prev = stops.get(stops.size() - 1).getNode(); next = depot; }
        else { prev = stops.get(pos - 1).getNode(); next = stops.get(pos).getNode(); }
        return inst.getDistance(prev, node) + inst.getDistance(node, next) - inst.getDistance(prev, next);
    }

    private double computeRemovalSaving(Route route, int idx, Instance inst) {
        Node depot = inst.getDepot();
        List<RouteStop> stops = route.getStops();
        Node n = stops.get(idx).getNode();
        Node prev = (idx == 0) ? depot : stops.get(idx - 1).getNode();
        Node next = (idx == stops.size() - 1) ? depot : stops.get(idx + 1).getNode();
        return inst.getDistance(prev, n) + inst.getDistance(n, next) - inst.getDistance(prev, next);
    }

    private double estimateArrival(Route route, int pos, Node node, Instance inst) {
        route.evaluate();
        Node depot = inst.getDepot();
        Node prev = (pos == 0) ? depot : route.getStops().get(pos - 1).getNode();
        double prevArr = (pos == 0) ? 0 : route.getArrivalTime(pos);
        return prevArr + inst.getDistance(prev, node);
    }

    private int findBestPos(Route route, Node node, Instance inst) {
        double best = Double.MAX_VALUE;
        int bestP = -1;
        for (int p = 0; p <= route.size(); p++) {
            double d = computeDetourTime(route, p, node, inst);
            if (d < best) { best = d; bestP = p; }
        }
        return bestP;
    }
}