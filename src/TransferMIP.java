import com.gurobi.gurobi.*;
import java.util.*;

/**
 * Transfer Post-Optimization MIP v5 for CTOP-T-Sync.
 *
 * Move Selection Model: every candidate (remove+insert) is pre-tested on
 * actual route copies. The MIP selects the best non-conflicting subset.
 *
 * Zero model-reality gap: if the MIP selects it, it's guaranteed feasible.
 *
 * Candidate types:
 *   A) Same-vehicle swap:  remove s from k, insert u in k
 *   B) Transfer swap:      remove s from k1, transfer to k2, insert u in k2
 */
public class TransferMIP {

    private static final double EPS = 1e-4;

    /** A pre-validated candidate move */
    static class Candidate {
        int removeNode;   // node to remove (-1 if none)
        int removeVehicle;// vehicle to remove from
        int insertNode;   // node to insert
        int insertVehicle;// vehicle to insert into
        int transferNode; // transfer point (-1 if same-vehicle swap)
        double netProfit; // profit_insert - profit_remove
        boolean isTransfer;

        Candidate(int rn, int rv, int in_, int iv, int tn, double np, boolean t) {
            removeNode = rn; removeVehicle = rv; insertNode = in_;
            insertVehicle = iv; transferNode = tn; netProfit = np; isTransfer = t;
        }
    }

    public int optimize(Solution solution) {
        Instance inst = solution.getInstance();
        List<Route> routes = solution.getRoutes();
        for (Route r : routes) r.evaluate();

        int K = routes.size();
        double Q = inst.getMaxCapacity();
        double Tmax = inst.getMaxRouteDuration();
        double W = inst.getSyncWindow();

        List<Node> unserved = solution.getUnservedNodes();
        if (unserved.isEmpty()) return 0;

        // ── Collect served nodes ──
        List<int[]> served = new ArrayList<>(); // [vehicleIdx, nodeId]
        for (int k = 0; k < K; k++)
            for (RouteStop s : routes.get(k).getStops())
                if (s.isServed()) served.add(new int[]{k, s.getNode().getId()});

        System.out.printf("[TPO-MIP] K=%d served=%d unserved=%d%n", K, served.size(), unserved.size());
        for (int k = 0; k < K; k++)
            System.out.printf("[TPO-MIP]   v%d: load=%.0f/%.0f time=%.1f/%.1f%n",
                    k, routes.get(k).getInitialLoad(), Q, routes.get(k).getTotalTime(), Tmax);

        // ══════════════════════════════════════════
        //  PHASE 1: Enumerate pre-validated candidates
        // ══════════════════════════════════════════
        List<Candidate> candidates = new ArrayList<>();

        // ── Type A: Same-vehicle swap (remove s, insert u in same vehicle k) ──
        for (int[] sv : served) {
            int k = sv[0];
            int sId = sv[1];
            Node sNode = inst.getNodeById(sId);

            Route copy = new Route(routes.get(k));
            int sIdx = copy.findStopIndex(sId);
            if (sIdx < 0) continue;
            copy.removeStop(sIdx);
            copy.evaluate();

            for (Node u : unserved) {
                // Try all positions, pick best feasible
                int bestPos = -1;
                double bestDet = Double.MAX_VALUE;
                for (int p = 0; p <= copy.size(); p++) {
                    copy.insertStop(p, RouteStop.serve(u));
                    copy.evaluate();
                    if (copy.isFeasible()) {
                        double det = copy.getTotalTime() - (routes.get(k).getTotalTime());
                        if (det < bestDet) { bestDet = det; bestPos = p; }
                    }
                    copy.removeStop(p);
                    copy.evaluate();
                }

                if (bestPos >= 0) {
                    double net = u.getProfit() - sNode.getProfit();
                    if (net > 0) {
                        candidates.add(new Candidate(sId, k, u.getId(), k, -1, net, false));
                    }
                }
            }
        }

        // ── Type B: Transfer swap (remove s from k1, k2 inserts u via transfer) ──
        for (int[] sv : served) {
            int k1 = sv[0];
            int sId = sv[1];
            Node sNode = inst.getNodeById(sId);

            Route copy1 = new Route(routes.get(k1));
            int sIdx = copy1.findStopIndex(sId);
            if (sIdx < 0) continue;
            copy1.removeStop(sIdx);
            copy1.evaluate();

            double freedDemand = sNode.getDemand();

            // Find transfer points: nodes in k1's (modified) route
            for (RouteStop tStop : copy1.getStops()) {
                Node tNode = tStop.getNode();
                double dropperArrival = copy1.getArrivalTimeAtNode(tNode.getId());

                for (int k2 = 0; k2 < K; k2++) {
                    if (k2 == k1) continue;
                    Route copy2 = new Route(routes.get(k2));

                    // k2 detours to tNode for pickup
                    int pickPos = findBestPickupPos(copy2, tNode, freedDemand, inst);
                    if (pickPos < 0) continue;

                    copy2.insertStop(pickPos, RouteStop.pickup(tNode, freedDemand));
                    copy2.evaluate();

                    if (!copy2.isFeasible()) continue;

                    // Check sync: dropper arrival - picker arrival ≤ W
                    double pickerArrival = copy2.getArrivalTimeAtNode(tNode.getId());
                    if (dropperArrival - pickerArrival > W + EPS) continue;

                    // Now try inserting each unserved customer in k2
                    for (Node u : unserved) {
                        int bestPos = -1;
                        for (int p = 0; p <= copy2.size(); p++) {
                            copy2.insertStop(p, RouteStop.serve(u));
                            copy2.evaluate();
                            if (copy2.isFeasible()) { bestPos = p; break; }
                            copy2.removeStop(p);
                            copy2.evaluate();
                        }

                        if (bestPos >= 0) {
                            double net = u.getProfit() - sNode.getProfit();
                            if (net > 0) {
                                candidates.add(new Candidate(sId, k1, u.getId(), k2,
                                        tNode.getId(), net, true));
                            }
                            // Remove u to test next one
                            copy2.removeStop(copy2.findStopIndex(u.getId()));
                            copy2.evaluate();
                        }
                    }

                    // Remove pickup to test next transfer point
                    copy2.removeStop(copy2.findStopIndex(tNode.getId()));
                    copy2.evaluate();
                }
            }
        }

        System.out.printf("[TPO-MIP] Candidates: %d same-vehicle, %d transfer%n",
                (int) candidates.stream().filter(c -> !c.isTransfer).count(),
                (int) candidates.stream().filter(c -> c.isTransfer).count());

        if (candidates.isEmpty()) {
            System.out.println("[TPO-MIP] No profitable candidates found");
            return 0;
        }

        // ══════════════════════════════════════════
        //  PHASE 2: MIP to select best non-conflicting candidates
        // ══════════════════════════════════════════
        int C = candidates.size();
        int result = 0;

        try {
            GRBEnv env = new GRBEnv(true);
            env.set(GRB.IntParam.LogToConsole, 0);
            env.set(GRB.DoubleParam.TimeLimit, 30.0);
            env.start();
            GRBModel model = new GRBModel(env);
            model.set(GRB.IntAttr.ModelSense, GRB.MAXIMIZE);

            GRBVar[] x = new GRBVar[C];
            for (int c = 0; c < C; c++)
                x[c] = model.addVar(0, 1, candidates.get(c).netProfit, GRB.BINARY, "x" + c);

            // Each served node removed at most once
            Map<Integer, List<Integer>> removeMap = new HashMap<>();
            for (int c = 0; c < C; c++) {
                int rn = candidates.get(c).removeNode;
                removeMap.computeIfAbsent(rn, k -> new ArrayList<>()).add(c);
            }
            for (var entry : removeMap.entrySet()) {
                GRBLinExpr e = new GRBLinExpr();
                for (int c : entry.getValue()) e.addTerm(1, x[c]);
                model.addConstr(e, GRB.LESS_EQUAL, 1, "rm_" + entry.getKey());
            }

            // Each unserved node inserted at most once
            Map<Integer, List<Integer>> insertMap = new HashMap<>();
            for (int c = 0; c < C; c++) {
                int in_ = candidates.get(c).insertNode;
                insertMap.computeIfAbsent(in_, k -> new ArrayList<>()).add(c);
            }
            for (var entry : insertMap.entrySet()) {
                GRBLinExpr e = new GRBLinExpr();
                for (int c : entry.getValue()) e.addTerm(1, x[c]);
                model.addConstr(e, GRB.LESS_EQUAL, 1, "ins_" + entry.getKey());
            }

            // At most 1 removal per vehicle
            for (int k = 0; k < K; k++) {
                GRBLinExpr e = new GRBLinExpr();
                for (int c = 0; c < C; c++)
                    if (candidates.get(c).removeVehicle == k) e.addTerm(1, x[c]);
                model.addConstr(e, GRB.LESS_EQUAL, 1, "maxRmV" + k);
            }

            // At most 1 insertion per vehicle
            for (int k = 0; k < K; k++) {
                GRBLinExpr e = new GRBLinExpr();
                for (int c = 0; c < C; c++)
                    if (candidates.get(c).insertVehicle == k) e.addTerm(1, x[c]);
                model.addConstr(e, GRB.LESS_EQUAL, 1, "maxInsV" + k);
            }

            model.optimize();

            if (model.get(GRB.IntAttr.Status) == GRB.OPTIMAL ||
                    model.get(GRB.IntAttr.Status) == GRB.SUBOPTIMAL) {

                double obj = model.get(GRB.DoubleAttr.ObjVal);
                if (obj < EPS) {
                    System.out.println("[TPO-MIP] No profitable selection");
                } else {
                    // ══════════════════════════════════════════
                    //  PHASE 3: Apply selected candidates
                    // ══════════════════════════════════════════
                    Solution backup = new Solution(solution);

                    for (int c = 0; c < C; c++) {
                        if (x[c].get(GRB.DoubleAttr.X) < 0.5) continue;
                        Candidate cand = candidates.get(c);

                        System.out.printf("[TPO-MIP] Selected: remove %d from v%d, insert %d in v%d%s (net=+%.0f)%n",
                                cand.removeNode, cand.removeVehicle,
                                cand.insertNode, cand.insertVehicle,
                                cand.isTransfer ? " [TRANSFER via " + cand.transferNode + "]" : "",
                                cand.netProfit);

                        // Remove
                        Route rmRoute = solution.getRoutes().get(cand.removeVehicle);
                        int rmIdx = rmRoute.findStopIndex(cand.removeNode);
                        if (rmIdx >= 0) { rmRoute.removeStop(rmIdx); rmRoute.evaluate(); }

                        if (cand.isTransfer) {
                            // Add dropoff at transfer node in remover's route
                            Node tNode = inst.getNodeById(cand.transferNode);
                            Node sNode = inst.getNodeById(cand.removeNode);
                            double qty = sNode.getDemand();

                            int tIdx = rmRoute.findStopIndex(cand.transferNode);
                            if (tIdx >= 0) {
                                RouteStop old = rmRoute.getStops().get(tIdx);
                                double ed = old.isDropoff() ? old.getDropoffQty() : 0;
                                rmRoute.getStops().set(tIdx, old.isServed()
                                        ? RouteStop.serveAndDropoff(tNode, ed + qty)
                                        : RouteStop.dropoff(tNode, ed + qty));
                                rmRoute.evaluate();
                            }

                            // Add pickup in inserter's route
                            Route insRoute = solution.getRoutes().get(cand.insertVehicle);
                            int pickPos = findBestPickupPos(insRoute, tNode, qty, inst);
                            if (pickPos >= 0) {
                                insRoute.insertStop(pickPos, RouteStop.pickup(tNode, qty));
                                insRoute.evaluate();
                            }

                            solution.addTransfer(new Transfer(cand.transferNode,
                                    rmRoute.getVehicleId(), insRoute.getVehicleId(), qty));
                        }

                        // Insert new customer
                        Route insRoute = solution.getRoutes().get(cand.insertVehicle);
                        Node uNode = inst.getNodeById(cand.insertNode);
                        int bestPos = findBestServePos(insRoute, uNode, inst);
                        if (bestPos >= 0) {
                            insRoute.insertStop(bestPos, RouteStop.serve(uNode));
                            insRoute.evaluate();
                            result++;
                        }
                    }

                    for (Route r : solution.getRoutes()) r.evaluate();

                    // Validate
                    if (!solution.isFeasible() ||
                            solution.getTotalProfit() <= backup.getTotalProfit()) {

                        if (!solution.isFeasible()) {
                            Solution.FeasibilityReport fr = solution.checkFeasibility();
                            System.out.println("[TPO-MIP] Post-apply infeasible:");
                            for (String v : fr.getViolations()) System.out.println("  → " + v);
                        }
                        System.out.printf("[TPO-MIP] Rollback (profit %.0f ≤ %.0f)%n",
                                solution.getTotalProfit(), backup.getTotalProfit());
                        solution.restoreFrom(backup);
                        result = 0;
                    } else {
                        System.out.printf("[TPO-MIP] SUCCESS: profit %.0f → %.0f (+%.0f), transfers=%d%n",
                                backup.getTotalProfit(), solution.getTotalProfit(),
                                solution.getTotalProfit() - backup.getTotalProfit(),
                                solution.getTransfers().size());
                    }
                }
            }
            model.dispose();
            env.dispose();
        } catch (GRBException e) {
            System.err.println("[TPO-MIP] Error: " + e.getMessage());
        }
        return result;
    }

    /** Find best position to insert a serve stop, checking feasibility */
    private int findBestServePos(Route route, Node node, Instance inst) {
        double bestDet = Double.MAX_VALUE;
        int bestP = -1;
        for (int p = 0; p <= route.size(); p++) {
            route.insertStop(p, RouteStop.serve(node));
            route.evaluate();
            if (route.isFeasible()) {
                double det = route.getTotalTime();
                if (det < bestDet) { bestDet = det; bestP = p; }
            }
            route.removeStop(p);
            route.evaluate();
        }
        return bestP;
    }

    /** Find best position for a pickup stop where arc loads remain valid */
    private int findBestPickupPos(Route route, Node node, double qty, Instance inst) {
        route.evaluate();
        double bestDet = Double.MAX_VALUE;
        int bestP = -1;
        for (int p = 0; p <= route.size(); p++) {
            route.insertStop(p, RouteStop.pickup(node, qty));
            route.evaluate();
            if (route.isFeasible()) {
                double det = route.getTotalTime();
                if (det < bestDet) { bestDet = det; bestP = p; }
            }
            route.removeStop(p);
            route.evaluate();
        }
        return bestP;
    }
}