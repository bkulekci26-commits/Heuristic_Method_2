import com.gurobi.gurobi.*;
import java.util.*;

/**
 * Transfer Post-Optimization MIP v6 for CTOP-T-Sync.
 *
 * Design: MIP PLANS globally, Routes VALIDATE locally.
 *
 * The MIP maximizes net profit by deciding:
 *   - Which served nodes to remove (frees capacity)
 *   - Which unserved nodes to insert (gains profit)
 *   - How to share demand across vehicles (transfers)
 *
 * KEY: No time constraint in MIP. Time feasibility is checked
 * by inserting into actual Route copies post-solve.
 * This eliminates the additive time approximation that broke v1-v5.
 */
public class TransferMIP {

    private static final double EPS = 1e-4;

    public int optimize(Solution solution) {
        Instance inst = solution.getInstance();
        List<Route> routes = solution.getRoutes();
        for (Route r : routes) r.evaluate();

        int K = routes.size();
        double Q = inst.getMaxCapacity();
        double W = inst.getSyncWindow();

        // ── Served nodes ──
        List<int[]> served = new ArrayList<>();
        List<Double> servedProfit = new ArrayList<>();
        List<Double> servedDemand = new ArrayList<>();
        for (int k = 0; k < K; k++) {
            Route route = routes.get(k);
            for (int s = 0; s < route.size(); s++) {
                RouteStop stop = route.getStops().get(s);
                if (stop.isServed()) {
                    served.add(new int[]{k, s, stop.getNode().getId()});
                    servedProfit.add(stop.getNode().getProfit());
                    servedDemand.add(stop.getNode().getDemand());
                }
            }
        }
        int S = served.size();

        // ── Unserved nodes ──
        List<Node> unserved = solution.getUnservedNodes();
        int U = unserved.size();
        if (S == 0 || U == 0) return 0;

        System.out.printf("[TPO-MIP] K=%d S=%d U=%d%n", K, S, U);
        for (int k = 0; k < K; k++)
            System.out.printf("[TPO-MIP]   v%d: load=%.0f/%.0f time=%.1f/%.1f%n",
                    k, routes.get(k).getInitialLoad(), Q,
                    routes.get(k).getTotalTime(), inst.getMaxRouteDuration());

        int result = 0;
        try {
            GRBEnv env = new GRBEnv(true);
            env.set(GRB.IntParam.LogToConsole, 0);
            env.set(GRB.DoubleParam.TimeLimit, 30.0);
            env.start();
            GRBModel model = new GRBModel(env);
            model.set(GRB.IntAttr.ModelSense, GRB.MAXIMIZE);

            // ═══ VARIABLES ═══
            GRBVar[] rm = new GRBVar[S];
            for (int si = 0; si < S; si++)
                rm[si] = model.addVar(0, 1, 0, GRB.BINARY, "rm" + si);

            GRBVar[][] ins = new GRBVar[U][K];
            for (int ui = 0; ui < U; ui++)
                for (int k = 0; k < K; k++)
                    ins[ui][k] = model.addVar(0, 1, 0, GRB.BINARY, "ins" + ui + "_" + k);

            GRBVar[][] share = new GRBVar[S][K];
            for (int si = 0; si < S; si++)
                for (int k = 0; k < K; k++)
                    share[si][k] = model.addVar(0, servedDemand.get(si), 0, GRB.CONTINUOUS, "sh" + si + "_" + k);

            // ═══ OBJECTIVE: max net profit ═══
            GRBLinExpr obj = new GRBLinExpr();
            for (int ui = 0; ui < U; ui++)
                for (int k = 0; k < K; k++)
                    obj.addTerm(unserved.get(ui).getProfit(), ins[ui][k]);
            for (int si = 0; si < S; si++)
                obj.addTerm(-servedProfit.get(si), rm[si]);
            model.setObjective(obj, GRB.MAXIMIZE);

            // ═══ CONSTRAINTS ═══

            // (C1) Each unserved node inserted at most once
            for (int ui = 0; ui < U; ui++) {
                GRBLinExpr e = new GRBLinExpr();
                for (int k = 0; k < K; k++) e.addTerm(1, ins[ui][k]);
                model.addConstr(e, GRB.LESS_EQUAL, 1, "insOnce" + ui);
            }

            // (C2) Demand sharing: Σ_k share[s][k] = demand(s) × (1 - rm[s])
            for (int si = 0; si < S; si++) {
                GRBLinExpr lhs = new GRBLinExpr();
                for (int k = 0; k < K; k++) lhs.addTerm(1, share[si][k]);
                GRBLinExpr rhs = new GRBLinExpr();
                rhs.addConstant(servedDemand.get(si));
                rhs.addTerm(-servedDemand.get(si), rm[si]);
                model.addConstr(lhs, GRB.EQUAL, rhs, "dem" + si);
            }

            // (C3) Capacity: Σ share[s][k] + Σ demand(u)·ins[u][k] ≤ Q
            for (int k = 0; k < K; k++) {
                GRBLinExpr e = new GRBLinExpr();
                for (int si = 0; si < S; si++) e.addTerm(1, share[si][k]);
                for (int ui = 0; ui < U; ui++)
                    e.addTerm(unserved.get(ui).getDemand(), ins[ui][k]);
                model.addConstr(e, GRB.LESS_EQUAL, Q, "cap" + k);
            }

            // (C4) Net profit ≥ 1
            model.addConstr(obj, GRB.GREATER_EQUAL, 1, "netPos");

            // (C5) Limit removals per vehicle
            for (int k = 0; k < K; k++) {
                GRBLinExpr e = new GRBLinExpr();
                for (int si = 0; si < S; si++)
                    if (served.get(si)[0] == k) e.addTerm(1, rm[si]);
                model.addConstr(e, GRB.LESS_EQUAL, 2, "maxRm" + k);
            }

            // (C6) Limit insertions per vehicle
            for (int k = 0; k < K; k++) {
                GRBLinExpr e = new GRBLinExpr();
                for (int ui = 0; ui < U; ui++) e.addTerm(1, ins[ui][k]);
                model.addConstr(e, GRB.LESS_EQUAL, 2, "maxIns" + k);
            }

            // NO time constraint — validated post-hoc
            // NO sync constraint — validated post-hoc

            // ═══ SOLVE ═══
            model.optimize();

            int status = model.get(GRB.IntAttr.Status);
            if (status == GRB.OPTIMAL || status == GRB.SUBOPTIMAL) {
                double objVal = model.get(GRB.DoubleAttr.ObjVal);
                System.out.printf("[TPO-MIP] MIP objective: +%.0f%n", objVal);

                // Extract solution
                List<int[]> removals = new ArrayList<>();
                for (int si = 0; si < S; si++)
                    if (rm[si].get(GRB.DoubleAttr.X) > 0.5) {
                        removals.add(new int[]{served.get(si)[0], served.get(si)[2]});
                        System.out.printf("[TPO-MIP] Plan: remove node %d (p=%.0f) from v%d%n",
                                served.get(si)[2], servedProfit.get(si), served.get(si)[0]);
                    }

                List<int[]> insertions = new ArrayList<>();
                for (int ui = 0; ui < U; ui++)
                    for (int k = 0; k < K; k++)
                        if (ins[ui][k].get(GRB.DoubleAttr.X) > 0.5) {
                            insertions.add(new int[]{ui, k});
                            System.out.printf("[TPO-MIP] Plan: insert node %d (p=%.0f) in v%d%n",
                                    unserved.get(ui).getId(), unserved.get(ui).getProfit(), k);
                        }

                List<double[]> shareOps = new ArrayList<>();
                for (int si = 0; si < S; si++) {
                    if (rm[si].get(GRB.DoubleAttr.X) > 0.5) continue;
                    int origK = served.get(si)[0];
                    for (int k = 0; k < K; k++) {
                        if (k == origK) continue;
                        double val = share[si][k].get(GRB.DoubleAttr.X);
                        if (val > EPS) {
                            shareOps.add(new double[]{si, origK, k, val});
                            System.out.printf("[TPO-MIP] Plan: share node %d → v%d delivers %.0f%n",
                                    served.get(si)[2], k, val);
                        }
                    }
                }

                // ═══ POST-MIP: Apply on actual routes ═══
                result = applyMipSolution(solution, inst, routes, served, unserved,
                        removals, insertions, shareOps, W);

            } else if (status == GRB.INFEASIBLE) {
                System.out.println("[TPO-MIP] Infeasible");
            }

            model.dispose();
            env.dispose();
        } catch (GRBException e) {
            System.err.println("[TPO-MIP] Error: " + e.getMessage());
        }
        return result;
    }

    /**
     * Apply MIP solution on actual routes with step-by-step validation.
     */
    private int applyMipSolution(Solution solution, Instance inst, List<Route> routes,
                                 List<int[]> served, List<Node> unserved,
                                 List<int[]> removals, List<int[]> insertions,
                                 List<double[]> shareOps, double W) {
        Solution backup = new Solution(solution);
        int transferCount = 0;
        int insertCount = 0;

        // Step 1: Removals (always safe)
        for (int[] r : removals) {
            Route route = routes.get(r[0]);
            int idx = route.findStopIndex(r[1]);
            if (idx >= 0) { route.removeStop(idx); route.evaluate(); }
        }

        // Step 2: Demand sharing (creates transfers)
        for (double[] op : shareOps) {
            int si = (int) op[0];
            int origK = (int) op[1];
            int otherK = (int) op[2];
            double qty = op[3];
            int nid = served.get(si)[2];
            Node tn = inst.getNodeById(nid);

            Route other = routes.get(otherK);
            int oi = other.findStopIndex(nid);
            if (oi >= 0) {
                RouteStop old = other.getStops().get(oi);
                double ed = old.isDropoff() ? old.getDropoffQty() : 0;
                other.getStops().set(oi, old.isServed()
                        ? RouteStop.serveAndDropoff(tn, ed + qty) : RouteStop.dropoff(tn, ed + qty));
            } else {
                int pos = findBestPos(other, tn, inst, true, qty);
                if (pos < 0) { System.out.printf("[TPO-MIP] SKIP share at node %d (no pos)%n", nid); continue; }
                other.insertStop(pos, RouteStop.dropoff(tn, qty));
            }
            other.evaluate();

            Route orig = routes.get(origK);
            int oi2 = orig.findStopIndex(nid);
            if (oi2 >= 0) {
                RouteStop old = orig.getStops().get(oi2);
                double ep = old.isPickup() ? old.getPickupQty() : 0;
                orig.getStops().set(oi2, old.isServed()
                        ? RouteStop.serveAndPickup(tn, ep + qty) : RouteStop.pickup(tn, ep + qty));
                orig.evaluate();
            }

            // Early sync check: dropper(other) arrival - picker(orig) arrival ≤ W
            // Dropper arrives first → no problem (load waits at node)
            // Picker arrives first → picker must wait ≤ W for dropper
            double dropperArr = other.getArrivalTimeAtNode(nid);
            double pickerArr = orig.getArrivalTimeAtNode(nid);
            double syncGap = dropperArr - pickerArr;
            if (syncGap > W + 1e-6) {
                System.out.printf("[TPO-MIP] SKIP share at node %d: sync gap=%.1f > W=%.1f (dropper=%.1f, picker=%.1f)%n",
                        nid, syncGap, W, dropperArr, pickerArr);
                // Revert: would be complex, let final validation handle it
                // But still register — final validation will rollback if needed
            }

            solution.addTransfer(new Transfer(nid, other.getVehicleId(), orig.getVehicleId(), qty));
            transferCount++;
        }

        // Step 3: Insertions one by one (sorted by profit desc)
        insertions.sort((a, b) -> Double.compare(
                unserved.get(b[0]).getProfit(), unserved.get(a[0]).getProfit()));

        for (int[] insOp : insertions) {
            Node u = unserved.get(insOp[0]);
            Route route = routes.get(insOp[1]);
            int pos = findBestPos(route, u, inst, false, 0);
            if (pos >= 0) {
                route.insertStop(pos, RouteStop.serve(u));
                route.evaluate();
                if (route.isFeasible()) {
                    insertCount++;
                    System.out.printf("[TPO-MIP] Inserted node %d (p=%.0f) in v%d%n",
                            u.getId(), u.getProfit(), insOp[1]);
                } else {
                    route.removeStop(route.findStopIndex(u.getId()));
                    route.evaluate();
                    System.out.printf("[TPO-MIP] SKIP node %d (time infeasible in v%d)%n", u.getId(), insOp[1]);
                }
            } else {
                System.out.printf("[TPO-MIP] SKIP node %d (no position in v%d)%n", u.getId(), insOp[1]);
            }
        }

        // Step 4: Validate
        for (Route r : routes) r.evaluate();
        if (!solution.isFeasible() || solution.getTotalProfit() <= backup.getTotalProfit()) {
            if (!solution.isFeasible()) {
                Solution.FeasibilityReport fr = solution.checkFeasibility();
                System.out.println("[TPO-MIP] Post-apply infeasible:");
                for (String v : fr.getViolations()) System.out.println("  → " + v);
            }
            System.out.printf("[TPO-MIP] Rollback (%.0f ≤ %.0f)%n",
                    solution.getTotalProfit(), backup.getTotalProfit());
            solution.restoreFrom(backup);
            for (Route r : solution.getRoutes()) r.evaluate();
            return 0;
        }

        System.out.printf("[TPO-MIP] SUCCESS: %.0f → %.0f (+%.0f), %d inserts, %d transfers%n",
                backup.getTotalProfit(), solution.getTotalProfit(),
                solution.getTotalProfit() - backup.getTotalProfit(),
                insertCount, transferCount);
        return insertCount + transferCount;
    }

    private int findBestPos(Route route, Node node, Instance inst, boolean isDropoff, double qty) {
        double bestTime = Double.MAX_VALUE;
        int bestP = -1;
        for (int p = 0; p <= route.size(); p++) {
            RouteStop s = isDropoff ? RouteStop.dropoff(node, qty) : RouteStop.serve(node);
            route.insertStop(p, s);
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
}