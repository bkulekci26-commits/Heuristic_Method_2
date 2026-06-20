import java.util.*;

public class TransformOperator {
    private enum MoveType { DIRECT_INSERT, SINGLE_SWAP, CROSS_TRANSFER, REVERSE_TRANSFER }

    private static class Move {
        MoveType type; double netProfit;
        Node insertNode; int insertRouteIdx; int insertPos;
        Node removeNode; int removeRouteIdx;
        int serverRouteIdx, donorRouteIdx, donorInsertPos, newCustInsertPos, pickerInsertPos;
        Node transferNode; double transferQty;
    }

    private static final long TIME_BUDGET_MS = 5000;

    public int optimize(Solution solution) {
        Solution backup = new Solution(solution);
        int totalMoves = 0;
        boolean improved = true;
        long startTime = System.currentTimeMillis();

        while (improved) {
            improved = false;
            if (System.currentTimeMillis() - startTime > TIME_BUDGET_MS) break;

            Move bestDirect = findBestDirectInsert(solution);
            Move bestSwap = findBestSingleSwap(solution);
            Move bestTransferA = findBestCrossTransfer(solution, startTime);
            Move bestTransferB = findBestReverseTransfer(solution, startTime);

            Move best = null;
            if (bestDirect != null && (best == null || bestDirect.netProfit > best.netProfit)) best = bestDirect;
            if (bestSwap != null && (best == null || bestSwap.netProfit > best.netProfit)) best = bestSwap;
            if (bestTransferA != null && (best == null || bestTransferA.netProfit > best.netProfit)) best = bestTransferA;
            if (bestTransferB != null && (best == null || bestTransferB.netProfit > best.netProfit)) best = bestTransferB;

            if (best != null && best.netProfit > 1e-6) {
                applyMove(solution, best);
                totalMoves++;
                improved = true;
            }
        }

        for (Route r : solution.getRoutes()) r.evaluate();
        if (!solution.isFeasible()) {
            solution.restoreFrom(backup);
            for (Route r : solution.getRoutes()) r.evaluate();
            return 0;
        }
        return totalMoves;
    }

    /**
     * Attempts a SINGLE best transfer move (cross or reverse) and applies it if it
     * is profit-positive and keeps the solution feasible. Lightweight alternative to
     * {@link #optimize(Solution)} (no internal improvement loop, no time budget) so it
     * can be invoked frequently inside the ALNS loop. Returns true iff a transfer was
     * applied. On infeasibility the solution is restored.
     */
    public boolean applyBestTransferMove(Solution solution) {
        Solution backup = new Solution(solution);
        long now = System.currentTimeMillis();
        Move a = findBestCrossTransfer(solution, now);
        Move b = findBestReverseTransfer(solution, now);

        Move best = null;
        if (a != null && (best == null || a.netProfit > best.netProfit)) best = a;
        if (b != null && (best == null || b.netProfit > best.netProfit)) best = b;
        if (best == null || best.netProfit <= 1e-6) return false;

        applyMove(solution, best);
        for (Route r : solution.getRoutes()) r.evaluate();
        if (!solution.isFeasible()) {
            solution.restoreFrom(backup);
            for (Route r : solution.getRoutes()) r.evaluate();
            return false;
        }
        return true;
    }

    /**
     * Unserved customers that NO single route can take by a plain direct insertion
     * (capacity or duration). These are exactly the customers a transfer can uniquely
     * help with — if a customer fits directly somewhere, a transfer is never the
     * cheaper way to serve it. Focusing the transfer search here targets the
     * "reachability" niche that split delivery cannot fill and avoids wasted work.
     */
    private List<Node> unservedNotDirectlyInsertable(Solution sol) {
        List<Node> result = new ArrayList<>();
        for (Node u : sol.getUnservedNodes()) {
            boolean insertable = false;
            for (Route r : sol.getRoutes()) {
                if (findBestFeasiblePos(new Route(r), u, sol.getInstance()) >= 0) { insertable = true; break; }
            }
            if (!insertable) result.add(u);
        }
        return result;
    }

    private Move findBestDirectInsert(Solution sol) {
        Move best = null;
        for (Node u : sol.getUnservedNodes()) {
            for (int ri = 0; ri < sol.getRoutes().size(); ri++) {
                Route route = sol.getRoutes().get(ri);
                for (int pos = 0; pos <= route.size(); pos++) {
                    if (route.canInsert(pos, RouteStop.serve(u))) {
                        if (best == null || u.getProfit() > best.netProfit) {
                            best = new Move(); best.type = MoveType.DIRECT_INSERT; best.netProfit = u.getProfit();
                            best.insertNode = u; best.insertRouteIdx = ri; best.insertPos = pos;
                        }
                    }
                }
            }
        }
        return best;
    }

    private Move findBestSingleSwap(Solution sol) {
        Move best = null;
        for (int ri = 0; ri < sol.getRoutes().size(); ri++) {
            Route route = sol.getRoutes().get(ri);
            for (int si = 0; si < route.size(); si++) {
                RouteStop stop = route.getStops().get(si);
                if (!stop.isServed()) continue;

                for (Node u : sol.getUnservedNodes()) {
                    double netProfit = u.getProfit() - stop.getNode().getProfit();
                    if (netProfit <= 0) continue;
                    Route copy = new Route(route); copy.removeStop(si);
                    int bestPos = findBestFeasiblePos(copy, u, sol.getInstance());
                    if (bestPos >= 0 && (best == null || netProfit > best.netProfit)) {
                        best = new Move(); best.type = MoveType.SINGLE_SWAP; best.netProfit = netProfit;
                        best.insertNode = u; best.removeNode = stop.getNode(); best.insertRouteIdx = ri; best.removeRouteIdx = ri; best.insertPos = bestPos;
                    }
                }
            }
        }
        return best;
    }

    private Move findBestCrossTransfer(Solution sol, long startTime) {
        Instance inst = sol.getInstance(); double W = inst.getSyncWindow();
        List<Route> routes = sol.getRoutes(); List<Node> unserved = unservedNotDirectlyInsertable(sol);
        Move best = null; unserved.sort((a, b) -> Double.compare(b.getProfit(), a.getProfit()));

        for (int i1 = 0; i1 < routes.size(); i1++) {
            if (System.currentTimeMillis() - startTime > TIME_BUDGET_MS) break;
            Route k1 = routes.get(i1); k1.evaluate();

            for (int sj = 0; sj < k1.size(); sj++) {
                RouteStop stopJ = k1.getStops().get(sj);
                if (!stopJ.isServed()) continue;

                for (int i2 = 0; i2 < routes.size(); i2++) {
                    if (i1 == i2) continue;
                    Route k2 = routes.get(i2); k2.evaluate();
                    if (k2.visitsNode(stopJ.getNode().getId())) continue;

                    int posK2 = findBestFeasiblePosForDropoff(k2, stopJ.getNode(), inst);
                    if (posK2 < 0) continue;
                    double k2SpareCap = k2.getRemainingCapacity();
                    if (k2SpareCap < 1) continue;

                    for (Node u : unserved) {
                        double needed = u.getDemand() - k1.getRemainingCapacity();
                        if (needed <= 0 || needed > k2SpareCap) continue;
                        if (best != null && u.getProfit() <= best.netProfit) continue;

                        double transferQty = needed; // BUG FIXED: Transfer EXACTLY what is needed

                        Route k2Copy = new Route(k2);
                        k2Copy.insertStop(posK2, RouteStop.dropoff(stopJ.getNode(), transferQty));
                        k2Copy.evaluate();
                        if (!k2Copy.isFeasible()) continue;

                        for (int posU = 0; posU <= k1.size(); posU++) {
                            Route k1Copy = new Route(k1);
                            k1Copy.insertStop(posU, RouteStop.serve(u));
                            int newPosJ = (posU <= sj) ? sj + 1 : sj;
                            k1Copy.getStops().set(newPosJ, new RouteStop(stopJ.getNode(), true, true, false, transferQty, 0, stopJ.getNode().getDemand()));
                            k1Copy.evaluate();

                            if (k1Copy.isFeasible() && (k2Copy.getArrivalTimeAtNode(stopJ.getNode().getId()) - k1Copy.getArrivalTimeAtNode(stopJ.getNode().getId())) <= W + 1e-6) {
                                best = new Move(); best.type = MoveType.CROSS_TRANSFER; best.netProfit = u.getProfit();
                                best.insertNode = u; best.serverRouteIdx = i1; best.donorRouteIdx = i2;
                                best.donorInsertPos = posK2; best.transferNode = stopJ.getNode();
                                best.transferQty = transferQty; best.newCustInsertPos = posU;
                            }
                        }
                    }
                }
            }
        }
        return best;
    }

    private Move findBestReverseTransfer(Solution sol, long startTime) {
        Instance inst = sol.getInstance(); double W = inst.getSyncWindow();
        List<Route> routes = sol.getRoutes(); List<Node> unserved = unservedNotDirectlyInsertable(sol);
        Move best = null; unserved.sort((a, b) -> Double.compare(b.getProfit(), a.getProfit()));

        for (int i2 = 0; i2 < routes.size(); i2++) {
            if (System.currentTimeMillis() - startTime > TIME_BUDGET_MS) break;
            Route k2 = routes.get(i2); k2.evaluate();
            double k2SpareCap = k2.getRemainingCapacity();
            if (k2SpareCap < 1) continue;

            for (int sj = 0; sj < k2.size(); sj++) {
                RouteStop stopJ = k2.getStops().get(sj);
                if (!stopJ.isServed()) continue;

                for (int i1 = 0; i1 < routes.size(); i1++) {
                    if (i1 == i2) continue;
                    Route k1 = routes.get(i1); k1.evaluate();
                    if (k1.visitsNode(stopJ.getNode().getId())) continue;

                    for (int posJ = 0; posJ <= k1.size(); posJ++) {
                        for (Node u : unserved) {
                            double needed = u.getDemand() - k1.getRemainingCapacity();
                            if (needed <= 0 || needed > k2SpareCap) continue;
                            if (best != null && u.getProfit() <= best.netProfit) continue;

                            double transferQty = needed; // BUG FIXED

                            Route k2Copy = new Route(k2);
                            k2Copy.getStops().set(sj, RouteStop.serveAndDropoff(stopJ.getNode(), transferQty));
                            k2Copy.evaluate();
                            if (!k2Copy.isFeasible()) continue;

                            for (int posU = 0; posU <= k1.size(); posU++) {
                                Route k1Copy = new Route(k1);
                                if (posU <= posJ) { k1Copy.insertStop(posU, RouteStop.serve(u)); k1Copy.insertStop(posJ + 1, RouteStop.pickup(stopJ.getNode(), transferQty)); }
                                else { k1Copy.insertStop(posJ, RouteStop.pickup(stopJ.getNode(), transferQty)); k1Copy.insertStop(posU + 1, RouteStop.serve(u)); }
                                k1Copy.evaluate();

                                if (k1Copy.isFeasible() && (k2Copy.getArrivalTimeAtNode(stopJ.getNode().getId()) - k1Copy.getArrivalTimeAtNode(stopJ.getNode().getId())) <= W + 1e-6) {
                                    best = new Move(); best.type = MoveType.REVERSE_TRANSFER; best.netProfit = u.getProfit();
                                    best.insertNode = u; best.serverRouteIdx = i2; best.donorRouteIdx = i1;
                                    best.transferNode = stopJ.getNode(); best.transferQty = transferQty;
                                    best.newCustInsertPos = posU; best.pickerInsertPos = posJ;
                                }
                            }
                        }
                    }
                }
            }
        }
        return best;
    }

    private void applyMove(Solution sol, Move move) {
        List<Route> routes = sol.getRoutes();
        if (move.type == MoveType.DIRECT_INSERT) {
            routes.get(move.insertRouteIdx).insertStop(move.insertPos, RouteStop.serve(move.insertNode));
            routes.get(move.insertRouteIdx).evaluate();
        } else if (move.type == MoveType.SINGLE_SWAP) {
            routes.get(move.insertRouteIdx).removeStop(routes.get(move.insertRouteIdx).findStopIndex(move.removeNode.getId()));
            routes.get(move.insertRouteIdx).insertStop(move.insertPos, RouteStop.serve(move.insertNode));
            routes.get(move.insertRouteIdx).evaluate();
        } else if (move.type == MoveType.CROSS_TRANSFER) {
            Route k1 = routes.get(move.serverRouteIdx); Route k2 = routes.get(move.donorRouteIdx);
            k1.insertStop(move.newCustInsertPos, RouteStop.serve(move.insertNode));
            k1.getStops().set(k1.findStopIndex(move.transferNode.getId()), new RouteStop(move.transferNode, true, true, false, move.transferQty, 0, move.transferNode.getDemand()));
            k1.evaluate();
            k2.insertStop(move.donorInsertPos, RouteStop.dropoff(move.transferNode, move.transferQty));
            k2.evaluate();
            sol.addTransfer(new Transfer(move.transferNode.getId(), k2.getVehicleId(), k1.getVehicleId(), move.transferQty));
        } else if (move.type == MoveType.REVERSE_TRANSFER) {
            Route k2 = routes.get(move.serverRouteIdx); Route k1 = routes.get(move.donorRouteIdx);
            k2.getStops().set(k2.findStopIndex(move.transferNode.getId()), RouteStop.serveAndDropoff(move.transferNode, move.transferQty));
            k2.evaluate();
            if (move.newCustInsertPos <= move.pickerInsertPos) { k1.insertStop(move.newCustInsertPos, RouteStop.serve(move.insertNode)); k1.insertStop(move.pickerInsertPos + 1, RouteStop.pickup(move.transferNode, move.transferQty)); }
            else { k1.insertStop(move.pickerInsertPos, RouteStop.pickup(move.transferNode, move.transferQty)); k1.insertStop(move.newCustInsertPos + 1, RouteStop.serve(move.insertNode)); }
            k1.evaluate();
            sol.addTransfer(new Transfer(move.transferNode.getId(), k2.getVehicleId(), k1.getVehicleId(), move.transferQty));
        }
    }

    private int findBestFeasiblePos(Route route, Node node, Instance inst) {
        double bestTime = Double.MAX_VALUE; int bestPos = -1;
        for (int p = 0; p <= route.size(); p++) {
            route.insertStop(p, RouteStop.serve(node)); route.evaluate();
            if (route.isFeasible() && route.getTotalTime() < bestTime) { bestTime = route.getTotalTime(); bestPos = p; }
            route.removeStop(p); route.evaluate();
        }
        return bestPos;
    }

    private int findBestFeasiblePosForDropoff(Route route, Node node, Instance inst) {
        double bestTime = Double.MAX_VALUE; int bestPos = -1;
        for (int p = 0; p <= route.size(); p++) {
            Node prev = (p == 0) ? inst.getDepot() : route.getStops().get(p - 1).getNode();
            Node next = (p == route.getStops().size()) ? inst.getDepot() : route.getStops().get(p).getNode();
            double newTime = route.getTotalTime() + inst.getDistance(prev, node) + inst.getDistance(node, next) - inst.getDistance(prev, next);
            if (newTime <= inst.getMaxRouteDuration() + 1e-6 && newTime < bestTime) { bestTime = newTime; bestPos = p; }
        }
        return bestPos;
    }
}