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

    public int optimize(Solution solution) {
        Solution backup = new Solution(solution);
        int totalMoves = 0;
        boolean improved = true;

        while (improved) {
            improved = false;

            Move bestDirect = findBestDirectInsert(solution);
            Move bestSwap = findBestSingleSwap(solution);
            Move bestTransferA = findBestCrossTransfer(solution);
            Move bestTransferB = findBestReverseTransfer(solution);

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

    private List<Node> getTopUnserved(Solution sol, int limit) {
        List<Node> unserved = sol.getUnservedNodes();
        unserved.sort((a, b) -> Double.compare(b.getProfit(), a.getProfit()));
        if (unserved.size() > limit) return unserved.subList(0, limit);
        return unserved;
    }

    private Move findBestDirectInsert(Solution sol) {
        Move best = null;
        for (Node u : getTopUnserved(sol, 15)) {
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

                for (Node u : getTopUnserved(sol, 15)) {
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

    private Move findBestCrossTransfer(Solution sol) {
        Instance inst = sol.getInstance();
        List<Route> routes = sol.getRoutes();
        List<Node> unserved = getTopUnserved(sol, 15);
        Move best = null;

        for (int i1 = 0; i1 < routes.size(); i1++) {
            Route k1 = routes.get(i1); k1.evaluate();

            for (int sj = 0; sj < k1.size(); sj++) {
                RouteStop stopJ = k1.getStops().get(sj);
                if (!stopJ.isServed()) continue;
                // FIX: Constraint 9 (Cannot be a dropoff if we want to make it a pickup)
                if (stopJ.isDropoff()) continue;

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

                        double transferQty = needed;

                        Route k2Copy = new Route(k2);
                        RouteStop dropoffStop = new RouteStop(stopJ.getNode(), false, false, true, 0, transferQty, 0);
                        k2Copy.insertStop(posK2, dropoffStop);
                        k2Copy.evaluate();
                        if (!k2Copy.isFeasible()) continue;

                        for (int posU = 0; posU <= k1.size(); posU++) {
                            Route k1Copy = new Route(k1);
                            k1Copy.insertStop(posU, RouteStop.serve(u));
                            int newPosJ = (posU <= sj) ? sj + 1 : sj;

                            RouteStop pickerStop = new RouteStop(
                                    stopJ.getNode(), stopJ.isServed(), true, stopJ.isDropoff(),
                                    stopJ.getPickupQty() + transferQty, stopJ.getDropoffQty(), stopJ.getDeliveryQty()
                            );
                            k1Copy.getStops().set(newPosJ, pickerStop);
                            k1Copy.evaluate();

                            double giverTime = k2Copy.getArrivalTimeAtNode(stopJ.getNode().getId());
                            double receiverRawTime = k1Copy.getArrivalTimeAtNode(stopJ.getNode().getId());
                            double requiredWaitTime = Math.max(0.0, giverTime - receiverRawTime);
                            pickerStop.setWaitingTime(requiredWaitTime);

                            k1Copy.evaluate();

                            if (k1Copy.isFeasible()) {
                                double score = u.getProfit() * 1.5;
                                if (best == null || score > best.netProfit) {
                                    best = new Move(); best.type = MoveType.CROSS_TRANSFER; best.netProfit = score;
                                    best.insertNode = u; best.serverRouteIdx = i1; best.donorRouteIdx = i2;
                                    best.donorInsertPos = posK2; best.transferNode = stopJ.getNode();
                                    best.transferQty = transferQty; best.newCustInsertPos = posU;
                                }
                            }
                        }
                    }
                }
            }
        }
        return best;
    }

    private Move findBestReverseTransfer(Solution sol) {
        Instance inst = sol.getInstance();
        List<Route> routes = sol.getRoutes();
        List<Node> unserved = getTopUnserved(sol, 15);
        Move best = null;

        for (int i2 = 0; i2 < routes.size(); i2++) {
            Route k2 = routes.get(i2); k2.evaluate();
            double k2SpareCap = k2.getRemainingCapacity();
            if (k2SpareCap < 1) continue;

            for (int sj = 0; sj < k2.size(); sj++) {
                RouteStop stopJ = k2.getStops().get(sj);
                if (!stopJ.isServed()) continue;
                // FIX: Constraint 9 (Cannot be a pickup if we want to make it a dropoff)
                if (stopJ.isPickup()) continue;

                for (int i1 = 0; i1 < routes.size(); i1++) {
                    if (i1 == i2) continue;
                    Route k1 = routes.get(i1); k1.evaluate();
                    if (k1.visitsNode(stopJ.getNode().getId())) continue;

                    for (int posJ = 0; posJ <= k1.size(); posJ++) {
                        for (Node u : unserved) {
                            double needed = u.getDemand() - k1.getRemainingCapacity();
                            if (needed <= 0 || needed > k2SpareCap) continue;

                            double transferQty = needed;

                            Route k2Copy = new Route(k2);
                            RouteStop dropoffStop = new RouteStop(
                                    stopJ.getNode(), stopJ.isServed(), stopJ.isPickup(), true,
                                    stopJ.getPickupQty(), stopJ.getDropoffQty() + transferQty, stopJ.getDeliveryQty()
                            );
                            k2Copy.getStops().set(sj, dropoffStop);
                            k2Copy.evaluate();
                            if (!k2Copy.isFeasible()) continue;

                            for (int posU = 0; posU <= k1.size(); posU++) {
                                Route k1Copy = new Route(k1);
                                RouteStop pickerStop = RouteStop.pickup(stopJ.getNode(), transferQty);

                                if (posU <= posJ) {
                                    k1Copy.insertStop(posU, RouteStop.serve(u));
                                    k1Copy.insertStop(posJ + 1, pickerStop);
                                } else {
                                    k1Copy.insertStop(posJ, pickerStop);
                                    k1Copy.insertStop(posU + 1, RouteStop.serve(u));
                                }
                                k1Copy.evaluate();

                                double giverTime = k2Copy.getArrivalTimeAtNode(stopJ.getNode().getId());
                                double receiverRawTime = k1Copy.getArrivalTimeAtNode(stopJ.getNode().getId());
                                double requiredWaitTime = Math.max(0.0, giverTime - receiverRawTime);
                                pickerStop.setWaitingTime(requiredWaitTime);

                                k1Copy.evaluate();

                                if (k1Copy.isFeasible()) {
                                    double score = u.getProfit() * 1.5;
                                    if (best == null || score > best.netProfit) {
                                        best = new Move(); best.type = MoveType.REVERSE_TRANSFER; best.netProfit = score;
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
            RouteStop stopJ = k1.getStops().get(k1.findStopIndex(move.transferNode.getId()));
            k1.getStops().set(k1.findStopIndex(move.transferNode.getId()), new RouteStop(move.transferNode, stopJ.isServed(), true, stopJ.isDropoff(), stopJ.getPickupQty() + move.transferQty, stopJ.getDropoffQty(), stopJ.getDeliveryQty()));
            k1.evaluate();
            k2.insertStop(move.donorInsertPos, RouteStop.dropoff(move.transferNode, move.transferQty));
            k2.evaluate();
            sol.addTransfer(new Transfer(move.transferNode.getId(), k2.getVehicleId(), k1.getVehicleId(), move.transferQty));
        } else if (move.type == MoveType.REVERSE_TRANSFER) {
            Route k2 = routes.get(move.serverRouteIdx); Route k1 = routes.get(move.donorRouteIdx);
            RouteStop stopJ = k2.getStops().get(k2.findStopIndex(move.transferNode.getId()));
            k2.getStops().set(k2.findStopIndex(move.transferNode.getId()), new RouteStop(move.transferNode, stopJ.isServed(), stopJ.isPickup(), true, stopJ.getPickupQty(), stopJ.getDropoffQty() + move.transferQty, stopJ.getDeliveryQty()));
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