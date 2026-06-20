import java.util.*;

public class LocalSearch {

    private int totalImprovements;
    public LocalSearch() { this.totalImprovements = 0; }

    public int improve(Solution solution) {
        totalImprovements = 0;
        boolean improved = true;
        while (improved) {
            improved = false;
            if (apply2Opt(solution)) { improved = true; continue; }
            if (applyRelocate(solution)) { improved = true; continue; }
            if (applySwap(solution)) { improved = true; continue; }
            if (applyReplace(solution)) { improved = true; continue; }
        }
        return totalImprovements;
    }

    private boolean apply2Opt(Solution solution) {
        for (Route route : solution.getRoutes()) {
            List<RouteStop> stops = route.getStops();
            int n = stops.size();
            if (n < 2) continue;
            double bestDist = route.getTotalDistance();
            for (int i = 0; i < n - 1; i++) {
                for (int j = i + 1; j < n; j++) {
                    reverseSegment(stops, i, j);
                    route.evaluate();
                    if (route.isFeasible() && route.getTotalDistance() < bestDist - 1e-6) {
                        bestDist = route.getTotalDistance();
                        totalImprovements++; return true;
                    } else {
                        reverseSegment(stops, i, j);
                        route.evaluate();
                    }
                }
            }
        }
        return false;
    }

    private void reverseSegment(List<RouteStop> stops, int i, int j) {
        while (i < j) { RouteStop tmp = stops.get(i); stops.set(i, stops.get(j)); stops.set(j, tmp); i++; j--; }
    }

    private boolean applyRelocate(Solution solution) {
        double currentObj = computeObjective(solution);
        List<Route> routes = solution.getRoutes();
        double bestImprovement = 0;
        int bestSourceRoute = -1, bestSourcePos = -1, bestTargetRoute = -1, bestTargetPos = -1;

        for (int r1 = 0; r1 < routes.size(); r1++) {
            Route source = routes.get(r1);
            if (source.isEmpty()) continue;
            for (int sPos = 0; sPos < source.size(); sPos++) {
                RouteStop stop = source.getStops().get(sPos);
                if (!stop.isServed()) continue;

                source.removeStop(sPos); source.evaluate();
                if (!source.isFeasible()) { source.insertStop(sPos, stop); source.evaluate(); continue; }

                for (int r2 = 0; r2 < routes.size(); r2++) {
                    if (r1 == r2) continue;
                    Route target = routes.get(r2);
                    for (int tPos = 0; tPos <= target.size(); tPos++) {
                        target.insertStop(tPos, stop); target.evaluate();
                        if (target.isFeasible()) {
                            double improvement = computeObjective(solution) - currentObj;
                            if (improvement > bestImprovement + 1e-6) {
                                bestImprovement = improvement; bestSourceRoute = r1; bestSourcePos = sPos; bestTargetRoute = r2; bestTargetPos = tPos;
                            }
                        }
                        target.removeStop(tPos); target.evaluate();
                    }
                }
                source.insertStop(sPos, stop); source.evaluate();
            }
        }
        if (bestImprovement > 1e-6) {
            RouteStop moved = routes.get(bestSourceRoute).removeStop(bestSourcePos);
            routes.get(bestSourceRoute).evaluate();
            routes.get(bestTargetRoute).insertStop(bestTargetPos, moved);
            routes.get(bestTargetRoute).evaluate();
            totalImprovements++; return true;
        }
        return false;
    }

    private boolean applySwap(Solution solution) {
        double currentObj = computeObjective(solution);
        List<Route> routes = solution.getRoutes();
        double bestImprovement = 0;
        int bestR1 = -1, bestPos1 = -1, bestR2 = -1, bestPos2 = -1;

        for (int r1 = 0; r1 < routes.size(); r1++) {
            for (int r2 = r1 + 1; r2 < routes.size(); r2++) {
                Route route1 = routes.get(r1); Route route2 = routes.get(r2);
                for (int p1 = 0; p1 < route1.size(); p1++) {
                    for (int p2 = 0; p2 < route2.size(); p2++) {
                        RouteStop stop1 = route1.getStops().get(p1); RouteStop stop2 = route2.getStops().get(p2);
                        if (!stop1.isServed() || !stop2.isServed()) continue;

                        route1.getStops().set(p1, stop2); route2.getStops().set(p2, stop1);
                        route1.evaluate(); route2.evaluate();

                        if (route1.isFeasible() && route2.isFeasible()) {
                            double improvement = computeObjective(solution) - currentObj;
                            if (improvement > bestImprovement + 1e-6) {
                                bestImprovement = improvement; bestR1 = r1; bestPos1 = p1; bestR2 = r2; bestPos2 = p2;
                            }
                        }
                        route1.getStops().set(p1, stop1); route2.getStops().set(p2, stop2);
                        route1.evaluate(); route2.evaluate();
                    }
                }
            }
        }
        if (bestImprovement > 1e-6) {
            Route route1 = routes.get(bestR1); Route route2 = routes.get(bestR2);
            RouteStop stop1 = route1.getStops().get(bestPos1); RouteStop stop2 = route2.getStops().get(bestPos2);
            route1.getStops().set(bestPos1, stop2); route2.getStops().set(bestPos2, stop1);
            route1.evaluate(); route2.evaluate();
            totalImprovements++; return true;
        }
        return false;
    }

    private boolean applyReplace(Solution solution) {
        double currentObj = computeObjective(solution);
        List<Route> routes = solution.getRoutes(); List<Node> unserved = solution.getUnservedNodes();
        if (unserved.isEmpty()) return false;
        double bestImprovement = 0; int bestRouteIdx = -1, bestRemovePos = -1, bestInsertRoute = -1, bestInsertPos = -1; Node bestUnserved = null;

        for (int ri = 0; ri < routes.size(); ri++) {
            Route route = routes.get(ri);
            for (int sPos = 0; sPos < route.size(); sPos++) {
                RouteStop served = route.getStops().get(sPos);
                // SAFEGUARD: Do not destroy split deliveries during simple replacement
                if (!served.isServed() || served.isSplitDelivery()) continue;

                route.removeStop(sPos); route.evaluate();

                for (Node uNode : unserved) {
                    for (int r2 = 0; r2 < routes.size(); r2++) {
                        Route target = routes.get(r2);
                        for (int iPos = 0; iPos <= target.size(); iPos++) {
                            target.insertStop(iPos, RouteStop.serve(uNode)); target.evaluate();
                            if (target.isFeasible()) {
                                double improvement = computeObjective(solution) - currentObj;
                                if (improvement > bestImprovement + 1e-6) {
                                    bestImprovement = improvement; bestRouteIdx = ri; bestRemovePos = sPos; bestInsertRoute = r2; bestInsertPos = iPos; bestUnserved = uNode;
                                }
                            }
                            target.removeStop(iPos); target.evaluate();
                        }
                    }
                }
                route.insertStop(sPos, served); route.evaluate();
            }
        }
        if (bestImprovement > 1e-6 && bestUnserved != null) {
            Route removeRoute = routes.get(bestRouteIdx); Route insertRoute = routes.get(bestInsertRoute);
            removeRoute.removeStop(bestRemovePos); removeRoute.evaluate();
            int adjustedPos = bestInsertPos;
            if (bestInsertRoute == bestRouteIdx && bestInsertPos > bestRemovePos) adjustedPos--;
            insertRoute.insertStop(adjustedPos, RouteStop.serve(bestUnserved)); insertRoute.evaluate();
            totalImprovements++; return true;
        }
        return false;
    }

    // ══════════════════════════════════════════════════════════
    // SPLIT-AWARE POST INSERTION (Crucial for SD-CTOP)
    // ══════════════════════════════════════════════════════════

    private static class InsertionPlan {
        List<Route> routes = new ArrayList<>(); List<Integer> positions = new ArrayList<>(); List<Double> quantities = new ArrayList<>(); double score;
        InsertionPlan(Route r, int p, double q, double s) { routes.add(r); positions.add(p); quantities.add(q); score = s; }
        InsertionPlan(List<Route> rs, List<Integer> ps, List<Double> qs, double s) { routes.addAll(rs); positions.addAll(ps); quantities.addAll(qs); score = s; }
    }

    public int postInsert(Solution solution) {
        int inserted = 0; boolean improved = true;
        while (improved) {
            improved = false;
            Map<Integer, Double> totalDelivery = new HashMap<>();
            for (Route r : solution.getRoutes()) {
                for (RouteStop s : r.getStops()) { if (s.isServed()) totalDelivery.merge(s.getNode().getId(), s.getDeliveryQty(), Double::sum); }
            }
            List<Node> candidates = new ArrayList<>();
            for (Node n : solution.getInstance().getNodes()) {
                if (!n.isDepot() && totalDelivery.getOrDefault(n.getId(), 0.0) < n.getDemand() - 1e-6) candidates.add(n);
            }
            if (candidates.isEmpty()) break;

            double bestScore = Double.NEGATIVE_INFINITY; InsertionPlan bestPlan = null; Node bestNode = null;

            for (Node cand : candidates) {
                double missingDemand = cand.getDemand() - totalDelivery.getOrDefault(cand.getId(), 0.0);
                InsertionPlan singlePlan = null;
                for (Route route : solution.getRoutes()) {
                    if (route.getRemainingCapacity() < missingDemand) continue;
                    for (int pos = 0; pos <= route.size(); pos++) {
                        if (route.canInsert(pos, RouteStop.servePartial(cand, missingDemand))) {
                            double cost = computeInsertionCost(route, pos, cand);
                            double score = cand.getProfit() / Math.max(cost, 1e-6);
                            if (singlePlan == null || score > singlePlan.score) singlePlan = new InsertionPlan(route, pos, missingDemand, score);
                        }
                    }
                }

                InsertionPlan splitPlan = null;
                if (singlePlan == null) {
                    List<Route> sRoutes = new ArrayList<>(); List<Integer> sPos = new ArrayList<>(); List<Double> sQty = new ArrayList<>();
                    double accum = 0, aggScore = 0;
                    for (Route route : solution.getRoutes()) {
                        if (accum >= missingDemand - 1e-6) break;
                        double rem = route.getRemainingCapacity(); if (rem < 1) continue;
                        double qty = Math.min(rem, missingDemand - accum);
                        int bPos = -1; double bScore = -1;
                        for (int p = 0; p <= route.size(); p++) {
                            if (route.canInsert(p, RouteStop.servePartial(cand, qty))) {
                                double sc = cand.getProfit() / Math.max(computeInsertionCost(route, p, cand), 1e-6);
                                if (sc > bScore) { bScore = sc; bPos = p; }
                            }
                        }
                        if (bPos != -1) { sRoutes.add(route); sPos.add(bPos); sQty.add(qty); accum += qty; aggScore += bScore; }
                    }
                    if (Math.abs(accum - missingDemand) < 1e-6 && sRoutes.size() > 1) splitPlan = new InsertionPlan(sRoutes, sPos, sQty, aggScore / sRoutes.size());
                }

                InsertionPlan candPlan = singlePlan != null ? singlePlan : splitPlan;
                if (candPlan != null && candPlan.score > bestScore) { bestScore = candPlan.score; bestPlan = candPlan; bestNode = cand; }
            }

            if (bestPlan != null) {
                for (int i = 0; i < bestPlan.routes.size(); i++) {
                    Route r = bestPlan.routes.get(i);
                    r.insertStop(bestPlan.positions.get(i), RouteStop.servePartial(bestNode, bestPlan.quantities.get(i)));
                    r.evaluate();
                }
                inserted++; improved = true;
            }
        }
        return inserted;
    }

    private double computeObjective(Solution solution) { double p = 0; for (Route r : solution.getRoutes()) p += r.getTotalProfit(); return p; }

    private double computeInsertionCost(Route route, int position, Node node) {
        Instance inst = route.getInstance(); Node depot = inst.getDepot(); List<RouteStop> stops = route.getStops();
        Node prev = (stops.isEmpty() || position == 0) ? depot : stops.get(position - 1).getNode();
        Node next = (stops.isEmpty() || position == stops.size()) ? depot : stops.get(position).getNode();
        return inst.getDistance(prev, node) + inst.getDistance(node, next) - inst.getDistance(prev, next);
    }
}