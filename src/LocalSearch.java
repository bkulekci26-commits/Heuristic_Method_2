import java.util.List;

/**
 * Local Search operators for CTOP-T-Sync.
 *
 * Implements four improvement operators applied in sequence:
 *   1. Intra-2opt   — reverse segment within route to reduce distance
 *   2. Relocate     — move a node from one route to another (inter-route)
 *   3. Swap         — exchange nodes between two routes (inter-route)
 *   4. Replace      — swap a served node with an unserved node
 *
 * Following the ILS framework from the CTOP literature:
 *   - Each operator runs to first improvement (or best improvement)
 *   - Operators are applied in sequence, restarting from the first
 *     whenever any improvement is found
 *   - Process terminates when a full pass produces no improvement
 *
 * References:
 *   - Tarantilis et al. (2013): VND with 2-opt, 1-1 Exchange, Relocate
 *   - ILS for CTOP (2019): SWAP1, SWAP2, 2-OPT, MOVE, INSERT, REPLACE
 *   - Ben-Said et al. (2016): AIDCH with destruction/construction
 */
public class LocalSearch {

    private int totalImprovements;

    public LocalSearch() {
        this.totalImprovements = 0;
    }

    // ══════════════════════════════════════════════════════════
    // MAIN DRIVER: Apply all operators in VND-style sequence
    // ══════════════════════════════════════════════════════════

    /**
     * Applies local search operators iteratively until no improvement is found.
     *
     * VND-style: cycle through operators in order. When any operator
     * improves the solution, restart from the first operator.
     *
     * @param solution  the solution to improve (modified in place)
     * @return number of improvements made
     */
    public int improve(Solution solution) {
        totalImprovements = 0;
        boolean improved = true;

        while (improved) {
            improved = false;

            // Operator 1: Intra-route 2-opt (reduce distance → more room for time)
            if (apply2Opt(solution)) {
                improved = true;
                continue;  // restart from first operator
            }

            // Operator 2: Inter-route relocate (move node to better route)
            if (applyRelocate(solution)) {
                improved = true;
                continue;
            }

            // Operator 3: Inter-route swap (exchange nodes between routes)
            if (applySwap(solution)) {
                improved = true;
                continue;
            }

            // Operator 4: Replace (swap served with unserved for better objective)
            if (applyReplace(solution)) {
                improved = true;
                continue;
            }
        }

        return totalImprovements;
    }

    // ══════════════════════════════════════════════════════════
    // OPERATOR 1: INTRA-ROUTE 2-OPT
    // ══════════════════════════════════════════════════════════

    /**
     * Reverses a segment [i, j] within a single route to reduce distance.
     * Accepts the first improving move found (first-improvement strategy).
     *
     * For each route, tries all pairs (i, j) where 0 ≤ i < j ≤ stops.size()-1.
     * A reversal is accepted if it reduces total distance while maintaining feasibility.
     *
     * @return true if any improvement was found
     */
    private boolean apply2Opt(Solution solution) {
        for (Route route : solution.getRoutes()) {
            List<RouteStop> stops = route.getStops();
            int n = stops.size();
            if (n < 2) continue;

            double bestDist = route.getTotalDistance();

            for (int i = 0; i < n - 1; i++) {
                for (int j = i + 1; j < n; j++) {
                    // Reverse the segment [i, j]
                    reverseSegment(stops, i, j);
                    route.evaluate();

                    if (route.isFeasible() && route.getTotalDistance() < bestDist - 1e-6) {
                        // Improvement found — keep it
                        bestDist = route.getTotalDistance();
                        totalImprovements++;
                        return true;  // restart
                    } else {
                        // Undo reversal
                        reverseSegment(stops, i, j);
                        route.evaluate();
                    }
                }
            }
        }
        return false;
    }

    /** Reverses stops[i..j] in place */
    private void reverseSegment(List<RouteStop> stops, int i, int j) {
        while (i < j) {
            RouteStop tmp = stops.get(i);
            stops.set(i, stops.get(j));
            stops.set(j, tmp);
            i++;
            j--;
        }
    }

    // ══════════════════════════════════════════════════════════
    // OPERATOR 2: INTER-ROUTE RELOCATE (MOVE)
    // ══════════════════════════════════════════════════════════

    /**
     * Moves a served node from one route to the best position in another route.
     * Accepts the move that produces the best objective improvement.
     *
     * A node is moved if:
     *   1. Removing it from its current route keeps that route feasible
     *   2. Inserting it in the target route at some position is feasible
     *   3. The overall objective value improves
     *
     * This operator primarily helps rebalance routes and may create
     * capacity/time slack for subsequent insertions.
     */
    private boolean applyRelocate(Solution solution) {
        double currentObj = computeObjective(solution);
        List<Route> routes = solution.getRoutes();

        double bestImprovement = 0;
        int bestSourceRoute = -1, bestSourcePos = -1;
        int bestTargetRoute = -1, bestTargetPos = -1;

        for (int r1 = 0; r1 < routes.size(); r1++) {
            Route source = routes.get(r1);
            if (source.isEmpty()) continue;

            for (int sPos = 0; sPos < source.size(); sPos++) {
                RouteStop stop = source.getStops().get(sPos);
                if (!stop.isServed()) continue; // only move served nodes

                // Remove from source
                source.removeStop(sPos);
                source.evaluate();

                if (!source.isFeasible()) {
                    // Restore and skip
                    source.insertStop(sPos, stop);
                    source.evaluate();
                    continue;
                }

                // Try inserting in each other route
                for (int r2 = 0; r2 < routes.size(); r2++) {
                    if (r1 == r2) continue;
                    Route target = routes.get(r2);

                    for (int tPos = 0; tPos <= target.size(); tPos++) {
                        target.insertStop(tPos, stop);
                        target.evaluate();

                        if (target.isFeasible()) {
                            double newObj = computeObjective(solution);
                            double improvement = newObj - currentObj;

                            if (improvement > bestImprovement + 1e-6) {
                                bestImprovement = improvement;
                                bestSourceRoute = r1;
                                bestSourcePos = sPos;
                                bestTargetRoute = r2;
                                bestTargetPos = tPos;
                            }
                        }

                        target.removeStop(tPos);
                        target.evaluate();
                    }
                }

                // Restore source
                source.insertStop(sPos, stop);
                source.evaluate();
            }
        }

        // Execute best move if found
        if (bestImprovement > 1e-6) {
            Route source = routes.get(bestSourceRoute);
            Route target = routes.get(bestTargetRoute);

            RouteStop moved = source.removeStop(bestSourcePos);
            source.evaluate();
            target.insertStop(bestTargetPos, moved);
            target.evaluate();

            totalImprovements++;
            return true;
        }

        return false;
    }

    // ══════════════════════════════════════════════════════════
    // OPERATOR 3: INTER-ROUTE SWAP
    // ══════════════════════════════════════════════════════════

    /**
     * Exchanges one node from route r1 with one node from route r2.
     * Best-improvement: finds the swap with the largest objective gain.
     *
     * This operator can redistribute capacity between routes:
     * e.g., swap a high-demand node for a low-demand node to free capacity.
     */
    private boolean applySwap(Solution solution) {
        double currentObj = computeObjective(solution);
        List<Route> routes = solution.getRoutes();

        double bestImprovement = 0;
        int bestR1 = -1, bestPos1 = -1;
        int bestR2 = -1, bestPos2 = -1;

        for (int r1 = 0; r1 < routes.size(); r1++) {
            for (int r2 = r1 + 1; r2 < routes.size(); r2++) {
                Route route1 = routes.get(r1);
                Route route2 = routes.get(r2);

                for (int p1 = 0; p1 < route1.size(); p1++) {
                    for (int p2 = 0; p2 < route2.size(); p2++) {
                        RouteStop stop1 = route1.getStops().get(p1);
                        RouteStop stop2 = route2.getStops().get(p2);

                        if (!stop1.isServed() || !stop2.isServed()) continue;

                        // Perform swap
                        route1.getStops().set(p1, stop2);
                        route2.getStops().set(p2, stop1);
                        route1.evaluate();
                        route2.evaluate();

                        if (route1.isFeasible() && route2.isFeasible()) {
                            double newObj = computeObjective(solution);
                            double improvement = newObj - currentObj;

                            if (improvement > bestImprovement + 1e-6) {
                                bestImprovement = improvement;
                                bestR1 = r1; bestPos1 = p1;
                                bestR2 = r2; bestPos2 = p2;
                            }
                        }

                        // Undo swap
                        route1.getStops().set(p1, stop1);
                        route2.getStops().set(p2, stop2);
                        route1.evaluate();
                        route2.evaluate();
                    }
                }
            }
        }

        // Execute best swap if found
        if (bestImprovement > 1e-6) {
            Route route1 = routes.get(bestR1);
            Route route2 = routes.get(bestR2);
            RouteStop stop1 = route1.getStops().get(bestPos1);
            RouteStop stop2 = route2.getStops().get(bestPos2);

            route1.getStops().set(bestPos1, stop2);
            route2.getStops().set(bestPos2, stop1);
            route1.evaluate();
            route2.evaluate();

            totalImprovements++;
            return true;
        }

        return false;
    }

    // ══════════════════════════════════════════════════════════
    // OPERATOR 4: REPLACE (most impactful for capacity-constrained CTOP)
    // ══════════════════════════════════════════════════════════

    /**
     * Replaces a served node with an unserved node to improve the objective.
     *
     * Strategy: For each served node s in each route, and each unserved node u:
     *   - Remove s from its route
     *   - Try inserting u at the best position in that route (or any route)
     *   - Accept if feasible and objective improves
     *
     * This is the key operator when capacity is binding: it can swap out
     * a high-demand/low-profit served node for a high-profit/low-demand
     * unserved node, improving profit per unit of capacity.
     */
    private boolean applyReplace(Solution solution) {
        double currentObj = computeObjective(solution);
        List<Route> routes = solution.getRoutes();
        List<Node> unserved = solution.getUnservedNodes();

        if (unserved.isEmpty()) return false;

        double bestImprovement = 0;
        int bestRouteIdx = -1, bestRemovePos = -1;
        int bestInsertRoute = -1, bestInsertPos = -1;
        Node bestUnserved = null;

        for (int ri = 0; ri < routes.size(); ri++) {
            Route route = routes.get(ri);

            for (int sPos = 0; sPos < route.size(); sPos++) {
                RouteStop served = route.getStops().get(sPos);
                if (!served.isServed()) continue;

                // Remove the served node
                route.removeStop(sPos);
                route.evaluate();

                // Try each unserved node
                for (Node uNode : unserved) {
                    // Try inserting in the SAME route (at various positions)
                    for (int iPos = 0; iPos <= route.size(); iPos++) {
                        RouteStop newStop = RouteStop.serve(uNode);
                        route.insertStop(iPos, newStop);
                        route.evaluate();

                        if (route.isFeasible()) {
                            double newObj = computeObjective(solution);
                            double improvement = newObj - currentObj;

                            if (improvement > bestImprovement + 1e-6) {
                                bestImprovement = improvement;
                                bestRouteIdx = ri;
                                bestRemovePos = sPos;
                                bestInsertRoute = ri;
                                bestInsertPos = iPos;
                                bestUnserved = uNode;
                            }
                        }

                        route.removeStop(iPos);
                        route.evaluate();
                    }

                    // Try inserting in OTHER routes too
                    for (int r2 = 0; r2 < routes.size(); r2++) {
                        if (r2 == ri) continue;
                        Route other = routes.get(r2);

                        for (int iPos = 0; iPos <= other.size(); iPos++) {
                            RouteStop newStop = RouteStop.serve(uNode);
                            other.insertStop(iPos, newStop);
                            other.evaluate();

                            if (other.isFeasible()) {
                                double newObj = computeObjective(solution);
                                double improvement = newObj - currentObj;

                                if (improvement > bestImprovement + 1e-6) {
                                    bestImprovement = improvement;
                                    bestRouteIdx = ri;
                                    bestRemovePos = sPos;
                                    bestInsertRoute = r2;
                                    bestInsertPos = iPos;
                                    bestUnserved = uNode;
                                }
                            }

                            other.removeStop(iPos);
                            other.evaluate();
                        }
                    }
                }

                // Restore the removed node
                route.insertStop(sPos, served);
                route.evaluate();
            }
        }

        // Execute best replacement if found
        if (bestImprovement > 1e-6 && bestUnserved != null) {
            Route removeRoute = routes.get(bestRouteIdx);
            Route insertRoute = routes.get(bestInsertRoute);

            // Remove the old served node
            removeRoute.removeStop(bestRemovePos);
            removeRoute.evaluate();

            // Adjust insertion position if same route and position shifted
            int adjustedPos = bestInsertPos;
            if (bestInsertRoute == bestRouteIdx && bestInsertPos > bestRemovePos) {
                adjustedPos--;
            }

            // Insert the new unserved node
            insertRoute.insertStop(adjustedPos, RouteStop.serve(bestUnserved));
            insertRoute.evaluate();

            totalImprovements++;
            return true;
        }

        return false;
    }

    // ══════════════════════════════════════════════════════════
    // POST-OPTIMIZATION: Try inserting unserved nodes after local search
    // ══════════════════════════════════════════════════════════

    /**
     * After local search, try to insert additional unserved nodes
     * into routes that have slack capacity and time.
     *
     * This is the "INSERT" operator from the ILS literature.
     * It exploits any space freed up by 2-opt and relocate.
     */
    public int postInsert(Solution solution) {
        int inserted = 0;
        boolean improved = true;

        while (improved) {
            improved = false;
            List<Node> unserved = solution.getUnservedNodes();

            double bestScore = Double.NEGATIVE_INFINITY;
            Route bestRoute = null;
            int bestPos = -1;
            Node bestNode = null;

            for (Node node : unserved) {
                for (Route route : solution.getRoutes()) {
                    for (int pos = 0; pos <= route.size(); pos++) {
                        RouteStop newStop = RouteStop.serve(node);
                        if (route.canInsert(pos, newStop)) {
                            // Score: profit of the new customer
                            double score = node.getProfit();

                            if (score > bestScore) {
                                bestScore = score;
                                bestRoute = route;
                                bestPos = pos;
                                bestNode = node;
                            }
                        }
                    }
                }
            }

            if (bestNode != null && bestScore > 0) {
                bestRoute.insertStop(bestPos, RouteStop.serve(bestNode));
                bestRoute.evaluate();
                inserted++;
                improved = true;
            }
        }

        return inserted;
    }

    // ══════════════════════════════════════════════════════════
    // UTILITY METHODS
    // ══════════════════════════════════════════════════════════

    /** Compute solution objective: Σ profit */
    private double computeObjective(Solution solution) {
        double profit = 0;
        for (Route r : solution.getRoutes()) {
            profit += r.getTotalProfit();
        }
        return profit;
    }

    /** Compute the extra distance from inserting a node at a given position */
    private double computeInsertionCost(Route route, int position, Node node) {
        Instance inst = route.getInstance();
        Node depot = inst.getDepot();
        List<RouteStop> stops = route.getStops();

        Node prev, next;
        if (stops.isEmpty()) {
            prev = depot; next = depot;
        } else if (position == 0) {
            prev = depot; next = stops.get(0).getNode();
        } else if (position == stops.size()) {
            prev = stops.get(stops.size() - 1).getNode(); next = depot;
        } else {
            prev = stops.get(position - 1).getNode();
            next = stops.get(position).getNode();
        }

        return inst.getDistance(prev, node) + inst.getDistance(node, next)
                - inst.getDistance(prev, next);
    }

    public int getTotalImprovements() { return totalImprovements; }
}