import java.util.*;

/**
 * Repair (insertion) operators for the ALNS framework.
 *
 * These operators attempt to reinsert nodes that were removed by destroy operators,
 * plus any previously unserved nodes, into the current solution.
 *
 * Adapted from Hammami et al. (2024) HALNS and Ropke & Pisinger (2006):
 *   1. Greedy best insertion — insert the node with the best score at best position
 *   2. Regret-2 insertion — insert the node that would lose most if not inserted now
 *   3. Random insertion — insert nodes at random feasible positions
 *
 * All operators respect CTOP constraints (time + capacity) during insertion.
 */
public class RepairOperators {

    private final Random rng;

    // Number of operators available
    public static final int NUM_OPERATORS = 3;

    public RepairOperators(Random rng) {
        this.rng = rng;
    }

    /**
     * Apply a specific repair operator.
     *
     * @param operatorIndex   0-2, selects which operator to use
     * @param solution        the solution to repair (modified in place)
     * @param removedNodes    nodes that were just removed (priority for reinsertion)
     * @return number of nodes successfully inserted
     */
    public int apply(int operatorIndex, Solution solution, List<Node> removedNodes) {
        // Build the full candidate pool: removed nodes + already unserved nodes
        Set<Integer> served = solution.getServedNodeIds();
        List<Node> candidates = new ArrayList<>();

        // Add removed nodes first (priority)
        for (Node n : removedNodes) {
            if (!served.contains(n.getId())) {
                candidates.add(n);
            }
        }

        // Add other unserved nodes
        for (Node n : solution.getInstance().getNodes()) {
            if (!n.isDepot() && !served.contains(n.getId())
                    && !containsNode(candidates, n.getId())) {
                candidates.add(n);
            }
        }

        switch (operatorIndex) {
            case 0: return greedyBestInsertion(solution, candidates);
            case 1: return regret2Insertion(solution, candidates);
            case 2: return randomInsertion(solution, candidates);
            default: return greedyBestInsertion(solution, candidates);
        }
    }

    public String getOperatorName(int index) {
        switch (index) {
            case 0: return "GreedyBest";
            case 1: return "Regret-2";
            case 2: return "Random";
            default: return "Unknown";
        }
    }

    // ══════════════════════════════════════════════════════════
    // OPERATOR 0: GREEDY BEST INSERTION
    // ══════════════════════════════════════════════════════════

    /**
     * Iteratively inserts the candidate with the best insertion score
     * at its best feasible position across all routes.
     *
     * Score: profit / ((ΔL / Tmax) × (demand / Q))
     * Higher score = better candidate.
     *
     * Continues until no feasible insertion exists.
     */
    private int greedyBestInsertion(Solution solution, List<Node> candidates) {
        Instance inst = solution.getInstance();
        int inserted = 0;
        boolean improved = true;

        while (improved && !candidates.isEmpty()) {
            improved = false;
            double bestScore = Double.NEGATIVE_INFINITY;
            Node bestNode = null;
            Route bestRoute = null;
            int bestPos = -1;
            int bestCandIdx = -1;

            for (int ci = 0; ci < candidates.size(); ci++) {
                Node cand = candidates.get(ci);

                for (Route route : solution.getRoutes()) {
                    for (int pos = 0; pos <= route.size(); pos++) {
                        RouteStop newStop = RouteStop.serve(cand);
                        if (route.canInsert(pos, newStop)) {
                            double score = computeInsertionScore(cand, route, pos, inst);
                            if (score > bestScore) {
                                bestScore = score;
                                bestNode = cand;
                                bestRoute = route;
                                bestPos = pos;
                                bestCandIdx = ci;
                            }
                        }
                    }
                }
            }

            if (bestNode != null) {
                bestRoute.insertStop(bestPos, RouteStop.serve(bestNode));
                bestRoute.evaluate();
                candidates.remove(bestCandIdx);
                inserted++;
                improved = true;
            }
        }

        return inserted;
    }

    // ══════════════════════════════════════════════════════════
    // OPERATOR 1: REGRET-2 INSERTION
    // ══════════════════════════════════════════════════════════

    /**
     * Regret-based insertion (Ropke & Pisinger, 2006).
     *
     * For each candidate, compute the difference between its best and
     * second-best insertion cost. The candidate with the largest "regret"
     * (i.e., the one that would lose the most if not inserted now) is
     * inserted first.
     *
     * This avoids the greedy trap of inserting "easy" nodes first while
     * leaving "difficult" nodes with no feasible position later.
     */
    private int regret2Insertion(Solution solution, List<Node> candidates) {
        Instance inst = solution.getInstance();
        int inserted = 0;
        boolean improved = true;

        while (improved && !candidates.isEmpty()) {
            improved = false;
            double maxRegret = Double.NEGATIVE_INFINITY;
            Node bestNode = null;
            Route bestRoute = null;
            int bestPos = -1;
            int bestCandIdx = -1;

            for (int ci = 0; ci < candidates.size(); ci++) {
                Node cand = candidates.get(ci);

                // Find top-2 insertion scores for this candidate
                double best1Score = Double.NEGATIVE_INFINITY;
                double best2Score = Double.NEGATIVE_INFINITY;
                Route best1Route = null;
                int best1Pos = -1;

                for (Route route : solution.getRoutes()) {
                    for (int pos = 0; pos <= route.size(); pos++) {
                        RouteStop newStop = RouteStop.serve(cand);
                        if (route.canInsert(pos, newStop)) {
                            double score = computeInsertionScore(cand, route, pos, inst);
                            if (score > best1Score) {
                                best2Score = best1Score;
                                best1Score = score;
                                best1Route = route;
                                best1Pos = pos;
                            } else if (score > best2Score) {
                                best2Score = score;
                            }
                        }
                    }
                }

                if (best1Route == null) continue; // no feasible insertion

                // Regret = difference between best and second-best
                // If no second-best exists, regret is very high (must insert now)
                double regret;
                if (best2Score == Double.NEGATIVE_INFINITY) {
                    regret = best1Score + 1000; // only one option → high regret
                } else {
                    regret = best1Score - best2Score;
                }

                if (regret > maxRegret) {
                    maxRegret = regret;
                    bestNode = cand;
                    bestRoute = best1Route;
                    bestPos = best1Pos;
                    bestCandIdx = ci;
                }
            }

            if (bestNode != null) {
                bestRoute.insertStop(bestPos, RouteStop.serve(bestNode));
                bestRoute.evaluate();
                candidates.remove(bestCandIdx);
                inserted++;
                improved = true;
            }
        }

        return inserted;
    }

    // ══════════════════════════════════════════════════════════
    // OPERATOR 2: RANDOM INSERTION
    // ══════════════════════════════════════════════════════════

    /**
     * Inserts candidates in random order at the first feasible position found.
     * Provides maximum diversification — the resulting solution structure
     * may be very different from greedy approaches.
     */
    private int randomInsertion(Solution solution, List<Node> candidates) {
        Instance inst = solution.getInstance();
        int inserted = 0;

        // Shuffle candidates for randomness
        Collections.shuffle(candidates, rng);

        for (Node cand : new ArrayList<>(candidates)) {
            // Shuffle routes too
            List<Route> shuffledRoutes = new ArrayList<>(solution.getRoutes());
            Collections.shuffle(shuffledRoutes, rng);

            boolean placed = false;
            for (Route route : shuffledRoutes) {
                // Try positions in random order
                List<Integer> positions = new ArrayList<>();
                for (int p = 0; p <= route.size(); p++) positions.add(p);
                Collections.shuffle(positions, rng);

                for (int pos : positions) {
                    RouteStop newStop = RouteStop.serve(cand);
                    if (route.canInsert(pos, newStop)) {
                        route.insertStop(pos, newStop);
                        route.evaluate();
                        inserted++;
                        placed = true;
                        break;
                    }
                }
                if (placed) break;
            }
        }

        return inserted;
    }

    // ══════════════════════════════════════════════════════════
    // SCORING FUNCTION
    // ══════════════════════════════════════════════════════════

    /**
     * Computes insertion score: profit / (normalized_detour × normalized_demand)
     * Higher = better insertion.
     */
    private double computeInsertionScore(Node cand, Route route, int position, Instance inst) {
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

        double deltaL = inst.getDistance(prev, cand) + inst.getDistance(cand, next)
                - inst.getDistance(prev, next);

        double distNorm = Math.max(deltaL / inst.getMaxRouteDuration(), 1e-6);
        double capNorm = Math.max(cand.getDemand() / inst.getMaxCapacity(), 1e-6);

        return cand.getProfit() / (distNorm * capNorm);
    }

    // ══════════════════════════════════════════════════════════
    // UTILITIES
    // ══════════════════════════════════════════════════════════

    private boolean containsNode(List<Node> list, int nodeId) {
        for (Node n : list) {
            if (n.getId() == nodeId) return true;
        }
        return false;
    }
}