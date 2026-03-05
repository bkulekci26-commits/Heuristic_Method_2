import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Set;

/**
 * Greedy Constructive Heuristic for CTOP-T-Sync.
 *
 * Builds an initial feasible solution (without transfers) using a
 * Best Insertion Algorithm (BIA). At each iteration, the unserved
 * customer with the best insertion score is inserted at its best
 * feasible position across all routes.
 *
 * Inspired by:
 *   - Ben-Said et al. (2016): ratio-based BIA with adaptive weights
 *   - Tarantilis et al. (2013): parallel insertion favoring high-profit customers
 *   - Hammami et al. (2024): nearest neighbor initialization for HALNS
 *
 * Score function for inserting customer c at position p in route k:
 *
 *   score(c, p, k) = pc^α / ( (ΔL_cp / Tmax)^β  ×  (dc / Q)^γ )
 *
 * where:
 *   pc     = profit of customer c
 *   ΔL_cp  = extra distance from inserting c at position p
 *   dc     = demand of customer c
 *   α, β, γ = weights controlling profit, distance, capacity importance
 *
 * Higher score = better insertion candidate.
 */
public class GreedyConstructive {

    // Weight parameters for the scoring function
    private final double alphaWeight;   // profit importance
    private final double betaWeight;    // distance penalty importance
    private final double gammaWeight;   // capacity consumption importance

    private final double objectiveAlpha; // α in the objective function (distance penalty)

    // ──────────────────────────── Constructor ────────────────────────────────

    /**
     * @param alphaWeight   profit exponent (higher = favor high-profit nodes)
     * @param betaWeight    distance exponent (higher = penalize detours more)
     * @param gammaWeight   capacity exponent (higher = penalize heavy demands)
     * @param objectiveAlpha  the α coefficient in the objective function
     */
    public GreedyConstructive(double alphaWeight, double betaWeight,
                              double gammaWeight, double objectiveAlpha) {
        this.alphaWeight = alphaWeight;
        this.betaWeight = betaWeight;
        this.gammaWeight = gammaWeight;
        this.objectiveAlpha = objectiveAlpha;
    }

    /** Default constructor with balanced weights */
    public GreedyConstructive(double objectiveAlpha) {
        this(1.0, 1.0, 0.5, objectiveAlpha);
    }

    // ──────────────────────────── Main Algorithm ─────────────────────────────

    /**
     * Constructs an initial feasible solution (no transfers).
     *
     * Algorithm:
     *   1. Initialize K empty routes (depot → depot)
     *   2. Pre-filter: only consider customers within Tmax/2 radius of depot
     *   3. Repeat until no feasible insertion exists:
     *      a. For each unserved customer c:
     *         - For each route k and each position p in k:
     *           - If inserting c at (k, p) is feasible: compute score
     *      b. Execute the insertion with the highest score
     *   4. Return the solution
     */
    public Solution construct(Instance instance) {
        Solution solution = new Solution(instance, objectiveAlpha);

        // Step 1: Initialize empty routes
        for (int k = 0; k < instance.getMaxVehicles(); k++) {
            solution.addRoute(new Route(k, instance));
        }

        // Step 2: Build candidate list (nodes reachable from depot within Tmax)
        Node depot = instance.getDepot();
        double maxRadius = instance.getMaxRouteDuration() / 2.0;

        List<Node> candidates = new ArrayList<>();
        for (Node n : instance.getNodes()) {
            if (!n.isDepot()) {
                double distToDepot = instance.getDistance(depot, n);
                if (distToDepot <= maxRadius) {
                    candidates.add(n);
                }
            }
        }

        // Sort candidates by profit/demand ratio (descending) for tie-breaking
        candidates.sort(Comparator.comparingDouble(
                (Node n) -> n.getProfit() / Math.max(n.getDemand(), 0.1)).reversed());

        System.out.printf("[Constructive] %d candidates within radius %.1f of depot%n",
                candidates.size(), maxRadius);

        // Step 3: Iterative best insertion
        int insertionCount = 0;
        boolean improved = true;

        while (improved) {
            improved = false;
            InsertionCandidate best = null;

            Set<Integer> served = solution.getServedNodeIds();

            for (Node cand : candidates) {
                if (served.contains(cand.getId())) continue;

                // Try inserting in each route at each position
                for (Route route : solution.getRoutes()) {
                    int maxPos = route.size() + 1; // can insert at 0..size (before any stop or at end)

                    for (int pos = 0; pos < maxPos; pos++) {
                        // Quick feasibility check
                        RouteStop newStop = RouteStop.serve(cand);
                        if (route.canInsert(pos, newStop)) {
                            double score = computeScore(cand, route, pos, instance);
                            if (best == null || score > best.score) {
                                best = new InsertionCandidate(cand, route, pos, score);
                            }
                        }
                    }
                }
            }

            // Execute best insertion
            if (best != null) {
                best.route.insertStop(best.position, RouteStop.serve(best.node));
                best.route.evaluate();
                insertionCount++;
                improved = true;

                if (insertionCount % 20 == 0) {
                    System.out.printf("[Constructive] %d nodes inserted, profit=%.1f, dist=%.1f%n",
                            insertionCount, solution.getTotalProfit(), solution.getTotalDistance());
                }
            }
        }

        // Evaluate all routes
        for (Route r : solution.getRoutes()) {
            r.evaluate();
        }

        System.out.printf("[Constructive] DONE: %d nodes served out of %d candidates%n",
                insertionCount, candidates.size());
        System.out.printf("[Constructive] Profit=%.1f, Distance=%.1f, Objective=%.2f%n",
                solution.getTotalProfit(), solution.getTotalDistance(), solution.getObjectiveValue());

        return solution;
    }

    // ──────────────────────────── Scoring Function ───────────────────────────

    /**
     * Computes the insertion score for placing customer c at position pos in route.
     *
     * score = profit^α / ( (ΔL/Tmax)^β × (demand/Q)^γ )
     *
     * ΔL is the extra distance caused by the insertion:
     *   ΔL = dist(prev, c) + dist(c, next) - dist(prev, next)
     *
     * Higher score = better candidate.
     */
    private double computeScore(Node cand, Route route, int position, Instance instance) {
        // Determine prev and next nodes around insertion point
        Node depot = instance.getDepot();
        Node prev, next;

        List<RouteStop> stops = route.getStops();

        if (stops.isEmpty()) {
            // Empty route: depot → cand → depot
            prev = depot;
            next = depot;
        } else if (position == 0) {
            // Insert before first stop: depot → cand → stops[0]
            prev = depot;
            next = stops.get(0).getNode();
        } else if (position == stops.size()) {
            // Insert after last stop: stops[last] → cand → depot
            prev = stops.get(stops.size() - 1).getNode();
            next = depot;
        } else {
            // Insert between stops[position-1] and stops[position]
            prev = stops.get(position - 1).getNode();
            next = stops.get(position).getNode();
        }

        // Extra distance (detour cost)
        double deltaL = instance.getDistance(prev, cand)
                + instance.getDistance(cand, next)
                - instance.getDistance(prev, next);

        // Normalize components
        double profitNorm = cand.getProfit();
        double distNorm = deltaL / instance.getMaxRouteDuration();
        double capNorm = cand.getDemand() / instance.getMaxCapacity();

        // Avoid division by zero
        double epsilon = 1e-6;
        distNorm = Math.max(distNorm, epsilon);
        capNorm = Math.max(capNorm, epsilon);

        // Score: higher is better
        double score = Math.pow(profitNorm, alphaWeight)
                / (Math.pow(distNorm, betaWeight) * Math.pow(capNorm, gammaWeight));

        return score;
    }

    // ──────────────────────────── Helper Class ───────────────────────────────

    /** Stores a candidate insertion (node, route, position, score) */
    private static class InsertionCandidate {
        final Node node;
        final Route route;
        final int position;
        final double score;

        InsertionCandidate(Node node, Route route, int position, double score) {
            this.node = node;
            this.route = route;
            this.position = position;
            this.score = score;
        }

        @Override
        public String toString() {
            return String.format("Insert node %d at pos %d in route v%d (score=%.2f)",
                    node.getId(), position, route.getVehicleId(), score);
        }
    }
}