import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Set;

/**
 * Greedy Constructive Heuristic for CTOP-T-Sync.
 *
 * Builds an initial feasible solution using a Best Insertion Algorithm (BIA).
 * At each iteration, the unserved customer with the best insertion score is
 * inserted at its best feasible position.
 * * SPLIT DELIVERY UPGRADE: If a customer cannot fit into any single route,
 * the heuristic pools the residual capacity of multiple routes to fulfill
 * 100% of the demand via split deliveries.
 */
public class GreedyConstructive {

    // Weight parameters for the scoring function
    private final double alphaWeight;   // profit importance
    private final double betaWeight;    // distance penalty importance
    private final double gammaWeight;   // capacity consumption importance

    // ──────────────────────────── Constructor ────────────────────────────────

    public GreedyConstructive(double alphaWeight, double betaWeight, double gammaWeight) {
        this.alphaWeight = alphaWeight;
        this.betaWeight = betaWeight;
        this.gammaWeight = gammaWeight;
    }

    public GreedyConstructive() {
        this(1.0, 1.0, 0.5);
    }

    // ────────────────────────── Main Algorithm ───────────────────────────

    public Solution construct(Instance instance) {
        Solution solution = new Solution(instance);

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

                // ---------------------------------------------------------
                // PHASE 1: Try standard SINGLE-ROUTE insertion first
                // ---------------------------------------------------------
                for (Route route : solution.getRoutes()) {
                    int maxPos = route.size() + 1;

                    for (int pos = 0; pos < maxPos; pos++) {
                        RouteStop newStop = RouteStop.serve(cand);
                        if (route.canInsert(pos, newStop)) {
                            double score = computeScore(cand, route, pos, instance);
                            if (best == null || score > best.score) {
                                best = new InsertionCandidate(cand, route, pos, score);
                            }
                        }
                    }
                }

                // ---------------------------------------------------------
                // PHASE 2: Try SPLIT-DELIVERY if single-route failed
                // ---------------------------------------------------------
                if (best == null) {
                    List<Route> splitRoutes = new ArrayList<>();
                    List<Integer> splitPositions = new ArrayList<>();
                    List<Double> splitQuantities = new ArrayList<>();
                    double accumulated = 0.0;
                    double aggregateScore = 0.0;

                    for (Route route : solution.getRoutes()) {
                        if (accumulated >= cand.getDemand() - 1e-6) break; // Demand fulfilled!

                        double remCap = route.getRemainingCapacity();
                        if (remCap <= 0) continue;

                        double qty = Math.min(remCap, cand.getDemand() - accumulated);
                        int bestPos = -1;
                        double bestPosScore = -1;

                        // Find the best position for this partial quantity in this route
                        for (int pos = 0; pos <= route.size(); pos++) {
                            if (route.canInsert(pos, RouteStop.servePartial(cand, qty))) {
                                double score = computeScore(cand, route, pos, instance);
                                if (score > bestPosScore) {
                                    bestPosScore = score;
                                    bestPos = pos;
                                }
                            }
                        }

                        // If a valid position was found in this route, add it to the pool
                        if (bestPos != -1) {
                            splitRoutes.add(route);
                            splitPositions.add(bestPos);
                            splitQuantities.add(qty);
                            accumulated += qty;
                            aggregateScore += bestPosScore;
                        }
                    }

                    // Only accept the split plan if it successfully pooled enough capacity
                    // to hit 100% of the demand (respecting the all-or-nothing rule)
                    if (Math.abs(accumulated - cand.getDemand()) < 1e-6 && splitRoutes.size() > 1) {
                        double finalSplitScore = aggregateScore / splitRoutes.size(); // Average score
                        if (best == null || finalSplitScore > best.score) {
                            best = new InsertionCandidate(cand, splitRoutes, splitPositions, splitQuantities, finalSplitScore);
                        }
                    }
                }
            }

            // ---------------------------------------------------------
            // EXECUTE the best insertion (Works for both Single and Split)
            // ---------------------------------------------------------
            if (best != null) {
                for (int i = 0; i < best.routes.size(); i++) {
                    Route r = best.routes.get(i);
                    // Use servePartial. If it's a single route, best.quantities.get(i) is exactly the full demand
                    RouteStop stop = RouteStop.servePartial(best.node, best.quantities.get(i));
                    r.insertStop(best.positions.get(i), stop);
                    r.evaluate();
                }
                insertionCount++;
                improved = true;

                if (insertionCount % 20 == 0) {
                    System.out.printf("[Constructive] %d nodes inserted, profit=%.1f, dist=%.1f%n",
                            insertionCount, solution.getTotalProfit(), solution.getTotalDistance());
                }
            }
        }

        // Evaluate all routes at the end to finalize arrival times
        for (Route r : solution.getRoutes()) {
            r.evaluate();
        }

        System.out.printf("[Constructive] DONE: %d nodes served out of %d candidates%n",
                insertionCount, candidates.size());
        System.out.printf("[Constructive] Profit=%.1f, Distance=%.1f%n",
                solution.getTotalProfit(), solution.getTotalDistance());

        return solution;
    }

    // ──────────────────────────── Scoring Function ───────────────────────────

    private double computeScore(Node cand, Route route, int position, Instance instance) {
        Node depot = instance.getDepot();
        Node prev, next;
        List<RouteStop> stops = route.getStops();

        if (stops.isEmpty()) {
            prev = depot;
            next = depot;
        } else if (position == 0) {
            prev = depot;
            next = stops.get(0).getNode();
        } else if (position == stops.size()) {
            prev = stops.get(stops.size() - 1).getNode();
            next = depot;
        } else {
            prev = stops.get(position - 1).getNode();
            next = stops.get(position).getNode();
        }

        double deltaL = instance.getDistance(prev, cand)
                + instance.getDistance(cand, next)
                - instance.getDistance(prev, next);

        double profitNorm = cand.getProfit();
        double distNorm = deltaL / instance.getMaxRouteDuration();
        double capNorm = cand.getDemand() / instance.getMaxCapacity();

        double epsilon = 1e-6;
        distNorm = Math.max(distNorm, epsilon);
        capNorm = Math.max(capNorm, epsilon);

        return Math.pow(profitNorm, alphaWeight)
                / (Math.pow(distNorm, betaWeight) * Math.pow(capNorm, gammaWeight));
    }

    // ──────────────────────────── Helper Class ───────────────────────────────

    /** Stores a candidate insertion (node, routes, positions, quantities, score) */
    private static class InsertionCandidate {
        final Node node;
        final List<Route> routes = new ArrayList<>();
        final List<Integer> positions = new ArrayList<>();
        final List<Double> quantities = new ArrayList<>();
        final double score;

        // Backward-compatible constructor for standard single-route insertion
        InsertionCandidate(Node node, Route route, int position, double score) {
            this.node = node;
            this.routes.add(route);
            this.positions.add(position);
            this.quantities.add(node.getDemand());
            this.score = score;
        }

        // New constructor for split delivery insertion
        InsertionCandidate(Node node, List<Route> routes, List<Integer> positions, List<Double> quantities, double score) {
            this.node = node;
            this.routes.addAll(routes);
            this.positions.addAll(positions);
            this.quantities.addAll(quantities);
            this.score = score;
        }

        @Override
        public String toString() {
            return String.format("Insert node %d into %d route(s) (score=%.2f)",
                    node.getId(), routes.size(), score);
        }
    }
}