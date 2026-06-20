import java.util.*;

/**
 * Repair (insertion) operators for the ALNS framework.
 * Upgraded for SD-CTOP: Operators now identify partially served customers
 * and attempt to fulfill their remaining demand. If a single route cannot
 * hold the remaining demand, they pool residual capacities to create Split Deliveries.
 */
public class RepairOperators {

    private final Random rng;
    public static final int NUM_OPERATORS = 3;

    /**
     * Score boost applied to a split-delivery insertion relative to a single-route
     * insertion. Lowered from the original 1.5 so that split deliveries no longer
     * crowd out single-route insertions and transfer-enabled completions. Exposed
     * as a tunable parameter (sweep e.g. {1.0, 1.1, 1.25, 1.5}).
     */
    public static double SPLIT_SCORE_MULTIPLIER = 1.1;

    public RepairOperators(Random rng) {
        this.rng = rng;
    }

    public int apply(int operatorIndex, Solution solution, List<Node> removedNodes) {
        // 1. Calculate how much demand is currently met for each customer
        Map<Integer, Double> totalDelivery = new HashMap<>();
        for (Route r : solution.getRoutes()) {
            for (RouteStop s : r.getStops()) {
                if (s.isServed()) totalDelivery.merge(s.getNode().getId(), s.getDeliveryQty(), Double::sum);
            }
        }

        List<Node> candidates = new ArrayList<>();

        // 2. Identify nodes missing demand (priority to recently removed)
        for (Node n : removedNodes) {
            if (totalDelivery.getOrDefault(n.getId(), 0.0) < n.getDemand() - 1e-6) {
                candidates.add(n);
            }
        }

        for (Node n : solution.getInstance().getNodes()) {
            if (!n.isDepot() && !containsNode(candidates, n.getId())) {
                if (totalDelivery.getOrDefault(n.getId(), 0.0) < n.getDemand() - 1e-6) {
                    candidates.add(n);
                }
            }
        }

        switch (operatorIndex) {
            case 0: return greedyBestInsertion(solution, candidates, totalDelivery);
            case 1: return regret2Insertion(solution, candidates, totalDelivery);
            case 2: return randomInsertion(solution, candidates, totalDelivery);
            default: return greedyBestInsertion(solution, candidates, totalDelivery);
        }
    }

    public String getOperatorName(int index) {
        switch (index) {
            case 0: return "GreedyBest_SD";
            case 1: return "Regret-2_SD";
            case 2: return "Random_SD";
            default: return "Unknown";
        }
    }

    // ══════════════════════════════════════════════════════════
    // OPERATOR 0: GREEDY BEST INSERTION (Split-Aware)
    // ══════════════════════════════════════════════════════════

    private int greedyBestInsertion(Solution solution, List<Node> candidates, Map<Integer, Double> totalDelivery) {
        Instance inst = solution.getInstance();
        int inserted = 0;
        boolean improved = true;

        while (improved && !candidates.isEmpty()) {
            improved = false;
            InsertionCandidate best = null;
            int bestCandIdx = -1;

            for (int ci = 0; ci < candidates.size(); ci++) {
                Node cand = candidates.get(ci);
                double missingDemand = cand.getDemand() - totalDelivery.getOrDefault(cand.getId(), 0.0);
                if (missingDemand < 1e-6) continue;

                InsertionCandidate candBest = evaluateInsertion(cand, missingDemand, solution, inst);
                if (candBest != null) {
                    if (best == null || candBest.score > best.score) {
                        best = candBest;
                        bestCandIdx = ci;
                    }
                }
            }

            if (best != null) {
                executeInsertion(best);
                totalDelivery.put(best.node.getId(), best.node.getDemand());
                candidates.remove(bestCandIdx);
                inserted++;
                improved = true;
            }
        }
        return inserted;
    }

    // ══════════════════════════════════════════════════════════
    // OPERATOR 1: REGRET-2 INSERTION (Split-Aware)
    // ══════════════════════════════════════════════════════════

    private int regret2Insertion(Solution solution, List<Node> candidates, Map<Integer, Double> totalDelivery) {
        Instance inst = solution.getInstance();
        int inserted = 0;
        boolean improved = true;

        while (improved && !candidates.isEmpty()) {
            improved = false;
            double maxRegret = Double.NEGATIVE_INFINITY;
            InsertionCandidate bestToInsert = null;
            int bestCandIdx = -1;

            for (int ci = 0; ci < candidates.size(); ci++) {
                Node cand = candidates.get(ci);
                double missingDemand = cand.getDemand() - totalDelivery.getOrDefault(cand.getId(), 0.0);
                if (missingDemand < 1e-6) continue;

                InsertionCandidate candBest = evaluateInsertion(cand, missingDemand, solution, inst);
                if (candBest == null) continue;

                double altScoreSum = 0;
                int altCount = 0;
                for (Route route : solution.getRoutes()) {
                    if (candBest.routes.contains(route)) continue;
                    for (int pos = 0; pos <= route.size(); pos++) {
                        if (route.canInsert(pos, RouteStop.servePartial(cand, Math.min(route.getRemainingCapacity(), missingDemand)))) {
                            altScoreSum += computeInsertionScore(cand, route, pos, inst);
                            altCount++;
                            break;
                        }
                    }
                }

                double regret = (altCount == 0) ? (candBest.score + 1000) : (candBest.score - (altScoreSum / altCount));

                if (regret > maxRegret) {
                    maxRegret = regret;
                    bestToInsert = candBest;
                    bestCandIdx = ci;
                }
            }

            if (bestToInsert != null) {
                executeInsertion(bestToInsert);
                totalDelivery.put(bestToInsert.node.getId(), bestToInsert.node.getDemand());
                candidates.remove(bestCandIdx);
                inserted++;
                improved = true;
            }
        }
        return inserted;
    }

    // ═════════════════════════════════════════════════════════
    // OPERATOR 2: RANDOM INSERTION (Single-only for diversification)
    // ═════════════════════════════════════════════════════════

    private int randomInsertion(Solution solution, List<Node> candidates, Map<Integer, Double> totalDelivery) {
        Instance inst = solution.getInstance();
        int inserted = 0;
        Collections.shuffle(candidates, rng);

        for (Node cand : new ArrayList<>(candidates)) {
            double missingDemand = cand.getDemand() - totalDelivery.getOrDefault(cand.getId(), 0.0);
            if (missingDemand < 1e-6) continue;

            List<Route> shuffledRoutes = new ArrayList<>(solution.getRoutes());
            Collections.shuffle(shuffledRoutes, rng);

            boolean placed = false;
            for (Route route : shuffledRoutes) {
                if (route.getRemainingCapacity() < missingDemand) continue;
                List<Integer> positions = new ArrayList<>();
                for (int p = 0; p <= route.size(); p++) positions.add(p);
                Collections.shuffle(positions, rng);

                for (int pos : positions) {
                    RouteStop newStop = RouteStop.servePartial(cand, missingDemand);
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
    // SPLIT EVALUATION & EXECUTION LOGIC
    // ══════════════════════════════════════════════════════════

    private InsertionCandidate evaluateInsertion(Node cand, double missingDemand, Solution solution, Instance instance) {
        InsertionCandidate best = null;

        // 1. Try Single-Route Insertion
        for (Route route : solution.getRoutes()) {
            if (route.getRemainingCapacity() < missingDemand) continue;
            for (int pos = 0; pos <= route.size(); pos++) {
                if (route.canInsert(pos, RouteStop.servePartial(cand, missingDemand))) {
                    double score = computeInsertionScore(cand, route, pos, instance);
                    if (best == null || score > best.score) {
                        best = new InsertionCandidate(cand, route, pos, missingDemand, score);
                    }
                }
            }
        }

        // 2. Try Split-Route Insertion (Capacity Pooling)
        if (best == null) {
            List<Route> splitRoutes = new ArrayList<>();
            List<Integer> splitPositions = new ArrayList<>();
            List<Double> splitQuantities = new ArrayList<>();
            double accumulated = 0.0;
            double aggregateScore = 0.0;

            for (Route route : solution.getRoutes()) {
                if (accumulated >= missingDemand - 1e-6) break;
                double remCap = route.getRemainingCapacity();
                if (remCap <= 0) continue;

                double qty = Math.min(remCap, missingDemand - accumulated);
                int bestPos = -1;
                double bestPosScore = -1;

                for (int pos = 0; pos <= route.size(); pos++) {
                    if (route.canInsert(pos, RouteStop.servePartial(cand, qty))) {
                        double score = computeInsertionScore(cand, route, pos, instance);
                        if (score > bestPosScore) {
                            bestPosScore = score;
                            bestPos = pos;
                        }
                    }
                }

                if (bestPos != -1) {
                    splitRoutes.add(route);
                    splitPositions.add(bestPos);
                    splitQuantities.add(qty);
                    accumulated += qty;
                    aggregateScore += bestPosScore;
                }
            }

            // Only accept if full demand can be met
            if (Math.abs(accumulated - missingDemand) < 1e-6 && splitRoutes.size() > 1) {
                double avgScore = aggregateScore / splitRoutes.size();

                // --- THE SPLIT INCENTIVE ---
                // Mild boost so split deliveries are explored but no longer dominate
                // single-route insertions and transfer-enabled completions. Tunable.
                best = new InsertionCandidate(cand, splitRoutes, splitPositions, splitQuantities, avgScore * SPLIT_SCORE_MULTIPLIER);
            }
        }
        return best;
    }

    private void executeInsertion(InsertionCandidate plan) {
        for (int i = 0; i < plan.routes.size(); i++) {
            Route r = plan.routes.get(i);
            RouteStop stop = RouteStop.servePartial(plan.node, plan.quantities.get(i));
            r.insertStop(plan.positions.get(i), stop);
            r.evaluate();
        }
    }

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

        double deltaL = inst.getDistance(prev, cand) + inst.getDistance(cand, next) - inst.getDistance(prev, next);
        double distNorm = Math.max(deltaL / inst.getMaxRouteDuration(), 1e-6);
        double capNorm = Math.max(cand.getDemand() / inst.getMaxCapacity(), 1e-6);

        return cand.getProfit() / (distNorm * capNorm);
    }

    private boolean containsNode(List<Node> list, int nodeId) {
        for (Node n : list) {
            if (n.getId() == nodeId) return true;
        }
        return false;
    }

    private static class InsertionCandidate {
        final Node node;
        final List<Route> routes = new ArrayList<>();
        final List<Integer> positions = new ArrayList<>();
        final List<Double> quantities = new ArrayList<>();
        final double score;

        InsertionCandidate(Node node, Route route, int position, double qty, double score) {
            this.node = node;
            this.routes.add(route);
            this.positions.add(position);
            this.quantities.add(qty);
            this.score = score;
        }

        InsertionCandidate(Node node, List<Route> routes, List<Integer> positions, List<Double> quantities, double score) {
            this.node = node;
            this.routes.addAll(routes);
            this.positions.addAll(positions);
            this.quantities.addAll(quantities);
            this.score = score;
        }
    }
}