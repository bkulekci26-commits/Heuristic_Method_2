import java.util.*;

/**
 * Destroy (removal) operators for the ALNS framework.
 *
 * Adapted from Hammami et al. (2024) HALNS for CTOP:
 *   1. Random removal
 *   2. Worst profit removal (lowest profit / demand ratio)
 *   3. Largest demand removal (free up the most capacity)
 *   4. Sequence removal (consecutive nodes in a route)
 *   5. Route removal (clear an entire route for restructuring)
 *
 * Each operator removes nodes from the solution and returns them as a list.
 * The removed nodes become candidates for re-insertion by repair operators.
 * Transfer-related stops (pickup/dropoff-only) are also cleaned up during removal.
 */
public class DestroyOperators {

    private final Random rng;

    // Number of operators available
    public static final int NUM_OPERATORS = 5;

    public DestroyOperators(Random rng) {
        this.rng = rng;
    }

    /**
     * Apply a specific destroy operator.
     *
     * @param operatorIndex  0-4, selects which operator to use
     * @param solution       the solution to destroy (modified in place)
     * @param beta           number of nodes to remove (guide, not strict for all operators)
     * @return list of removed Node objects (available for reinsertion)
     */
    public List<Node> apply(int operatorIndex, Solution solution, int beta) {
        switch (operatorIndex) {
            case 0: return randomRemoval(solution, beta);
            case 1: return worstProfitRemoval(solution, beta);
            case 2: return largestDemandRemoval(solution, beta);
            case 3: return sequenceRemoval(solution, beta);
            case 4: return routeRemoval(solution);
            default: return randomRemoval(solution, beta);
        }
    }

    public String getOperatorName(int index) {
        switch (index) {
            case 0: return "Random";
            case 1: return "WorstProfit";
            case 2: return "LargestDemand";
            case 3: return "Sequence";
            case 4: return "RouteRemoval";
            default: return "Unknown";
        }
    }

    // ══════════════════════════════════════════════════════════
    // OPERATOR 0: RANDOM REMOVAL
    // ══════════════════════════════════════════════════════════

    /**
     * Removes β randomly selected served nodes.
     * Simple but effective for diversification.
     */
    private List<Node> randomRemoval(Solution solution, int beta) {
        List<Node> removed = new ArrayList<>();
        List<int[]> servedPositions = getServedPositions(solution);

        if (servedPositions.isEmpty()) return removed;

        Collections.shuffle(servedPositions, rng);
        int toRemove = Math.min(beta, servedPositions.size());

        // Sort by route index descending, then position descending
        // (to avoid index shifting issues when removing multiple from same route)
        List<int[]> selected = new ArrayList<>(servedPositions.subList(0, toRemove));
        selected.sort((a, b) -> a[0] != b[0] ? b[0] - a[0] : b[1] - a[1]);

        for (int[] pos : selected) {
            Route route = solution.getRoutes().get(pos[0]);
            if (pos[1] < route.size()) {
                RouteStop stop = route.removeStop(pos[1]);
                removed.add(stop.getNode());
                route.evaluate();
            }
        }

        cleanupTransfers(solution, removed);
        return removed;
    }

    // ══════════════════════════════════════════════════════════
    // OPERATOR 1: WORST PROFIT REMOVAL
    // ══════════════════════════════════════════════════════════

    /**
     * Removes β served nodes with the worst profit-to-demand ratio.
     * Uses roulette wheel selection to avoid deterministic cycling.
     *
     * Rationale: Removing low-value nodes frees capacity for
     * potentially higher-value insertions.
     */
    private List<Node> worstProfitRemoval(Solution solution, int beta) {
        List<Node> removed = new ArrayList<>();
        List<int[]> servedPositions = getServedPositions(solution);

        if (servedPositions.isEmpty()) return removed;

        int toRemove = Math.min(beta, servedPositions.size());

        for (int r = 0; r < toRemove; r++) {
            // Rebuild positions list (indexes shift after each removal)
            servedPositions = getServedPositions(solution);
            if (servedPositions.isEmpty()) break;

            // Compute inverse ratio weights (lower ratio = higher removal weight)
            double[] weights = new double[servedPositions.size()];
            double maxRatio = 0;
            for (int i = 0; i < servedPositions.size(); i++) {
                int[] pos = servedPositions.get(i);
                Node n = solution.getRoutes().get(pos[0]).getStops().get(pos[1]).getNode();
                double ratio = n.getProfit() / Math.max(n.getDemand(), 0.1);
                maxRatio = Math.max(maxRatio, ratio);
            }

            for (int i = 0; i < servedPositions.size(); i++) {
                int[] pos = servedPositions.get(i);
                Node n = solution.getRoutes().get(pos[0]).getStops().get(pos[1]).getNode();
                double ratio = n.getProfit() / Math.max(n.getDemand(), 0.1);
                weights[i] = maxRatio - ratio + 0.1; // inverse: low ratio → high weight
            }

            int selected = rouletteSelect(weights);
            int[] pos = servedPositions.get(selected);
            Route route = solution.getRoutes().get(pos[0]);
            RouteStop stop = route.removeStop(pos[1]);
            removed.add(stop.getNode());
            route.evaluate();
        }

        cleanupTransfers(solution, removed);
        return removed;
    }

    // ══════════════════════════════════════════════════════════
    // OPERATOR 2: LARGEST DEMAND REMOVAL
    // ══════════════════════════════════════════════════════════

    /**
     * Removes β served nodes with the largest demand.
     * Uses roulette wheel to avoid cycling.
     *
     * Rationale: Freeing up the most capacity per removal,
     * creating larger insertion slots for new customers.
     * This operator is specifically designed for the CTOP (Hammami 2024).
     */
    private List<Node> largestDemandRemoval(Solution solution, int beta) {
        List<Node> removed = new ArrayList<>();

        int toRemove = Math.min(beta, countServedNodes(solution));

        for (int r = 0; r < toRemove; r++) {
            List<int[]> servedPositions = getServedPositions(solution);
            if (servedPositions.isEmpty()) break;

            // Weight by demand (higher demand → higher removal probability)
            double[] weights = new double[servedPositions.size()];
            for (int i = 0; i < servedPositions.size(); i++) {
                int[] pos = servedPositions.get(i);
                Node n = solution.getRoutes().get(pos[0]).getStops().get(pos[1]).getNode();
                weights[i] = n.getDemand() + 0.1;
            }

            int selected = rouletteSelect(weights);
            int[] pos = servedPositions.get(selected);
            Route route = solution.getRoutes().get(pos[0]);
            RouteStop stop = route.removeStop(pos[1]);
            removed.add(stop.getNode());
            route.evaluate();
        }

        cleanupTransfers(solution, removed);
        return removed;
    }

    // ══════════════════════════════════════════════════════════
    // OPERATOR 3: SEQUENCE REMOVAL
    // ══════════════════════════════════════════════════════════

    /**
     * Randomly selects a route and removes a consecutive sequence of β nodes.
     *
     * This was identified as the most impactful removal operator in
     * Hammami (2024)'s sensitivity analysis. It creates a contiguous
     * "slot" in a route that allows inserting a fundamentally different
     * sequence of customers.
     */
    private List<Node> sequenceRemoval(Solution solution, int beta) {
        List<Node> removed = new ArrayList<>();
        List<Route> nonEmpty = getNonEmptyRoutes(solution);

        if (nonEmpty.isEmpty()) return removed;

        // Pick a random non-empty route
        Route route = nonEmpty.get(rng.nextInt(nonEmpty.size()));
        int n = route.size();
        if (n == 0) return removed;

        int seqLen = Math.min(beta, n);
        int startPos = rng.nextInt(n - seqLen + 1);

        // Remove from end to start (to keep indexes valid)
        for (int i = startPos + seqLen - 1; i >= startPos; i--) {
            RouteStop stop = route.removeStop(i);
            if (stop.isServed()) {
                removed.add(stop.getNode());
            }
        }
        route.evaluate();

        cleanupTransfers(solution, removed);
        return removed;
    }

    // ══════════════════════════════════════════════════════════
    // OPERATOR 4: ROUTE REMOVAL
    // ══════════════════════════════════════════════════════════

    /**
     * Removes all served nodes from a randomly selected route.
     * β parameter is ignored — the entire route is cleared.
     *
     * Powerful diversification: completely restructures one vehicle's path.
     */
    private List<Node> routeRemoval(Solution solution) {
        List<Node> removed = new ArrayList<>();
        List<Route> nonEmpty = getNonEmptyRoutes(solution);

        if (nonEmpty.isEmpty()) return removed;

        Route route = nonEmpty.get(rng.nextInt(nonEmpty.size()));

        // Remove all stops (iterate backwards)
        for (int i = route.size() - 1; i >= 0; i--) {
            RouteStop stop = route.removeStop(i);
            if (stop.isServed()) {
                removed.add(stop.getNode());
            }
        }
        route.evaluate();

        cleanupTransfers(solution, removed);
        return removed;
    }

    // ══════════════════════════════════════════════════════════
    // UTILITIES
    // ══════════════════════════════════════════════════════════

    /** Returns list of [routeIndex, stopPosition] for all served nodes */
    private List<int[]> getServedPositions(Solution solution) {
        List<int[]> positions = new ArrayList<>();
        List<Route> routes = solution.getRoutes();
        for (int ri = 0; ri < routes.size(); ri++) {
            List<RouteStop> stops = routes.get(ri).getStops();
            for (int si = 0; si < stops.size(); si++) {
                if (stops.get(si).isServed()) {
                    positions.add(new int[]{ri, si});
                }
            }
        }
        return positions;
    }

    /** Returns non-empty routes */
    private List<Route> getNonEmptyRoutes(Solution solution) {
        List<Route> nonEmpty = new ArrayList<>();
        for (Route r : solution.getRoutes()) {
            if (!r.isEmpty()) nonEmpty.add(r);
        }
        return nonEmpty;
    }

    /** Counts total served nodes in the solution */
    private int countServedNodes(Solution solution) {
        int count = 0;
        for (Route r : solution.getRoutes()) {
            for (RouteStop s : r.getStops()) {
                if (s.isServed()) count++;
            }
        }
        return count;
    }

    /** Roulette wheel selection based on weights */
    private int rouletteSelect(double[] weights) {
        double total = 0;
        for (double w : weights) total += w;
        if (total <= 0) return rng.nextInt(weights.length);

        double r = rng.nextDouble() * total;
        double cumulative = 0;
        for (int i = 0; i < weights.length; i++) {
            cumulative += weights[i];
            if (r <= cumulative) return i;
        }
        return weights.length - 1;
    }

    /**
     * Cleans up transfer-related stops after removal.
     * If a node was involved in a transfer (as giver or receiver),
     * we need to remove the corresponding pickup/dropoff stops
     * and the Transfer object from the solution.
     */
    private void cleanupTransfers(Solution solution, List<Node> removedNodes) {
        Set<Integer> removedIds = new HashSet<>();
        for (Node n : removedNodes) removedIds.add(n.getId());

        // Remove any Transfer objects involving removed nodes
        Iterator<Transfer> it = solution.getTransfers().iterator();
        while (it.hasNext()) {
            Transfer t = it.next();
            // If the transfer node was removed, or involved vehicles' stops changed
            if (removedIds.contains(t.getTransferNodeId())) {
                it.remove();
                // Also clean pickup/dropoff stops in routes
                cleanupTransferStops(solution, t);
            }
        }

        // Also remove any transfer-only stops (pickup/dropoff) that are now orphaned
        for (Route r : solution.getRoutes()) {
            boolean changed = false;
            for (int i = r.size() - 1; i >= 0; i--) {
                RouteStop stop = r.getStops().get(i);
                if (stop.isTransferOnly()) {
                    // Check if this stop still has a valid transfer
                    boolean hasValidTransfer = false;
                    for (Transfer t : solution.getTransfers()) {
                        if (t.getTransferNodeId() == stop.getNode().getId()) {
                            hasValidTransfer = true;
                            break;
                        }
                    }
                    if (!hasValidTransfer) {
                        r.removeStop(i);
                        changed = true;
                    }
                }
                // If a served+dropoff/pickup stop lost its transfer, revert to serve-only
                if (stop.isServed() && (stop.isDropoff() || stop.isPickup())) {
                    boolean hasValidTransfer = false;
                    for (Transfer t : solution.getTransfers()) {
                        if (t.getTransferNodeId() == stop.getNode().getId()) {
                            hasValidTransfer = true;
                            break;
                        }
                    }
                    if (!hasValidTransfer) {
                        r.getStops().set(i, RouteStop.serve(stop.getNode()));
                        changed = true;
                    }
                }
            }
            if (changed) r.evaluate();
        }
    }

    /** Removes pickup/dropoff stops associated with a specific transfer */
    private void cleanupTransferStops(Solution solution, Transfer t) {
        for (Route r : solution.getRoutes()) {
            boolean changed = false;
            for (int i = r.size() - 1; i >= 0; i--) {
                RouteStop stop = r.getStops().get(i);
                if (stop.getNode().getId() == t.getTransferNodeId()) {
                    if (stop.isTransferOnly()) {
                        r.removeStop(i);
                        changed = true;
                    } else if (stop.isServed() && (stop.isDropoff() || stop.isPickup())) {
                        // Revert to serve-only
                        r.getStops().set(i, RouteStop.serve(stop.getNode()));
                        changed = true;
                    }
                }
            }
            if (changed) r.evaluate();
        }
    }
}