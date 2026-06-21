import java.util.*;

/**
 * Destroy (removal) operators for the ALNS framework.
 * Upgraded for SD-CTOP and CTOP-T.
 */
public class DestroyOperators {

    private final Random rng;
    // We now have 4 operators!
    public static final int NUM_OPERATORS = 4;

    public DestroyOperators(Random rng) {
        this.rng = rng;
    }

    public List<Node> apply(int operatorIndex, Solution solution, int beta) {
        switch (operatorIndex) {
            case 0: return randomRemoval(solution, beta);
            case 1: return worstProfitRemoval(solution, beta);
            case 2: return shawRemoval(solution, beta);
            case 3: return intersectionRemoval(solution, beta); // NEW OPERATOR
            default: return randomRemoval(solution, beta);
        }
    }

    public String getOperatorName(int index) {
        switch (index) {
            case 0: return "Random_Removal";
            case 1: return "Worst_Profit_Removal";
            case 2: return "Shaw_Distance_Removal";
            case 3: return "Intersecting_Route_Removal"; // NEW OPERATOR
            default: return "Unknown";
        }
    }

    // ══════════════════════════════════════════════════════════
    // OPERATOR 3: INTERSECTING ROUTE REMOVAL (Transfer Enabler)
    // ══════════════════════════════════════════════════════════

    /**
     * Hunts for two routes that are geographically close to each other.
     * Deletes a cluster of nodes around their intersection to create a
     * localized vacuum of time and capacity across multiple vehicles.
     */
    private List<Node> intersectionRemoval(Solution solution, int beta) {
        List<Node> removedNodes = new ArrayList<>();
        List<Route> activeRoutes = new ArrayList<>();

        for (Route r : solution.getRoutes()) {
            if (r.getTotalDemandServed() > 0) activeRoutes.add(r);
        }

        // If less than 2 routes are active, we can't find an intersection. Fallback to Shaw.
        if (activeRoutes.size() < 2) {
            return shawRemoval(solution, beta);
        }

        Instance inst = solution.getInstance();

        // 1. Pick a random active route (Route A)
        Route routeA = activeRoutes.get(rng.nextInt(activeRoutes.size()));

        // 2. Find the Route (Route B) that comes geographically closest to Route A
        Route routeB = null;
        double minDistance = Double.MAX_VALUE;
        Node focalNode = null;

        for (Route r : activeRoutes) {
            if (r.getVehicleId() == routeA.getVehicleId()) continue;

            for (RouteStop stopA : routeA.getStops()) {
                if (!stopA.isServed()) continue;
                for (RouteStop stopB : r.getStops()) {
                    if (!stopB.isServed()) continue;

                    double dist = inst.getDistance(stopA.getNode(), stopB.getNode());
                    if (dist < minDistance) {
                        minDistance = dist;
                        routeB = r;
                        focalNode = stopA.getNode(); // The center of the intersection
                    }
                }
            }
        }

        // Safety check
        if (routeB == null || focalNode == null) return shawRemoval(solution, beta);

        // 3. Collect all served nodes from BOTH Route A and Route B
        List<Node> candidateNodes = new ArrayList<>();
        for (RouteStop s : routeA.getStops()) {
            if (s.isServed() && !candidateNodes.contains(s.getNode())) candidateNodes.add(s.getNode());
        }
        for (RouteStop s : routeB.getStops()) {
            if (s.isServed() && !candidateNodes.contains(s.getNode())) candidateNodes.add(s.getNode());
        }

        // 4. Sort the candidates based on their distance to the Focal Node (the intersection)
        final Node finalFocal = focalNode;
        candidateNodes.sort(Comparator.comparingDouble(n -> inst.getDistance(n, finalFocal)));

        // 5. Remove the top 'beta' nodes (the ones closest to the intersection)
        int numToRemove = Math.min(beta, candidateNodes.size());
        for (int i = 0; i < numToRemove; i++) {
            Node n = candidateNodes.get(i);

            // Remove the node from whichever route(s) it belongs to
            boolean removedFromA = routeA.removeStopByNodeId(n.getId());
            boolean removedFromB = routeB.removeStopByNodeId(n.getId());

            if (removedFromA) routeA.evaluate();
            if (removedFromB) routeB.evaluate();

            removedNodes.add(n);
        }

        return removedNodes;
    }

    // ══════════════════════════════════════════════════════════
    // STANDARD OPERATORS (0, 1, 2)
    // ══════════════════════════════════════════════════════════

    private List<Node> randomRemoval(Solution solution, int beta) {
        List<Node> removedNodes = new ArrayList<>();
        List<RouteStop> servedStops = new ArrayList<>();

        for (Route r : solution.getRoutes()) {
            for (RouteStop s : r.getStops()) {
                if (s.isServed()) servedStops.add(s);
            }
        }

        int removeCount = Math.min(beta, servedStops.size());
        for (int i = 0; i < removeCount; i++) {
            int idx = rng.nextInt(servedStops.size());
            RouteStop stop = servedStops.remove(idx);

            for (Route r : solution.getRoutes()) {
                if (r.removeStopByNodeId(stop.getNode().getId())) {
                    r.evaluate();
                }
            }
            if (!removedNodes.contains(stop.getNode())) {
                removedNodes.add(stop.getNode());
            }
        }
        return removedNodes;
    }

    private List<Node> worstProfitRemoval(Solution solution, int beta) {
        List<Node> removedNodes = new ArrayList<>();
        List<RouteStop> servedStops = new ArrayList<>();

        for (Route r : solution.getRoutes()) {
            for (RouteStop s : r.getStops()) {
                if (s.isServed()) servedStops.add(s);
            }
        }

        // Sort ascending by profit/demand ratio
        servedStops.sort(Comparator.comparingDouble(s -> s.getNode().getProfit() / Math.max(s.getNode().getDemand(), 1e-6)));

        int removeCount = Math.min(beta, servedStops.size());
        for (int i = 0; i < removeCount; i++) {
            Node n = servedStops.get(i).getNode();
            for (Route r : solution.getRoutes()) {
                if (r.removeStopByNodeId(n.getId())) r.evaluate();
            }
            if (!removedNodes.contains(n)) removedNodes.add(n);
        }
        return removedNodes;
    }

    private List<Node> shawRemoval(Solution solution, int beta) {
        List<Node> removedNodes = new ArrayList<>();
        List<RouteStop> servedStops = new ArrayList<>();

        for (Route r : solution.getRoutes()) {
            for (RouteStop s : r.getStops()) {
                if (s.isServed()) servedStops.add(s);
            }
        }

        if (servedStops.isEmpty()) return removedNodes;

        Node seed = servedStops.get(rng.nextInt(servedStops.size())).getNode();
        Instance inst = solution.getInstance();

        servedStops.sort(Comparator.comparingDouble(s -> inst.getDistance(s.getNode(), seed)));

        int removeCount = Math.min(beta, servedStops.size());
        for (int i = 0; i < removeCount; i++) {
            Node n = servedStops.get(i).getNode();
            for (Route r : solution.getRoutes()) {
                if (r.removeStopByNodeId(n.getId())) r.evaluate();
            }
            if (!removedNodes.contains(n)) removedNodes.add(n);
        }
        return removedNodes;
    }
}