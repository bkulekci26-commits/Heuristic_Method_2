import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Represents a complete solution to the CTOP-T-Sync problem.
 *
 * A solution consists of:
 *   - A set of routes (one per vehicle, some may be empty)
 *   - A set of transfers between vehicles at customer nodes
 *
 * Objective function (eq. 1):
 *   max Z = Σ pj·zjk - α · Σ cij·xijk
 *         = total_profit - α · total_distance
 *
 * Global feasibility requires:
 *   1. Each route individually feasible (time + capacity)
 *   2. Each customer served at most once across all routes (constraint 5)
 *   3. Transfer conservation: total pickup = total dropoff at each node (constraint 10)
 *   4. Synchronization: arrival time gap ≤ W at each transfer node (constraint 17)
 */
public class Solution {

    private final Instance instance;
    private final List<Route> routes;
    private final List<Transfer> transfers;
    private final double alpha;          // penalty coefficient for distance

    // ──────────────────────────── Constructors ───────────────────────────────

    public Solution(Instance instance, double alpha) {
        this.instance = instance;
        this.alpha = alpha;
        this.routes = new ArrayList<>();
        this.transfers = new ArrayList<>();
    }

    /** Deep copy constructor */
    public Solution(Solution other) {
        this.instance = other.instance;
        this.alpha = other.alpha;
        this.routes = new ArrayList<>();
        for (Route r : other.routes) {
            this.routes.add(new Route(r));
        }
        this.transfers = new ArrayList<>(other.transfers); // shallow copy OK for now
    }

    // ──────────────────────────── Objective Function ─────────────────────────

    /** Objective value: total profit - α × total distance  (eq. 1) */
    public double getObjectiveValue() {
        return getTotalProfit() - alpha * getTotalDistance();
    }

    /** Sum of profits from all served customers across all routes */
    public double getTotalProfit() {
        double profit = 0;
        for (Route r : routes) {
            profit += r.getTotalProfit();
        }
        return profit;
    }

    /** Sum of distances across all routes */
    public double getTotalDistance() {
        double dist = 0;
        for (Route r : routes) {
            dist += r.getTotalDistance();
        }
        return dist;
    }

    // ──────────────────────────── Feasibility ────────────────────────────────

    /**
     * Complete feasibility check (all constraints).
     * Returns a FeasibilityReport with details on any violations.
     */
    public FeasibilityReport checkFeasibility() {
        FeasibilityReport report = new FeasibilityReport();

        // 1. Route-level feasibility (time + capacity)
        for (Route r : routes) {
            r.evaluate();
            if (!r.isFeasible()) {
                report.addViolation(String.format(
                        "Route v%d infeasible: time=%.1f/%.1f, maxLoad=%.1f/%.1f",
                        r.getVehicleId(), r.getTotalTime(), instance.getMaxRouteDuration(),
                        r.getInitialLoad(), instance.getMaxCapacity()));
            }
        }

        // 2. Each customer served at most once (constraint 5)
        Set<Integer> servedNodes = new HashSet<>();
        for (Route r : routes) {
            for (RouteStop s : r.getStops()) {
                if (s.isServed()) {
                    int nodeId = s.getNode().getId();
                    if (!servedNodes.add(nodeId)) {
                        report.addViolation("Constraint 5 violated: node "
                                + nodeId + " served by multiple vehicles");
                    }
                }
            }
        }

        // 3. Transfer conservation at each node (constraint 10)
        for (Transfer t : transfers) {
            // Conservation is checked per transfer pair
            // (more nuanced check when multiple transfers at same node)
        }
        // Aggregate check: at each node, total pickup qty == total dropoff qty
        checkTransferConservation(report);

        // 4. Synchronization window (constraint 17)
        for (Transfer t : transfers) {
            double giverTime = getArrivalTimeAtNode(t.getGivingVehicleId(), t.getTransferNodeId());
            double receiverTime = getArrivalTimeAtNode(t.getReceivingVehicleId(), t.getTransferNodeId());
            double gap = Math.abs(giverTime - receiverTime);

            if (gap > instance.getSyncWindow() + 1e-6) {
                report.addViolation(String.format(
                        "Sync violated at node %d: giver(v%d)=%.1f, receiver(v%d)=%.1f, gap=%.1f > W=%.1f",
                        t.getTransferNodeId(), t.getGivingVehicleId(), giverTime,
                        t.getReceivingVehicleId(), receiverTime, gap, instance.getSyncWindow()));
            }
        }

        return report;
    }

    /** Shorthand: is the entire solution feasible? */
    public boolean isFeasible() {
        return checkFeasibility().isFeasible();
    }

    /** Check constraint (10): at each node, Σ qpick = Σ qdrop */
    private void checkTransferConservation(FeasibilityReport report) {
        // Collect all pickup and dropoff quantities per node
        java.util.Map<Integer, Double> totalPickup = new java.util.HashMap<>();
        java.util.Map<Integer, Double> totalDropoff = new java.util.HashMap<>();

        for (Route r : routes) {
            for (RouteStop s : r.getStops()) {
                int nid = s.getNode().getId();
                if (s.isPickup()) {
                    totalPickup.merge(nid, s.getPickupQty(), Double::sum);
                }
                if (s.isDropoff()) {
                    totalDropoff.merge(nid, s.getDropoffQty(), Double::sum);
                }
            }
        }

        // Check conservation
        Set<Integer> allTransferNodes = new HashSet<>();
        allTransferNodes.addAll(totalPickup.keySet());
        allTransferNodes.addAll(totalDropoff.keySet());

        for (int nid : allTransferNodes) {
            double pick = totalPickup.getOrDefault(nid, 0.0);
            double drop = totalDropoff.getOrDefault(nid, 0.0);
            if (Math.abs(pick - drop) > 1e-6) {
                report.addViolation(String.format(
                        "Constraint 10 violated at node %d: total_pickup=%.1f ≠ total_dropoff=%.1f",
                        nid, pick, drop));
            }
        }
    }

    // ──────────────────────────── Queries ────────────────────────────────────

    /** Get arrival time of a vehicle at a specific node */
    private double getArrivalTimeAtNode(int vehicleId, int nodeId) {
        Route route = getRouteByVehicleId(vehicleId);
        return route.getArrivalTimeAtNode(nodeId);
    }

    /** Find route by vehicle ID */
    public Route getRouteByVehicleId(int vehicleId) {
        for (Route r : routes) {
            if (r.getVehicleId() == vehicleId) return r;
        }
        throw new IllegalArgumentException("Vehicle " + vehicleId + " not found in solution");
    }

    /** Set of all customer node IDs currently being served */
    public Set<Integer> getServedNodeIds() {
        Set<Integer> served = new HashSet<>();
        for (Route r : routes) {
            for (RouteStop s : r.getStops()) {
                if (s.isServed()) served.add(s.getNode().getId());
            }
        }
        return served;
    }

    /** List of all customer nodes NOT yet served */
    public List<Node> getUnservedNodes() {
        Set<Integer> served = getServedNodeIds();
        List<Node> unserved = new ArrayList<>();
        for (Node n : instance.getNodes()) {
            if (!n.isDepot() && !served.contains(n.getId())) {
                unserved.add(n);
            }
        }
        return unserved;
    }

    /** Total number of customers served */
    public int getNumServed() {
        return getServedNodeIds().size();
    }

    /** Total number of customers in the instance (excluding depot) */
    public int getNumCustomers() {
        return (int) instance.getNodes().stream().filter(n -> !n.isDepot()).count();
    }

    // ──────────────────────────── Modification ───────────────────────────────

    public void addRoute(Route r)        { routes.add(r); }
    public void addTransfer(Transfer t)  { transfers.add(t); }
    public void removeTransfer(Transfer t) { transfers.remove(t); }

    public List<Route> getRoutes()       { return routes; }
    public List<Transfer> getTransfers() { return transfers; }
    public Instance getInstance()        { return instance; }
    public double getAlpha()             { return alpha; }

    // ──────────────────────────── Display ────────────────────────────────────

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("╔══════════════════════════════════════════╗\n");
        sb.append("║           CTOP-T-Sync Solution           ║\n");
        sb.append("╠══════════════════════════════════════════╣\n");

        for (Route r : routes) {
            sb.append("  ").append(r).append("\n");
        }

        if (!transfers.isEmpty()) {
            sb.append("  ── Transfers ──\n");
            for (Transfer t : transfers) {
                sb.append("  ").append(t).append("\n");
            }
        }

        sb.append("╠══════════════════════════════════════════╣\n");
        sb.append(String.format("  Served: %d / %d customers\n", getNumServed(), getNumCustomers()));
        sb.append(String.format("  Profit: %.2f\n", getTotalProfit()));
        sb.append(String.format("  Distance: %.2f\n", getTotalDistance()));
        sb.append(String.format("  Objective (α=%.3f): %.2f\n", alpha, getObjectiveValue()));
        sb.append(String.format("  Transfers: %d\n", transfers.size()));
        sb.append(String.format("  Feasible: %s\n", isFeasible()));
        sb.append("╚══════════════════════════════════════════╝");

        return sb.toString();
    }

    /** Detailed printout including per-route arc loads */
    public String toDetailedString() {
        StringBuilder sb = new StringBuilder(toString());
        sb.append("\n\n── Detailed Route Info ──\n");
        for (Route r : routes) {
            sb.append(r.toDetailedString()).append("\n\n");
        }

        FeasibilityReport report = checkFeasibility();
        if (!report.isFeasible()) {
            sb.append("── VIOLATIONS ──\n");
            for (String v : report.getViolations()) {
                sb.append("  ✗ ").append(v).append("\n");
            }
        }

        return sb.toString();
    }

    // ──────────────────────────── Inner Class ────────────────────────────────

    /** Collects feasibility violation messages for debugging */
    public static class FeasibilityReport {
        private final List<String> violations = new ArrayList<>();

        public void addViolation(String msg) { violations.add(msg); }
        public boolean isFeasible()          { return violations.isEmpty(); }
        public List<String> getViolations()  { return violations; }

        @Override
        public String toString() {
            if (isFeasible()) return "FEASIBLE";
            return "INFEASIBLE (" + violations.size() + " violations):\n  "
                    + String.join("\n  ", violations);
        }
    }
}