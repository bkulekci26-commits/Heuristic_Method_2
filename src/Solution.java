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
 * Objective function:
 *   max Z = Σ pj·zjk   (maximize total collected profit)
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

    // ──────────────────────────── Constructors ───────────────────────────────

    public Solution(Instance instance) {
        this.instance = instance;
        this.routes = new ArrayList<>();
        this.transfers = new ArrayList<>();
    }

    /** Deep copy constructor */
    public Solution(Solution other) {
        this.instance = other.instance;
        this.routes = new ArrayList<>();
        for (Route r : other.routes) {
            this.routes.add(new Route(r));
        }
        this.transfers = new ArrayList<>(other.transfers);
    }

    /** Restore this solution's state from another solution (for rollback) */
    public void restoreFrom(Solution other) {
        this.routes.clear();
        for (Route r : other.routes) {
            this.routes.add(new Route(r));
        }
        this.transfers.clear();
        this.transfers.addAll(other.transfers);
    }

    // ──────────────────────────── Objective Function ─────────────────────────

    /** Objective value = total collected profit */
    public double getObjectiveValue() {
        return getTotalProfit();
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
            try {
                Route giverRoute = getRouteByVehicleId(t.getGivingVehicleId());
                Route receiverRoute = getRouteByVehicleId(t.getReceivingVehicleId());

                // Check that both vehicles still visit the transfer node
                if (!giverRoute.visitsNode(t.getTransferNodeId())) {
                    report.addViolation(String.format(
                            "Stale transfer: giver v%d no longer visits node %d",
                            t.getGivingVehicleId(), t.getTransferNodeId()));
                    continue;
                }
                if (!receiverRoute.visitsNode(t.getTransferNodeId())) {
                    report.addViolation(String.format(
                            "Stale transfer: receiver v%d no longer visits node %d",
                            t.getReceivingVehicleId(), t.getTransferNodeId()));
                    continue;
                }

                double giverTime = giverRoute.getArrivalTimeAtNode(t.getTransferNodeId());
                double receiverTime = receiverRoute.getArrivalTimeAtNode(t.getTransferNodeId());

                // Constraint (17): u_j,k1 - u_j,k2 ≤ W
                // DIRECTIONAL: giver can arrive before receiver (goods wait at node),
                // but receiver should not wait more than W for the giver to arrive.
                // giverTime - receiverTime ≤ W
                double syncGap = giverTime - receiverTime;

                if (syncGap > instance.getSyncWindow() + 1e-6) {
                    report.addViolation(String.format(
                            "Sync violated at node %d: giver(v%d)=%.1f arrives %.1f after receiver(v%d)=%.1f, exceeds W=%.1f",
                            t.getTransferNodeId(), t.getGivingVehicleId(), giverTime,
                            syncGap, t.getReceivingVehicleId(), receiverTime, instance.getSyncWindow()));
                }
            } catch (Exception e) {
                report.addViolation("Transfer check error at node " + t.getTransferNodeId()
                        + ": " + e.getMessage());
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

    /**
     * Removes any Transfer objects that reference nodes no longer present
     * in the corresponding vehicle's route. Also reverts any orphaned
     * pickup/dropoff stops to serve-only (or removes them if transfer-only).
     *
     * Must be called after any destroy/repair operation that may have
     * altered routes containing transfer nodes.
     */
    public void cleanupStaleTransfers() {
        // 1. Remove Transfer objects where giver or receiver no longer visits the node
        java.util.Iterator<Transfer> it = transfers.iterator();
        java.util.Set<Integer> validTransferNodes = new java.util.HashSet<>();

        while (it.hasNext()) {
            Transfer t = it.next();
            Route giverRoute = getRouteByVehicleId(t.getGivingVehicleId());
            Route receiverRoute = getRouteByVehicleId(t.getReceivingVehicleId());

            boolean giverOk = giverRoute.visitsNode(t.getTransferNodeId());
            boolean receiverOk = receiverRoute.visitsNode(t.getTransferNodeId());

            if (giverOk && receiverOk) {
                validTransferNodes.add(t.getTransferNodeId());
            } else {
                it.remove();
            }
        }

        // 2. Clean up orphaned pickup/dropoff stops in all routes
        for (Route r : routes) {
            boolean changed = false;
            for (int i = r.getStops().size() - 1; i >= 0; i--) {
                RouteStop stop = r.getStops().get(i);
                int nodeId = stop.getNode().getId();

                if (stop.isTransferOnly() && !validTransferNodes.contains(nodeId)) {
                    // Transfer-only stop with no valid transfer → remove
                    r.removeStop(i);
                    changed = true;
                } else if (stop.isServed() && (stop.isPickup() || stop.isDropoff())
                        && !validTransferNodes.contains(nodeId)) {
                    // Served + transfer action, but transfer is gone → revert to serve-only
                    r.getStops().set(i, RouteStop.serve(stop.getNode()));
                    changed = true;
                }
            }
            if (changed) r.evaluate();
        }
    }

    public List<Route> getRoutes()       { return routes; }
    public List<Transfer> getTransfers() { return transfers; }
    public Instance getInstance()        { return instance; }

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