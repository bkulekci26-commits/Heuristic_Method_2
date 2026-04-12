import java.util.*;

/**
 * Represents a complete solution to the CTOP-T-Sync problem.
 *
 * Supports two modes:
 *   1. CTOP mode: each customer served by at most one vehicle
 *   2. SD-CTOP mode: split deliveries allowed (a customer can be served
 *      by multiple vehicles with partial demand). The Transform operator
 *      converts SD-CTOP solutions to CTOP-T-Sync solutions with transfers.
 *
 * Objective function:
 *   max Z = Σ pj · zjk   (maximize total collected profit)
 *   Note: profit is collected ONCE per customer, even if served by multiple routes.
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

    /** Restore this solution's state from another (rollback support) */
    public void restoreFrom(Solution other) {
        this.routes.clear();
        for (Route r : other.routes) {
            this.routes.add(new Route(r));
        }
        this.transfers.clear();
        this.transfers.addAll(other.transfers);
    }

    // ──────────────────────────── Objective Function ─────────────────────────

    /** Objective value = total collected profit (each customer counted once) */
    public double getObjectiveValue() {
        return getTotalProfit();
    }

    /**
     * Total profit from served customers.
     * Each customer's profit counted ONCE even if split across multiple routes.
     */
    public double getTotalProfit() {
        Set<Integer> counted = new HashSet<>();
        double profit = 0;
        for (Route r : routes) {
            for (RouteStop s : r.getStops()) {
                if (s.isServed() && counted.add(s.getNode().getId())) {
                    profit += s.getNode().getProfit();
                }
            }
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
     * Complete feasibility check for CTOP-T-Sync mode (after Transform).
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

        // 2. Each customer served at most once (constraint 5) — standard CTOP mode
        //    In SD-CTOP mode, multiple routes can serve same customer (split delivery)
        Set<Integer> servedNodes = new HashSet<>();
        for (Route r : routes) {
            for (RouteStop s : r.getStops()) {
                if (s.isServed()) {
                    int nodeId = s.getNode().getId();
                    if (!servedNodes.add(nodeId)) {
                        // Check if this is a split delivery (allowed in SD mode)
                        // In final CTOP-T-Sync mode, this would be a violation
                        report.addViolation("Constraint 5 violated: node "
                                + nodeId + " served by multiple vehicles");
                    }
                }
            }
        }

        // 3. Transfer conservation at each node (constraint 10)
        checkTransferConservation(report);

        // 4. Synchronization window (constraint 17)
        for (Transfer t : transfers) {
            try {
                Route giverRoute = getRouteByVehicleId(t.getGivingVehicleId());
                Route receiverRoute = getRouteByVehicleId(t.getReceivingVehicleId());

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

                // DIRECTIONAL sync: giver(dropper) - receiver(picker) ≤ W
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

    /**
     * Feasibility check allowing split deliveries (for SD-CTOP intermediate solutions).
     * Same as checkFeasibility() but does NOT flag duplicate service as violation.
     * Instead checks that total delivery across routes matches demand.
     */
    public FeasibilityReport checkFeasibilitySD() {
        FeasibilityReport report = new FeasibilityReport();

        // 1. Route-level feasibility
        for (Route r : routes) {
            r.evaluate();
            if (!r.isFeasible()) {
                report.addViolation(String.format(
                        "Route v%d infeasible: time=%.1f/%.1f, maxLoad=%.1f/%.1f",
                        r.getVehicleId(), r.getTotalTime(), instance.getMaxRouteDuration(),
                        r.getInitialLoad(), instance.getMaxCapacity()));
            }
        }

        // 2. Split delivery demand conservation: total delivery = demand for each served customer
        Map<Integer, Double> totalDelivery = new HashMap<>();
        Map<Integer, Double> customerDemand = new HashMap<>();
        for (Route r : routes) {
            for (RouteStop s : r.getStops()) {
                if (s.isServed()) {
                    int nid = s.getNode().getId();
                    totalDelivery.merge(nid, s.getDeliveryQty(), Double::sum);
                    customerDemand.put(nid, s.getNode().getDemand());
                }
            }
        }
        for (Map.Entry<Integer, Double> e : totalDelivery.entrySet()) {
            double delivered = e.getValue();
            double demand = customerDemand.get(e.getKey());
            if (Math.abs(delivered - demand) > 1e-6) {
                report.addViolation(String.format(
                        "SD demand mismatch at node %d: delivered=%.1f ≠ demand=%.1f",
                        e.getKey(), delivered, demand));
            }
        }

        return report;
    }

    /** Check constraint (10): at each node, Σ qpick = Σ qdrop */
    private void checkTransferConservation(FeasibilityReport report) {
        Map<Integer, Double> totalPickup = new HashMap<>();
        Map<Integer, Double> totalDropoff = new HashMap<>();

        for (Route r : routes) {
            for (RouteStop s : r.getStops()) {
                int nid = s.getNode().getId();
                if (s.isPickup()) totalPickup.merge(nid, s.getPickupQty(), Double::sum);
                if (s.isDropoff()) totalDropoff.merge(nid, s.getDropoffQty(), Double::sum);
            }
        }

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

    // ──────────────────────────── Split Delivery Queries ─────────────────────

    /**
     * Returns customers currently served by multiple routes (split deliveries).
     * Each entry: nodeId → list of (routeIndex, deliveryQty).
     */
    public Map<Integer, List<int[]>> getSplitCustomers() {
        // Map: nodeId → list of [routeIndex, vehicleId]
        Map<Integer, List<int[]>> visitMap = new HashMap<>();

        for (int ri = 0; ri < routes.size(); ri++) {
            Route r = routes.get(ri);
            for (RouteStop s : r.getStops()) {
                if (s.isServed()) {
                    int nid = s.getNode().getId();
                    visitMap.computeIfAbsent(nid, k -> new ArrayList<>())
                            .add(new int[]{ri, r.getVehicleId()});
                }
            }
        }

        // Filter to only those served by 2+ routes
        Map<Integer, List<int[]>> splits = new HashMap<>();
        for (Map.Entry<Integer, List<int[]>> e : visitMap.entrySet()) {
            if (e.getValue().size() > 1) {
                splits.put(e.getKey(), e.getValue());
            }
        }
        return splits;
    }

    /**
     * Get the residual capacity of a route at a specific stop position.
     * This is the remaining capacity the vehicle has when it arrives at that position.
     * z_jk in Aguayo's notation.
     */
    public double getResidualCapacity(Route route, int stopIndex) {
        route.evaluate();
        // Arc load before the stop = load on arc entering this stop
        double loadBeforeStop = route.getArcLoad(stopIndex);
        // Residual = Q - load before stop + what this stop consumes (because consumption hasn't happened yet)
        // Actually, arcLoad[i] = load on arc (stop[i-1] → stop[i]) for i>=1, arcLoad[0] = load on arc (depot → stop[0])
        // The load entering stop[stopIndex] is arcLoad[stopIndex]
        // Residual capacity at this point = Q - arcLoad[stopIndex]
        return instance.getMaxCapacity() - loadBeforeStop;
    }

    // ──────────────────────────── Standard Queries ───────────────────────────

    public Route getRouteByVehicleId(int vehicleId) {
        for (Route r : routes) {
            if (r.getVehicleId() == vehicleId) return r;
        }
        throw new IllegalArgumentException("Vehicle " + vehicleId + " not found");
    }

    /** Set of all customer node IDs currently being served (across all routes) */
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

    public int getNumServed() { return getServedNodeIds().size(); }
    public int getNumCustomers() {
        return (int) instance.getNodes().stream().filter(n -> !n.isDepot()).count();
    }

    // ──────────────────────────── Modification ───────────────────────────────

    public void addRoute(Route r)          { routes.add(r); }
    public void addTransfer(Transfer t)    { transfers.add(t); }
    public void removeTransfer(Transfer t) { transfers.remove(t); }
    public void clearTransfers()           { transfers.clear(); }

    /**
     * Removes any Transfer objects that reference nodes no longer present
     * in the corresponding vehicle's route. Also reverts orphaned stops.
     */
    public void cleanupStaleTransfers() {
        Iterator<Transfer> it = transfers.iterator();
        Set<Integer> validTransferNodes = new HashSet<>();

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

        for (Route r : routes) {
            boolean changed = false;
            for (int i = r.getStops().size() - 1; i >= 0; i--) {
                RouteStop stop = r.getStops().get(i);
                int nodeId = stop.getNode().getId();

                if (stop.isTransferOnly() && !validTransferNodes.contains(nodeId)) {
                    r.removeStop(i);
                    changed = true;
                } else if (stop.isServed() && (stop.isPickup() || stop.isDropoff())
                        && !validTransferNodes.contains(nodeId)) {
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

        Map<Integer, List<int[]>> splits = getSplitCustomers();
        if (!splits.isEmpty()) {
            sb.append("  ── Split Deliveries ──\n");
            for (Map.Entry<Integer, List<int[]>> e : splits.entrySet()) {
                sb.append(String.format("  Node %d split across vehicles: ", e.getKey()));
                for (int[] rv : e.getValue()) {
                    sb.append(String.format("v%d ", rv[1]));
                }
                sb.append("\n");
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