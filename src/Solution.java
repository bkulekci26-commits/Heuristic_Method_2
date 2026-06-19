import java.util.*;

/**
 * Represents a complete solution to the CTOP-T-Sync problem.
 */
public class Solution {

    private final Instance instance;
    private final List<Route> routes;
    private final List<Transfer> transfers;

    public Solution(Instance instance) {
        this.instance = instance;
        this.routes = new ArrayList<>();
        this.transfers = new ArrayList<>();
    }

    public Solution(Solution other) {
        this.instance = other.instance;
        this.routes = new ArrayList<>();
        for (Route r : other.routes) {
            this.routes.add(new Route(r));
        }
        this.transfers = new ArrayList<>(other.transfers);
    }

    public void restoreFrom(Solution other) {
        this.routes.clear();
        for (Route r : other.routes) {
            this.routes.add(new Route(r));
        }
        this.transfers.clear();
        this.transfers.addAll(other.transfers);
    }

    // ──────────────────────────── Objective Function ─────────────────────────

    /** * Calculates the ALNS objective value.
     * Enforces "All-or-Nothing" constraint: heavily penalizes partial service.
     */
    /** Objective value = total collected profit minus penalties for partial service */
    public double getObjectiveValue() {
        double objective = getTotalProfit();

        // Add "All-or-Nothing" penalty for split deliveries that don't meet full demand
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
            // If the demand is only partially fulfilled, apply a severe penalty
            if (Math.abs(delivered - demand) > 1e-6) {
                objective -= 10000.0 * Math.abs(demand - delivered);
            }
        }
        return objective;
    }

    /**
     * Safety Net for ALNS: Sweeps through the solution and completely removes
     * any customer whose demand was only partially fulfilled. Restores mathematical feasibility.
     */
    public void cleanupFailedSplits() {
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

        Set<Integer> brokenCustomers = new HashSet<>();
        for (Map.Entry<Integer, Double> e : totalDelivery.entrySet()) {
            double delivered = e.getValue();
            double demand = customerDemand.get(e.getKey());
            // If strictly between 0 and full demand, it is a broken split
            if (delivered > 1e-6 && Math.abs(delivered - demand) > 1e-6) {
                brokenCustomers.add(e.getKey());
            }
        }

        if (!brokenCustomers.isEmpty()) {
            for (Route r : routes) {
                boolean routeChanged = false;
                for (int i = r.getStops().size() - 1; i >= 0; i--) {
                    RouteStop stop = r.getStops().get(i);
                    if (stop.isServed() && brokenCustomers.contains(stop.getNode().getId())) {
                        r.removeStop(i);
                        routeChanged = true;
                    }
                }
                if (routeChanged) r.evaluate();
            }
        }
    }

    /** * Returns the true profit of the solution (without mathematical penalties).
     * Only counts customers whose demand is FULLY met.
     */
    public double getTotalProfit() {
        Map<Integer, Double> totalDelivery = new HashMap<>();
        Map<Integer, Double> customerDemand = new HashMap<>();
        Map<Integer, Double> customerProfit = new HashMap<>();

        for (Route r : routes) {
            for (RouteStop s : r.getStops()) {
                if (s.isServed()) {
                    int nid = s.getNode().getId();
                    totalDelivery.merge(nid, s.getDeliveryQty(), Double::sum);
                    customerDemand.putIfAbsent(nid, s.getNode().getDemand());
                    customerProfit.putIfAbsent(nid, s.getNode().getProfit());
                }
            }
        }

        double profit = 0;
        for (Map.Entry<Integer, Double> e : totalDelivery.entrySet()) {
            if (Math.abs(e.getValue() - customerDemand.get(e.getKey())) < 1e-6) {
                profit += customerProfit.get(e.getKey());
            }
        }
        return profit;
    }

    public double getTotalDistance() {
        double dist = 0;
        for (Route r : routes) {
            dist += r.getTotalDistance();
        }
        return dist;
    }

    // ──────────────────────────── Feasibility ────────────────────────────────

    public FeasibilityReport checkFeasibility() {
        FeasibilityReport report = new FeasibilityReport();

        for (Route r : routes) {
            r.evaluate();
            if (!r.isFeasible()) {
                report.addViolation(String.format(
                        "Route v%d infeasible: time=%.1f/%.1f, maxLoad=%.1f/%.1f",
                        r.getVehicleId(), r.getTotalTime(), instance.getMaxRouteDuration(),
                        r.getInitialLoad(), instance.getMaxCapacity()));
            }
        }

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

        checkTransferConservation(report);

        for (Transfer t : transfers) {
            try {
                Route giverRoute = getRouteByVehicleId(t.getGivingVehicleId());
                Route receiverRoute = getRouteByVehicleId(t.getReceivingVehicleId());

                if (!giverRoute.visitsNode(t.getTransferNodeId())) {
                    report.addViolation(String.format("Stale transfer: giver v%d no longer visits node %d", t.getGivingVehicleId(), t.getTransferNodeId()));
                    continue;
                }
                if (!receiverRoute.visitsNode(t.getTransferNodeId())) {
                    report.addViolation(String.format("Stale transfer: receiver v%d no longer visits node %d", t.getReceivingVehicleId(), t.getTransferNodeId()));
                    continue;
                }

                double giverTime = giverRoute.getArrivalTimeAtNode(t.getTransferNodeId());
                double receiverTime = receiverRoute.getArrivalTimeAtNode(t.getTransferNodeId());

                double syncGap = giverTime - receiverTime;

                if (syncGap > instance.getSyncWindow() + 1e-6) {
                    report.addViolation(String.format(
                            "Sync violated at node %d: giver(v%d)=%.1f arrives %.1f after receiver(v%d)=%.1f, exceeds W=%.1f",
                            t.getTransferNodeId(), t.getGivingVehicleId(), giverTime,
                            syncGap, t.getReceivingVehicleId(), receiverTime, instance.getSyncWindow()));
                }
            } catch (Exception e) {
                report.addViolation("Transfer check error at node " + t.getTransferNodeId() + ": " + e.getMessage());
            }
        }

        return report;
    }

    public boolean isFeasible() {
        return checkFeasibility().isFeasible();
    }

    public FeasibilityReport checkFeasibilitySD() {
        FeasibilityReport report = new FeasibilityReport();

        for (Route r : routes) {
            r.evaluate();
            if (!r.isFeasible()) {
                report.addViolation(String.format(
                        "Route v%d infeasible: time=%.1f/%.1f, maxLoad=%.1f/%.1f",
                        r.getVehicleId(), r.getTotalTime(), instance.getMaxRouteDuration(),
                        r.getInitialLoad(), instance.getMaxCapacity()));
            }
        }

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

    public Map<Integer, List<int[]>> getSplitCustomers() {
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

        Map<Integer, List<int[]>> splits = new HashMap<>();
        for (Map.Entry<Integer, List<int[]>> e : visitMap.entrySet()) {
            if (e.getValue().size() > 1) {
                splits.put(e.getKey(), e.getValue());
            }
        }
        return splits;
    }

    public double getResidualCapacity(Route route, int stopIndex) {
        route.evaluate();
        double loadBeforeStop = route.getArcLoad(stopIndex);
        return instance.getMaxCapacity() - loadBeforeStop;
    }

    // ──────────────────────────── Standard Queries ───────────────────────────

    public Route getRouteByVehicleId(int vehicleId) {
        for (Route r : routes) {
            if (r.getVehicleId() == vehicleId) return r;
        }
        throw new IllegalArgumentException("Vehicle " + vehicleId + " not found");
    }

    public Set<Integer> getServedNodeIds() {
        Set<Integer> served = new HashSet<>();
        for (Route r : routes) {
            for (RouteStop s : r.getStops()) {
                if (s.isServed()) served.add(s.getNode().getId());
            }
        }
        return served;
    }

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


    /**
     * Sweeps through the solution and completely removes any customer
     * whose demand was only partially fulfilled. Restores feasibility.
     */
    public void cleanupPartialDeliveries() {
        // 1. Calculate total deliveries
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

        // 2. Identify broken customers
        Set<Integer> brokenCustomers = new HashSet<>();
        for (Map.Entry<Integer, Double> e : totalDelivery.entrySet()) {
            double delivered = e.getValue();
            double demand = customerDemand.get(e.getKey());

            // If delivered is greater than 0 but less than full demand
            if (delivered > 1e-6 && Math.abs(delivered - demand) > 1e-6) {
                brokenCustomers.add(e.getKey());
            }
        }

        // 3. Remove all stops belonging to broken customers
        if (!brokenCustomers.isEmpty()) {
            for (Route r : routes) {
                boolean routeChanged = false;
                for (int i = r.getStops().size() - 1; i >= 0; i--) {
                    RouteStop stop = r.getStops().get(i);
                    if (stop.isServed() && brokenCustomers.contains(stop.getNode().getId())) {
                        r.removeStop(i);
                        routeChanged = true;
                    }
                }
                if (routeChanged) {
                    r.evaluate();
                }
            }
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