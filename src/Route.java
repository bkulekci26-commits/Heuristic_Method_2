import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Represents a single vehicle's route in the CTOP-T-Sync problem.
 */
public class Route {

    private final int vehicleId;
    private final List<RouteStop> stops;
    private final Instance instance;

    private double[] arrivalTimes;
    private double[] arcLoads;
    private double totalTime;
    private double totalDistance;
    private double totalProfit;
    private double initialLoad;
    private boolean evaluated;

    public Route(int vehicleId, Instance instance) {
        this.vehicleId = vehicleId;
        this.instance = instance;
        this.stops = new ArrayList<>();
        this.evaluated = false;
    }

    /** Deep copy constructor */
    public Route(Route other) {
        this.vehicleId = other.vehicleId;
        this.instance = other.instance;
        this.stops = new ArrayList<>();
        for (RouteStop s : other.stops) {
            RouteStop newStop = new RouteStop(
                    s.getNode(), s.isServed(), s.isPickup(), s.isDropoff(),
                    s.getPickupQty(), s.getDropoffQty(), s.getDeliveryQty());
            newStop.setWaitingTime(s.getWaitingTime()); // Copy waiting time
            this.stops.add(newStop);
        }
        this.evaluated = false;
    }

    public void evaluate() {
        int n = stops.size();
        Node depot = instance.getDepot();

        arrivalTimes = new double[n + 2];
        arcLoads = new double[n + 1];

        initialLoad = 0;
        for (RouteStop s : stops) {
            initialLoad += s.getLoadConsumption();
        }

        arrivalTimes[0] = 0.0;
        arcLoads[0] = initialLoad;

        double load = initialLoad;
        double dist = 0.0;
        double profit = 0.0;
        double currentDepartureTime = 0.0; // Tracks delays caused by waitingTime
        Node prev = depot;

        for (int i = 0; i < n; i++) {
            RouteStop s = stops.get(i);
            Node curr = s.getNode();

            // 1. Travel to this stop
            double travel = instance.getDistance(prev, curr);
            dist += travel;

            // 2. Arrival is based on departure from previous node
            arrivalTimes[i + 1] = currentDepartureTime + travel;

            // 3. Process load
            load -= s.getLoadConsumption();
            if (i < n) {
                arcLoads[i + 1] = load;
            }

            // 4. Base profit (Checked strictly in Solution.java)
            if (s.isServed()) {
                profit += s.getNode().getProfit();
            }

            // 5. Update departure time (arrival + waiting time for sync)
            currentDepartureTime = arrivalTimes[i + 1] + s.getWaitingTime();
            prev = curr;
        }

        double returnTravel = instance.getDistance(prev, depot);
        dist += returnTravel;
        arrivalTimes[n + 1] = currentDepartureTime + returnTravel;

        if (n == 0) {
            arrivalTimes[1] = 0.0;
        }

        totalTime = arrivalTimes[n + 1];
        totalDistance = dist;
        totalProfit = profit;
        evaluated = true;
    }

    public boolean isFeasible() {
        if (!evaluated) evaluate();
        if (totalTime > instance.getMaxRouteDuration() + 1e-6) return false;
        for (double load : arcLoads) {
            if (load < -1e-6 || load > instance.getMaxCapacity() + 1e-6) return false;
        }
        return true;
    }

    public boolean canInsert(int position, RouteStop newStop) {
        stops.add(position, newStop);
        evaluate();
        boolean feasible = isFeasible();
        stops.remove(position);
        evaluated = false;
        return feasible;
    }

    public void insertStop(int position, RouteStop stop) {
        stops.add(position, stop);
        evaluated = false;
    }

    public RouteStop removeStop(int position) {
        RouteStop removed = stops.remove(position);
        evaluated = false;
        return removed;
    }

    public boolean removeStopByNodeId(int nodeId) {
        for (int i = 0; i < stops.size(); i++) {
            if (stops.get(i).getNode().getId() == nodeId) {
                stops.remove(i);
                evaluated = false;
                return true;
            }
        }
        return false;
    }

    public double getArrivalTime(int fullSequenceIndex) {
        if (!evaluated) evaluate();
        return arrivalTimes[fullSequenceIndex];
    }

    public double getArrivalTimeAtNode(int nodeId) {
        if (!evaluated) evaluate();
        for (int i = 0; i < stops.size(); i++) {
            if (stops.get(i).getNode().getId() == nodeId) {
                return arrivalTimes[i + 1];
            }
        }
        throw new IllegalStateException("Node " + nodeId + " not found in route v" + vehicleId);
    }

    public double getArcLoad(int arcIndex) {
        if (!evaluated) evaluate();
        return arcLoads[arcIndex];
    }

    public double getRemainingCapacity() {
        if (!evaluated) evaluate();
        double maxLoad = 0;
        for (double load : arcLoads) {
            maxLoad = Math.max(maxLoad, load);
        }
        return instance.getMaxCapacity() - maxLoad;
    }

    public double getRemainingTime() {
        if (!evaluated) evaluate();
        return instance.getMaxRouteDuration() - totalTime;
    }

    public double getTotalDemandServed() {
        return stops.stream().filter(RouteStop::isServed).mapToDouble(s -> s.getNode().getDemand()).sum();
    }

    public boolean visitsNode(int nodeId) {
        return stops.stream().anyMatch(s -> s.getNode().getId() == nodeId);
    }

    public boolean servesNode(int nodeId) {
        return stops.stream().anyMatch(s -> s.getNode().getId() == nodeId && s.isServed());
    }

    public int findStopIndex(int nodeId) {
        for (int i = 0; i < stops.size(); i++) {
            if (stops.get(i).getNode().getId() == nodeId) return i;
        }
        return -1;
    }

    public int getVehicleId()               { return vehicleId; }
    public List<RouteStop> getStops()       { return stops; }
    public int size()                       { return stops.size(); }
    public boolean isEmpty()                { return stops.isEmpty(); }
    public Instance getInstance()           { return instance; }

    public double getTotalTime() {
        if (!evaluated) evaluate();
        return totalTime;
    }

    public double getTotalDistance() {
        if (!evaluated) evaluate();
        return totalDistance;
    }

    public double getTotalProfit() {
        if (!evaluated) evaluate();
        return totalProfit;
    }

    public double getInitialLoad() {
        if (!evaluated) evaluate();
        return initialLoad;
    }

    @Override
    public String toString() {
        if (!evaluated && !stops.isEmpty()) evaluate();
        StringBuilder sb = new StringBuilder();
        sb.append(String.format("Route[v%d]: 0", vehicleId));
        for (RouteStop s : stops) {
            sb.append(" → ").append(s);
        }
        sb.append(" → 0");
        if (evaluated) {
            sb.append(String.format("  |  profit=%.1f  dist=%.1f  time=%.1f  load0=%.1f",
                    totalProfit, totalDistance, totalTime, initialLoad));
        }
        return sb.toString();
    }

    public String toDetailedString() {
        if (!evaluated) evaluate();
        StringBuilder sb = new StringBuilder();
        sb.append(String.format("=== Route Vehicle %d ===\n", vehicleId));
        sb.append(String.format("  depot(t=%.1f, load=%.1f)", arrivalTimes[0], arcLoads[0]));
        for (int i = 0; i < stops.size(); i++) {
            RouteStop s = stops.get(i);
            sb.append(String.format("\n  → %s (t=%.1f, arc_load=%.1f)",
                    s, arrivalTimes[i + 1], (i + 1 < arcLoads.length) ? arcLoads[i + 1] : 0.0));
        }
        sb.append(String.format("\n  → depot(t=%.1f)", arrivalTimes[stops.size() + 1]));
        sb.append(String.format("\n  Feasible: %s | Profit: %.1f | Dist: %.1f | Time: %.1f / %.1f | MaxLoad: %.1f / %.1f",
                isFeasible(), totalProfit, totalDistance, totalTime,
                instance.getMaxRouteDuration(), initialLoad, instance.getMaxCapacity()));
        return sb.toString();
    }
}