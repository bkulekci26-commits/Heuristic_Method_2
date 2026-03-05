import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Represents a single vehicle's route in the CTOP-T-Sync problem.
 *
 * Structure: depot → stop[0] → stop[1] → ... → stop[n-1] → depot
 * The depot is NOT stored in the stops list; it is handled implicitly.
 *
 * Load model (delivery problem):
 *   - Vehicle departs depot carrying all goods it will deliver + drop off
 *   - At each stop, load changes according to RouteStop.getLoadConsumption()
 *   - Vehicle returns to depot with zero load
 *
 * Arc loads:
 *   arcLoads[0] = initial load on arc (depot → stop[0])
 *   arcLoads[i] = load on arc (stop[i-1] → stop[i])        for i ∈ [1, n-1]
 *   arcLoads[n] = load on arc (stop[n-1] → depot)
 *
 * Arrival times:
 *   arrivalTimes[0] = 0.0  (departure from depot)
 *   arrivalTimes[i] = arrival time at stop[i-1]             for i ∈ [1, n]
 *   arrivalTimes[n+1] = arrival time back at depot (= total route duration)
 */
public class Route {

    private final int vehicleId;
    private final List<RouteStop> stops;   // ordered customer stops (no depot)
    private final Instance instance;

    // ── Cached evaluation results (valid after evaluate()) ──
    private double[] arrivalTimes;   // size = stops.size() + 2
    private double[] arcLoads;       // size = stops.size() + 1
    private double totalTime;
    private double totalDistance;
    private double totalProfit;
    private double initialLoad;      // load leaving depot
    private boolean evaluated;

    // ──────────────────────────── Constructor ────────────────────────────────

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
            this.stops.add(new RouteStop(
                    s.getNode(), s.isServed(), s.isPickup(), s.isDropoff(),
                    s.getPickupQty(), s.getDropoffQty()));
        }
        this.evaluated = false;
    }

    // ──────────────────────────── Evaluation ─────────────────────────────────

    /**
     * Recalculates all cached values: arrival times, arc loads, totals.
     * Must be called after any modification to the stops list.
     */
    public void evaluate() {
        int n = stops.size();
        Node depot = instance.getDepot();

        arrivalTimes = new double[n + 2];
        arcLoads = new double[n + 1];

        // Step 1: Compute initial load (total consumption along route)
        initialLoad = 0;
        for (RouteStop s : stops) {
            initialLoad += s.getLoadConsumption();
        }

        // Step 2: Compute arrival times and arc loads
        arrivalTimes[0] = 0.0;       // depart depot at time 0
        arcLoads[0] = initialLoad;    // load on arc depot → stop[0]

        double load = initialLoad;
        double dist = 0.0;
        double profit = 0.0;
        Node prev = depot;

        for (int i = 0; i < n; i++) {
            RouteStop s = stops.get(i);
            Node curr = s.getNode();

            // Travel to this stop
            double travel = instance.getDistance(prev, curr);
            dist += travel;
            arrivalTimes[i + 1] = arrivalTimes[i] + travel;

            // Process stop: update load
            load -= s.getLoadConsumption();

            // Load on next arc
            if (i < n) {
                arcLoads[i + 1] = load;   // handles both middle arcs and last arc to depot
            }

            // Accumulate profit (only from served customers)
            if (s.isServed()) {
                profit += s.getNode().getProfit();
            }

            prev = curr;
        }

        // Return to depot
        double returnTravel = instance.getDistance(prev, depot);
        dist += returnTravel;
        arrivalTimes[n + 1] = arrivalTimes[n] + returnTravel;

        // Handle empty route edge case
        if (n == 0) {
            arrivalTimes[1] = 0.0;
        }

        totalTime = arrivalTimes[n + 1];
        totalDistance = dist;
        totalProfit = profit;
        evaluated = true;
    }

    // ──────────────────────────── Feasibility ────────────────────────────────

    /**
     * Checks route feasibility:
     *   1. Total route duration ≤ Tmax        (constraint 16)
     *   2. Load on every arc ∈ [0, Q]         (constraint 11)
     */
    public boolean isFeasible() {
        if (!evaluated) evaluate();

        // Time constraint
        if (totalTime > instance.getMaxRouteDuration() + 1e-6) return false;

        // Capacity constraint: check every arc
        for (double load : arcLoads) {
            if (load < -1e-6 || load > instance.getMaxCapacity() + 1e-6) return false;
        }

        return true;
    }

    /**
     * Quick feasibility check after hypothetical insertion at given position.
     * Does NOT modify the route. Returns true if insertion would be feasible.
     */
    public boolean canInsert(int position, RouteStop newStop) {
        // Temporarily insert
        stops.add(position, newStop);
        evaluate();
        boolean feasible = isFeasible();
        // Undo
        stops.remove(position);
        evaluated = false;
        return feasible;
    }

    // ──────────────────────────── Modification Operations ────────────────────

    /** Insert a stop at the given position (0-indexed among customer stops) */
    public void insertStop(int position, RouteStop stop) {
        stops.add(position, stop);
        evaluated = false;
    }

    /** Remove and return the stop at given position */
    public RouteStop removeStop(int position) {
        RouteStop removed = stops.remove(position);
        evaluated = false;
        return removed;
    }

    /** Remove a stop by node ID. Returns true if found and removed. */
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

    // ──────────────────────────── Queries ────────────────────────────────────

    /**
     * Returns the arrival time at the given stop index.
     * Index 0 = depot departure, 1..n = stops, n+1 = depot return.
     */
    public double getArrivalTime(int fullSequenceIndex) {
        if (!evaluated) evaluate();
        return arrivalTimes[fullSequenceIndex];
    }

    /** Returns arrival time at a specific node ID. Throws if not found. */
    public double getArrivalTimeAtNode(int nodeId) {
        if (!evaluated) evaluate();
        for (int i = 0; i < stops.size(); i++) {
            if (stops.get(i).getNode().getId() == nodeId) {
                return arrivalTimes[i + 1]; // +1 because index 0 is depot
            }
        }
        throw new IllegalStateException(
                "Node " + nodeId + " not found in route of vehicle " + vehicleId);
    }

    /** Returns the load on the arc leaving the given stop index */
    public double getArcLoad(int arcIndex) {
        if (!evaluated) evaluate();
        return arcLoads[arcIndex];
    }

    /** Remaining capacity: how much more load the vehicle can carry at its heaviest point */
    public double getRemainingCapacity() {
        if (!evaluated) evaluate();
        double maxLoad = 0;
        for (double load : arcLoads) {
            maxLoad = Math.max(maxLoad, load);
        }
        return instance.getMaxCapacity() - maxLoad;
    }

    /** Remaining time budget */
    public double getRemainingTime() {
        if (!evaluated) evaluate();
        return instance.getMaxRouteDuration() - totalTime;
    }

    /** Total demand served on this route */
    public double getTotalDemandServed() {
        return stops.stream()
                .filter(RouteStop::isServed)
                .mapToDouble(s -> s.getNode().getDemand())
                .sum();
    }

    /** Checks if this route visits a specific node (for any purpose) */
    public boolean visitsNode(int nodeId) {
        return stops.stream().anyMatch(s -> s.getNode().getId() == nodeId);
    }

    /** Checks if this route serves (delivers to) a specific node */
    public boolean servesNode(int nodeId) {
        return stops.stream()
                .anyMatch(s -> s.getNode().getId() == nodeId && s.isServed());
    }

    /** Find the index of a stop with the given node ID, or -1 */
    public int findStopIndex(int nodeId) {
        for (int i = 0; i < stops.size(); i++) {
            if (stops.get(i).getNode().getId() == nodeId) return i;
        }
        return -1;
    }

    // ──────────────────────────── Getters ────────────────────────────────────

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

    // ──────────────────────────── Display ────────────────────────────────────

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

    /** Detailed printout with arc-by-arc loads and arrival times */
    public String toDetailedString() {
        if (!evaluated) evaluate();

        StringBuilder sb = new StringBuilder();
        sb.append(String.format("=== Route Vehicle %d ===\n", vehicleId));
        sb.append(String.format("  depot(t=%.1f, load=%.1f)", arrivalTimes[0], arcLoads[0]));

        for (int i = 0; i < stops.size(); i++) {
            RouteStop s = stops.get(i);
            sb.append(String.format("\n  → %s (t=%.1f, arc_load=%.1f)",
                    s, arrivalTimes[i + 1],
                    (i + 1 < arcLoads.length) ? arcLoads[i + 1] : 0.0));
        }

        sb.append(String.format("\n  → depot(t=%.1f)", arrivalTimes[stops.size() + 1]));
        sb.append(String.format("\n  Feasible: %s | Profit: %.1f | Dist: %.1f | Time: %.1f / %.1f | MaxLoad: %.1f / %.1f",
                isFeasible(), totalProfit, totalDistance, totalTime,
                instance.getMaxRouteDuration(), initialLoad, instance.getMaxCapacity()));

        return sb.toString();
    }
}