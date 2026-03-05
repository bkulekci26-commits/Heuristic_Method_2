import java.util.List;

/**
 * Stores all instance data for a CTOP-T-Sync problem.
 *
 * Parameters from the MIP model:
 *   - Q     : maxCapacity        (vehicle capacity)
 *   - Tmax  : maxRouteDuration   (time limit per route)
 *   - K     : maxVehicles        (number of vehicles)
 *   - W     : syncWindow         (maximum synchronization offset)
 */
public class Instance {

    private final String name;
    private final List<Node> nodes;
    private final int depotId;
    private final int maxVehicles;
    private final double maxCapacity;       // Q
    private final double maxRouteDuration;  // Tmax
    private final double syncWindow;        // W

    // ──────────────────────────── Constructors ───────────────────────────────

    /** Full constructor with sync window */
    public Instance(String name, List<Node> nodes, int depotId,
                    int maxVehicles, double maxCapacity,
                    double maxRouteDuration, double syncWindow) {
        this.name = name;
        this.nodes = nodes;
        this.depotId = depotId;
        this.maxVehicles = maxVehicles;
        this.maxCapacity = maxCapacity;
        this.maxRouteDuration = maxRouteDuration;
        this.syncWindow = syncWindow;
    }

    /** Constructor without sync window (defaults to Tmax, i.e. no sync restriction) */
    public Instance(String name, List<Node> nodes, int depotId,
                    int maxVehicles, double maxCapacity, double maxRouteDuration) {
        this(name, nodes, depotId, maxVehicles, maxCapacity, maxRouteDuration, maxRouteDuration);
    }

    // ──────────────────────────── Getters ────────────────────────────────────

    public String getName()              { return name; }
    public List<Node> getNodes()         { return nodes; }
    public int getDepotId()              { return depotId; }
    public int getMaxVehicles()          { return maxVehicles; }
    public double getMaxCapacity()       { return maxCapacity; }
    public double getMaxRouteDuration()  { return maxRouteDuration; }
    public double getSyncWindow()        { return syncWindow; }

    /** Number of customer nodes (excluding depot) */
    public int getNumCustomers() {
        return (int) nodes.stream().filter(n -> !n.isDepot()).count();
    }

    // ──────────────────────────── Utility ────────────────────────────────────

    public Node getNodeById(int id) {
        // Use direct index if IDs are sequential (0, 1, 2, ...)
        if (id >= 0 && id < nodes.size() && nodes.get(id).getId() == id) {
            return nodes.get(id);
        }
        // Fallback to linear search
        return nodes.stream()
                .filter(n -> n.getId() == id)
                .findFirst()
                .orElseThrow(() -> new IllegalArgumentException("Node not found: " + id));
    }

    public Node getDepot() {
        return getNodeById(depotId);
    }

    /** Euclidean distance between two nodes */
    public double getDistance(Node n1, Node n2) {
        double dx = n1.getX() - n2.getX();
        double dy = n1.getY() - n2.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    /** Precomputed distance matrix (optional, for performance) */
    private double[][] distMatrix;

    public void precomputeDistances() {
        int n = nodes.size();
        distMatrix = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                double d = getDistance(nodes.get(i), nodes.get(j));
                distMatrix[i][j] = d;
                distMatrix[j][i] = d;
            }
        }
    }

    /** Fast distance lookup using precomputed matrix */
    public double getDistanceFast(int id1, int id2) {
        if (distMatrix != null) return distMatrix[id1][id2];
        return getDistance(getNodeById(id1), getNodeById(id2));
    }

    @Override
    public String toString() {
        return String.format(
                "Instance[%s] vehicles=%d Q=%.1f Tmax=%.1f W=%.1f customers=%d",
                name, maxVehicles, maxCapacity, maxRouteDuration, syncWindow, getNumCustomers());
    }
}