/**
 * Represents a single stop in a vehicle's route.
 *
 * Maps directly to the MIP decision variables:
 *   - served   ↔ zjk : customer j is served (demand fulfilled, profit collected)
 *   - pickup   ↔ pjk : vehicle k picks up transferred load at node j
 *   - dropoff  ↔ rjk : vehicle k drops off load at node j for another vehicle
 *
 * Constraint (8): vjk ≤ zjk + pjk + rjk  → if visited, must do at least one action
 * Constraint (9): pjk + rjk ≤ 1          → cannot both pickup and dropoff
 *
 * Note: A vehicle CAN serve AND pickup, or serve AND dropoff, at the same node.
 * Example: Vehicle serves customer j (delivers dj) and drops off extra load for transfer.
 */
public class RouteStop {

    private final Node node;

    // MIP variable mappings
    private boolean served;         // zjk
    private boolean pickup;         // pjk (receives transferred load)
    private boolean dropoff;        // rjk (leaves load for transfer)
    private double pickupQty;       // q_pick_jk
    private double dropoffQty;      // q_drop_jk

    // ──────────────────────────── Factory Methods ────────────────────────────

    /** Standard service stop: vehicle delivers demand and collects profit */
    public static RouteStop serve(Node node) {
        return new RouteStop(node, true, false, false, 0, 0);
    }

    /** Pickup-only stop: vehicle picks up load left by another vehicle */
    public static RouteStop pickup(Node node, double qty) {
        return new RouteStop(node, false, true, false, qty, 0);
    }

    /** Dropoff-only stop: vehicle leaves load for another vehicle to collect */
    public static RouteStop dropoff(Node node, double qty) {
        return new RouteStop(node, false, false, true, 0, qty);
    }

    /** Service + dropoff: serve customer AND leave extra load for transfer */
    public static RouteStop serveAndDropoff(Node node, double dropoffQty) {
        return new RouteStop(node, true, false, true, 0, dropoffQty);
    }

    /** Service + pickup: serve customer AND pick up transferred load */
    public static RouteStop serveAndPickup(Node node, double pickupQty) {
        return new RouteStop(node, true, true, false, pickupQty, 0);
    }

    // ──────────────────────────── Constructor ────────────────────────────────

    public RouteStop(Node node, boolean served, boolean pickup, boolean dropoff,
                     double pickupQty, double dropoffQty) {
        this.node = node;
        this.served = served;
        this.pickup = pickup;
        this.dropoff = dropoff;
        this.pickupQty = pickupQty;
        this.dropoffQty = dropoffQty;
        validate();
    }

    private void validate() {
        if (pickup && dropoff)
            throw new IllegalArgumentException(
                    "Constraint (9) violated: cannot both pickup and dropoff at node " + node.getId());
        if (!served && !pickup && !dropoff)
            throw new IllegalArgumentException(
                    "Constraint (8) violated: visited node " + node.getId() + " must have at least one action");
        if (pickupQty < 0 || dropoffQty < 0)
            throw new IllegalArgumentException("Transfer quantities must be non-negative");
    }

    // ──────────────────────────── Load Computation ──────────────────────────

    /**
     * Net load consumed at this stop (from the vehicle's perspective).
     * Positive = vehicle loses load, Negative = vehicle gains load.
     *
     * Maps to constraint (12): Σ y_ijk - Σ y_jlk = dj·zjk + qdrop_jk - qpick_jk
     */
    public double getLoadConsumption() {
        double consumption = 0;
        if (served) consumption += node.getDemand();   // deliver demand
        consumption += dropoffQty;                      // leave load for transfer
        consumption -= pickupQty;                       // gain load from transfer
        return consumption;
    }

    // ──────────────────────────── Getters & Setters ─────────────────────────

    public Node getNode()           { return node; }
    public boolean isServed()       { return served; }
    public boolean isPickup()       { return pickup; }
    public boolean isDropoff()      { return dropoff; }
    public double getPickupQty()    { return pickupQty; }
    public double getDropoffQty()   { return dropoffQty; }

    public void setServed(boolean s)       { this.served = s; validate(); }
    public void setPickup(boolean p, double qty) {
        this.pickup = p;
        this.pickupQty = p ? qty : 0;
        validate();
    }
    public void setDropoff(boolean d, double qty) {
        this.dropoff = d;
        this.dropoffQty = d ? qty : 0;
        validate();
    }

    /** Is this stop purely for transfer (not serving the customer)? */
    public boolean isTransferOnly() {
        return !served && (pickup || dropoff);
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append(node.getId());
        if (served) sb.append("[S]");
        if (pickup) sb.append(String.format("[P:%.1f]", pickupQty));
        if (dropoff) sb.append(String.format("[D:%.1f]", dropoffQty));
        return sb.toString();
    }
}