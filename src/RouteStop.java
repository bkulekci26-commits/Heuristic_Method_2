/**
 * Represents a single stop in a vehicle's route.
 *
 * Maps directly to the MIP decision variables:
 * - served   ↔ zjk : customer j is served (demand fulfilled, profit collected)
 * - pickup   ↔ pjk : vehicle k picks up transferred load at node j
 * - dropoff  ↔ rjk : vehicle k drops off load at node j for another vehicle
 * - waitingTime ↔ wjk : time vehicle k waits at node j for synchronization
 *
 * Split Delivery Extension (Aguayo et al., 2025 adaptation):
 * - deliveryQty : amount this route delivers to the customer
 * In standard mode:  deliveryQty = node.demand (full service)
 * In split mode:     deliveryQty = partial amount (split across routes)
 */
public class RouteStop {

    private final Node node;

    // MIP variable mappings
    private boolean served;         // zjk
    private boolean pickup;         // pjk (receives transferred load)
    private boolean dropoff;        // rjk (leaves load for transfer)
    private double pickupQty;       // q_pick_jk
    private double dropoffQty;      // q_drop_jk
    private double deliveryQty;     // how much demand this route delivers
    private double waitingTime;     // wjk (explicit waiting time)

    // ──────────────────────────── Factory Methods ────────────────────────────

    public static RouteStop serve(Node node) {
        return new RouteStop(node, true, false, false, 0, 0, node.getDemand());
    }

    public static RouteStop servePartial(Node node, double partialDemand) {
        return new RouteStop(node, true, false, false, 0, 0, partialDemand);
    }

    public static RouteStop pickup(Node node, double qty) {
        return new RouteStop(node, false, true, false, qty, 0, 0);
    }

    public static RouteStop dropoff(Node node, double qty) {
        return new RouteStop(node, false, false, true, 0, qty, 0);
    }

    public static RouteStop serveAndDropoff(Node node, double dropoffQty) {
        return new RouteStop(node, true, false, true, 0, dropoffQty, node.getDemand());
    }

    public static RouteStop serveAndPickup(Node node, double pickupQty) {
        return new RouteStop(node, true, true, false, pickupQty, 0, node.getDemand());
    }

    // ──────────────────────────── Constructors ──────────────────────────────

    public RouteStop(Node node, boolean served, boolean pickup, boolean dropoff,
                     double pickupQty, double dropoffQty) {
        this(node, served, pickup, dropoff, pickupQty, dropoffQty,
                served ? node.getDemand() : 0);
    }

    public RouteStop(Node node, boolean served, boolean pickup, boolean dropoff,
                     double pickupQty, double dropoffQty, double deliveryQty) {
        this.node = node;
        this.served = served;
        this.pickup = pickup;
        this.dropoff = dropoff;
        this.pickupQty = pickupQty;
        this.dropoffQty = dropoffQty;
        this.deliveryQty = deliveryQty;
        this.waitingTime = 0.0;
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

    public double getLoadConsumption() {
        double consumption = 0;
        if (served) consumption += deliveryQty;
        consumption += dropoffQty;
        consumption -= pickupQty;
        return consumption;
    }

    // ──────────────────────────── Getters & Setters ─────────────────────────

    public Node getNode()           { return node; }
    public boolean isServed()       { return served; }
    public boolean isPickup()       { return pickup; }
    public boolean isDropoff()      { return dropoff; }
    public double getPickupQty()    { return pickupQty; }
    public double getDropoffQty()   { return dropoffQty; }
    public double getDeliveryQty()  { return deliveryQty; }
    public double getWaitingTime()  { return waitingTime; }

    public void setServed(boolean s)       { this.served = s; validate(); }
    public void setDeliveryQty(double qty) { this.deliveryQty = qty; }
    public void setWaitingTime(double w)   { this.waitingTime = Math.max(0, w); }

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

    public boolean isTransferOnly() {
        return !served && (pickup || dropoff);
    }

    public boolean isSplitDelivery() {
        return served && deliveryQty < node.getDemand() - 1e-6;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append(node.getId());
        if (served) {
            if (isSplitDelivery()) {
                sb.append(String.format("[S:%.0f/%.0f]", deliveryQty, node.getDemand()));
            } else {
                sb.append("[S]");
            }
        }
        if (pickup) sb.append(String.format("[P:%.1f]", pickupQty));
        if (dropoff) sb.append(String.format("[D:%.1f]", dropoffQty));
        if (waitingTime > 0) sb.append(String.format("[W:%.1f]", waitingTime));
        return sb.toString();
    }
}