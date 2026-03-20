/**
 * Represents a load transfer between two vehicles at a customer node.
 *
 * The giving vehicle (k1) drops off load at the transfer node,
 * and the receiving vehicle (k2) picks it up.
 *
 * Constraint (10): Σ_k qpick_jk = Σ_k qdrop_jk   (conservation at each node)
 * Constraint (17): u_j,k1 - u_j,k2 ≤ W  (directional synchronization)
 */
public class Transfer {

    private final int transferNodeId;       // where the transfer happens
    private final int givingVehicleId;      // k1: drops off load (rjk1 = 1)
    private final int receivingVehicleId;   // k2: picks up load  (pjk2 = 1)
    private double quantity;                // amount transferred

    // ──────────────────────────── Constructor ────────────────────────────────

    public Transfer(int transferNodeId, int givingVehicleId,
                    int receivingVehicleId, double quantity) {
        this.transferNodeId = transferNodeId;
        this.givingVehicleId = givingVehicleId;
        this.receivingVehicleId = receivingVehicleId;
        this.quantity = quantity;
    }

    // ──────────────────────────── Getters & Setters ─────────────────────────

    public int getTransferNodeId()       { return transferNodeId; }
    public int getGivingVehicleId()      { return givingVehicleId; }
    public int getReceivingVehicleId()   { return receivingVehicleId; }
    public double getQuantity()          { return quantity; }
    public void setQuantity(double q)    { this.quantity = q; }

    @Override
    public String toString() {
        return String.format("Transfer[node=%d]: v%d ---(%.1f units)---> v%d",
                transferNodeId, givingVehicleId, quantity, receivingVehicleId);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof Transfer)) return false;
        Transfer t = (Transfer) o;
        return transferNodeId == t.transferNodeId
                && givingVehicleId == t.givingVehicleId
                && receivingVehicleId == t.receivingVehicleId;
    }

    @Override
    public int hashCode() {
        return 31 * (31 * transferNodeId + givingVehicleId) + receivingVehicleId;
    }
}