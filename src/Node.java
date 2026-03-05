public class Node {
    private final int id;
    private final double x, y;
    private final double demand;
    private final double profit;
    private final boolean isDepot;

    public Node(int id, double x, double y, double demand, double profit, boolean isDepot) {
        this.id = id;
        this.x = x;
        this.y = y;
        this.demand = demand;
        this.profit = profit;
        this.isDepot = isDepot;
    }

    public int getId()         { return id; }
    public double getX()       { return x; }
    public double getY()       { return y; }
    public double getDemand()  { return demand; }
    public double getProfit()  { return profit; }
    public boolean isDepot()   { return isDepot; }

    @Override
    public String toString() {
        return String.format("Node[%d](%.1f,%.1f) demand=%.1f profit=%.1f",
                id, x, y, demand, profit);
    }
}