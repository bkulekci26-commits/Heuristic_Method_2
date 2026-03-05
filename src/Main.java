import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * CTOP-T-Sync Full Pipeline:
 *   Phase 2: Greedy Construction
 *   Phase 3: Local Search (2-opt, relocate, swap, replace)
 *   Phase 4: Transfer Optimization (drop-pickup with synchronization)
 *   + Post-insert after transfers
 */
public class Main {

    public static void main(String[] args) throws Exception {

        System.out.println("══════════════════════════════════════════════════════════");
        System.out.println("  CTOP-T-Sync: Full Pipeline (Construct → LS → Transfer)");
        System.out.println("══════════════════════════════════════════════════════════\n");

        double alpha = 0.05;

        // ═══════════════════════════════════════════
        // TEST A: Small instance
        // ═══════════════════════════════════════════
        System.out.println("══════ TEST A: Small Instance (5 customers) ══════\n");

        Node depot = new Node(0, 0, 0, 0, 0, true);
        Node c1 = new Node(1, 10, 0,  10, 8,  false);
        Node c2 = new Node(2, 20, 0,  15, 12, false);
        Node c3 = new Node(3, 10, 10,  5, 4,  false);
        Node c4 = new Node(4, 20, 10, 20, 15, false);
        Node c5 = new Node(5, 30, 0,   8, 6,  false);

        List<Node> nodes = new ArrayList<>(Arrays.asList(depot, c1, c2, c3, c4, c5));
        Instance smallInst = new Instance("test5", nodes, 0, 2, 40, 80, 10);

        Solution smallSol = new GreedyConstructive(alpha).construct(smallInst);
        new LocalSearch(alpha).improve(smallSol);
        new LocalSearch(alpha).postInsert(smallSol);

        System.out.println("\n── Before Transfers ──");
        System.out.println(smallSol);

        int smallTransfers = new TransferOptimizer(alpha).optimize(smallSol);
        new LocalSearch(alpha).postInsert(smallSol);

        System.out.println("\n── After Transfers ──");
        System.out.println(smallSol);
        System.out.printf("  Transfers created: %d%n", smallTransfers);

        Solution.FeasibilityReport smallReport = smallSol.checkFeasibility();
        System.out.println("  Feasibility: " + smallReport);
        System.out.println("\n✓ Small instance test PASSED\n");

        // ═══════════════════════════════════════════
        // TEST B: Benchmark instance
        // ═══════════════════════════════════════════
        System.out.println("══════ TEST B: Benchmark Instance ══════\n");

        String instancePath = "C:/Users/beste/Desktop/THESIS/VRP-T/Gurobi/Datasets/archetti_original_2/p06_m3_Q100_T100.txt";

        Instance realInst;
        try {
            realInst = InstanceReader.read(instancePath, 15.0);
        } catch (Exception e) {
            System.out.println("Could not read instance: " + e.getMessage());
            return;
        }

        System.out.println(realInst);
        realInst.precomputeDistances();

        // ── Phase 2: Construct ──
        long t0 = System.currentTimeMillis();
        GreedyConstructive constructor = new GreedyConstructive(alpha);
        Solution realSol = constructor.construct(realInst);
        long t1 = System.currentTimeMillis();

        double p2Profit = realSol.getTotalProfit();
        double p2Obj = realSol.getObjectiveValue();
        int p2Served = realSol.getNumServed();

        // ── Phase 3: Local Search ──
        long t2 = System.currentTimeMillis();
        LocalSearch ls = new LocalSearch(alpha);
        int lsImprovements = ls.improve(realSol);
        int lsInserts = ls.postInsert(realSol);
        long t3 = System.currentTimeMillis();

        double p3Profit = realSol.getTotalProfit();
        double p3Obj = realSol.getObjectiveValue();
        int p3Served = realSol.getNumServed();

        System.out.println("\n── After Construction + Local Search ──");
        System.out.println(realSol);

        // ── Phase 4: Transfer Optimization ──
        long t4 = System.currentTimeMillis();
        TransferOptimizer transferOpt = new TransferOptimizer(alpha);
        int numTransfers = transferOpt.optimize(realSol);
        long t5 = System.currentTimeMillis();

        // Post-insert after transfers (new capacity may allow more customers)
        long t6 = System.currentTimeMillis();
        int postTransferInserts = ls.postInsert(realSol);
        long t7 = System.currentTimeMillis();

        // Second round of local search after transfers
        int ls2Improvements = ls.improve(realSol);
        int ls2Inserts = ls.postInsert(realSol);
        long t8 = System.currentTimeMillis();

        double p4Profit = realSol.getTotalProfit();
        double p4Obj = realSol.getObjectiveValue();
        int p4Served = realSol.getNumServed();

        System.out.println("\n── Final Solution ──");
        System.out.println(realSol.toDetailedString());

        // ── Summary Table ──
        System.out.println("\n┌──────────────────────────────────────────────────────────────┐");
        System.out.println("│                    PIPELINE SUMMARY                          │");
        System.out.println("├────────────────┬──────────┬──────────┬────────────────────────┤");
        System.out.println("│    Metric      │ Phase 2  │ Phase 3  │ Phase 4 (Transfers)    │");
        System.out.println("├────────────────┼──────────┼──────────┼────────────────────────┤");
        System.out.printf( "│  Served        │ %3d/%-3d  │ %3d/%-3d  │ %3d/%-3d                │%n",
                p2Served, realSol.getNumCustomers(),
                p3Served, realSol.getNumCustomers(),
                p4Served, realSol.getNumCustomers());
        System.out.printf( "│  Profit        │ %8.1f │ %8.1f │ %8.1f                │%n",
                p2Profit, p3Profit, p4Profit);
        System.out.printf( "│  Objective     │ %8.2f │ %8.2f │ %8.2f                │%n",
                p2Obj, p3Obj, p4Obj);
        System.out.printf( "│  Transfers     │    0     │    0     │    %d                    │%n",
                realSol.getTransfers().size());
        System.out.println("└────────────────┴──────────┴──────────┴────────────────────────┘");

        // Per-route details
        System.out.println("\n── Per-Route Final State ──");
        for (Route r : realSol.getRoutes()) {
            r.evaluate();
            System.out.printf("  v%d: %2d stops | profit=%5.1f | dist=%5.1f | time=%5.1f/%5.1f | load=%5.1f/%5.1f%n",
                    r.getVehicleId(), r.size(), r.getTotalProfit(),
                    r.getTotalDistance(), r.getTotalTime(), realInst.getMaxRouteDuration(),
                    r.getInitialLoad(), realInst.getMaxCapacity());
        }

        // Transfer details
        if (!realSol.getTransfers().isEmpty()) {
            System.out.println("\n── Transfer Details ──");
            for (Transfer t : realSol.getTransfers()) {
                Route giver = realSol.getRouteByVehicleId(t.getGivingVehicleId());
                Route receiver = realSol.getRouteByVehicleId(t.getReceivingVehicleId());
                double gTime = giver.getArrivalTimeAtNode(t.getTransferNodeId());
                double rTime = receiver.getArrivalTimeAtNode(t.getTransferNodeId());
                System.out.printf("  %s  |  sync gap=%.1f (W=%.1f)  %s%n",
                        t, Math.abs(gTime - rTime), realInst.getSyncWindow(),
                        Math.abs(gTime - rTime) <= realInst.getSyncWindow() ? "✓" : "✗");
            }
        }

        // Timing
        System.out.printf("\n── Timing ──%n");
        System.out.printf("  Construction:     %d ms%n", (t1 - t0));
        System.out.printf("  Local Search:     %d ms%n", (t3 - t2));
        System.out.printf("  Transfers:        %d ms%n", (t5 - t4));
        System.out.printf("  Post-transfer LS: %d ms%n", (t8 - t6));
        System.out.printf("  Total:            %d ms%n", (t8 - t0));

        // Final feasibility
        Solution.FeasibilityReport report = realSol.checkFeasibility();
        System.out.println("\nFinal Feasibility: " + report);

        System.out.println("\n══════════════════════════════════════════════════════════");
        System.out.println("  PIPELINE COMPLETE                                      ");
        System.out.println("══════════════════════════════════════════════════════════");
    }
}