import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.stream.*;

/**
 * Experiment Runner for CTOP-T-Sync.
 *
 * Iterates over all instance files in a given directory, runs the ALNS
 * pipeline on each, and writes detailed results to a CSV file for analysis.
 *
 * Usage:
 *   ExperimentRunner.run(folderPath, outputCsvPath, alnsIterations, syncWindow, seed);
 */
public class ExperimentRunner {

    /**
     * Run experiments on all .txt instance files in the given folder.
     * Uses multi-start: runs ALNS with each seed, keeps the best solution.
     *
     * Tuned parameters (from Stage 1 + Stage 2 experiments):
     *   I=10000, c=0.9995, betaMax=6, seg=100, lambda=0.8
     *
     * @param instanceFolder  path to the directory containing instance .txt files
     * @param outputCsvPath   path for the output CSV report
     * @param alnsIterations  number of ALNS iterations per run
     * @param syncWindow      W parameter for synchronization
     * @param seeds           array of random seeds (multi-start: best-of-k)
     */
    public static void run(String instanceFolder, String outputCsvPath,
                           int alnsIterations, double syncWindow, long[] seeds) throws IOException {

        // ── Discover instance files ──
        List<File> instanceFiles = Files.list(Paths.get(instanceFolder))
                .filter(p -> p.toString().endsWith(".txt"))
                .map(Path::toFile)
                .sorted(Comparator.comparing(File::getName))
                .collect(Collectors.toList());

        System.out.printf("Found %d instance files in %s%n", instanceFiles.size(), instanceFolder);
        System.out.printf("Multi-start: best-of-%d seeds %s%n%n",
                seeds.length, java.util.Arrays.toString(seeds));

        if (instanceFiles.isEmpty()) {
            System.out.println("No .txt files found. Exiting.");
            return;
        }

        // ── Prepare CSV output ──
        List<String[]> results = new ArrayList<>();

        // CSV Header
        String[] header = {
                "Instance", "Customers", "Vehicles", "Capacity_Q", "Time_Tmax", "Sync_W",
                "Baseline_Profit", "Baseline_Served", "Baseline_Distance",
                "ALNS_Profit", "ALNS_Served", "ALNS_Distance", "ALNS_Transfers",
                "Improvement_Profit", "Improvement_Pct",
                "Gap_To_Baseline_Pct",
                "Best_Seed", "Num_Seeds",
                "Seed_Profits",
                "Num_New_Bests", "Acceptance_Rate_Pct",
                "Construction_Profit",
                "Sync_Gaps",
                "Route0_Profit", "Route0_Dist", "Route0_Time", "Route0_Load", "Route0_Stops",
                "Route1_Profit", "Route1_Dist", "Route1_Time", "Route1_Load", "Route1_Stops",
                "Route2_Profit", "Route2_Dist", "Route2_Time", "Route2_Load", "Route2_Stops",
                "Route3_Profit", "Route3_Dist", "Route3_Time", "Route3_Load", "Route3_Stops",
                "Elapsed_ms", "Feasible",
                "Transfer_Details"
        };
        results.add(header);

        // ── Run each instance ──
        int totalInstances = instanceFiles.size();
        int completed = 0;

        for (File file : instanceFiles) {
            completed++;
            String fileName = file.getName().replace(".txt", "");

            System.out.println("═".repeat(60));
            System.out.printf("  [%d/%d] Processing: %s%n", completed, totalInstances, fileName);
            System.out.println("═".repeat(60));

            try {
                // Read instance
                Instance inst = InstanceReader.read(file.getAbsolutePath(), syncWindow);
                inst.precomputeDistances();

                // ── Baseline: Construction + LS ──
                GreedyConstructive constructor = new GreedyConstructive();
                Solution baseline = constructor.construct(inst);
                new LocalSearch().improve(baseline);
                new LocalSearch().postInsert(baseline);

                double baselineProfit = baseline.getTotalProfit();
                int baselineServed = baseline.getNumServed();
                double baselineDist = baseline.getTotalDistance();
                double constructProfit = baselineProfit; // before LS for reference

                // ── Multi-start ALNS (best-of-k seeds) ──
                // Tuned parameters from Stage 1 + Stage 2 experiments:
                //   I=10000, c=0.9995, betaMax=6, seg=100, lambda=0.8
                long t0 = System.currentTimeMillis();
                Solution best = null;
                double bestProfit = Double.NEGATIVE_INFINITY;
                long bestSeed = seeds[0];
                int bestNewBests = 0;
                double bestAcceptRate = 0;
                StringBuilder seedProfits = new StringBuilder();

                for (long s : seeds) {
                    ALNSEngine alns = new ALNSEngine(
                            alnsIterations,
                            100,        // segmentLength (not significant)
                            50.0,       // initTemperature (auto-calibrated)
                            0.9995,     // coolingRate (tuned)
                            0.01,       // minTemperature
                            2,          // betaMin
                            6,          // betaMax (not significant, default)
                            0.8,        // reactionFactor (not significant)
                            s);
                    Solution candidate = alns.solve(inst);

                    double candProfit = candidate.getTotalProfit();
                    if (seedProfits.length() > 0) seedProfits.append(";");
                    seedProfits.append(String.format("%.0f", candProfit));

                    if (candidate.isFeasible() && candProfit > bestProfit) {
                        best = candidate;
                        bestProfit = candProfit;
                        bestSeed = s;
                        bestNewBests = alns.getNewBestCount();
                        bestAcceptRate = alns.getAcceptanceRate();
                    }
                }

                // Fallback: if no feasible solution, take last
                if (best == null) {
                    ALNSEngine fallback = new ALNSEngine(alnsIterations, seeds[0]);
                    best = fallback.solve(inst);
                    bestProfit = best.getTotalProfit();
                    bestSeed = seeds[0];
                }

                long elapsed = System.currentTimeMillis() - t0;

                // ── Collect metrics ──
                double alnsProfit = best.getTotalProfit();
                int alnsServed = best.getNumServed();
                double alnsDist = best.getTotalDistance();
                int alnsTransfers = best.getTransfers().size();
                double improvement = alnsProfit - baselineProfit;
                double improvementPct = baselineProfit > 0
                        ? 100.0 * improvement / baselineProfit : 0;

                boolean feasible = best.isFeasible();

                // ALNS statistics (from best seed's run)
                int numNewBests = bestNewBests;
                double acceptanceRate = bestAcceptRate;

                // Sync gaps
                StringBuilder syncGaps = new StringBuilder();
                for (Transfer t : best.getTransfers()) {
                    try {
                        Route giverRoute = best.getRouteByVehicleId(t.getGivingVehicleId());
                        Route receiverRoute = best.getRouteByVehicleId(t.getReceivingVehicleId());
                        giverRoute.evaluate();
                        receiverRoute.evaluate();
                        double giverArr = giverRoute.getArrivalTimeAtNode(t.getTransferNodeId());
                        double receiverArr = receiverRoute.getArrivalTimeAtNode(t.getTransferNodeId());
                        double gap = giverArr - receiverArr;
                        if (syncGaps.length() > 0) syncGaps.append(";");
                        syncGaps.append(String.format("%.1f", gap));
                    } catch (Exception e2) {
                        if (syncGaps.length() > 0) syncGaps.append(";");
                        syncGaps.append("ERR");
                    }
                }

                // Per-route details (pad to 4 routes)
                double[][] routeData = new double[4][5]; // [profit, dist, time, load, stops]
                for (int i = 0; i < Math.min(4, best.getRoutes().size()); i++) {
                    Route r = best.getRoutes().get(i);
                    r.evaluate();
                    routeData[i][0] = r.getTotalProfit();
                    routeData[i][1] = r.getTotalDistance();
                    routeData[i][2] = r.getTotalTime();
                    routeData[i][3] = r.getInitialLoad();
                    routeData[i][4] = r.size();
                }

                // Transfer details string
                StringBuilder transferStr = new StringBuilder();
                for (Transfer t : best.getTransfers()) {
                    if (transferStr.length() > 0) transferStr.append(" | ");
                    transferStr.append(String.format("node%d:v%d->v%d(%.0f)",
                            t.getTransferNodeId(), t.getGivingVehicleId(),
                            t.getReceivingVehicleId(), t.getQuantity()));
                }

                // ── Build result row ──
                String[] row = {
                        fileName,
                        String.valueOf(inst.getNumCustomers()),
                        String.valueOf(inst.getMaxVehicles()),
                        fmt(inst.getMaxCapacity()),
                        fmt(inst.getMaxRouteDuration()),
                        fmt(syncWindow),
                        fmt(baselineProfit),
                        String.valueOf(baselineServed),
                        fmt(baselineDist),
                        fmt(alnsProfit),
                        String.valueOf(alnsServed),
                        fmt(alnsDist),
                        String.valueOf(alnsTransfers),
                        fmt(improvement),
                        fmt(improvementPct),
                        fmt(improvementPct), // gap is same as improvement over baseline
                        String.valueOf(bestSeed),
                        String.valueOf(seeds.length),
                        seedProfits.toString(),
                        String.valueOf(numNewBests),
                        fmt(acceptanceRate),
                        fmt(constructProfit),
                        syncGaps.toString(),
                        // Route 0-3
                        fmt(routeData[0][0]), fmt(routeData[0][1]), fmt(routeData[0][2]), fmt(routeData[0][3]), fmt(routeData[0][4]),
                        fmt(routeData[1][0]), fmt(routeData[1][1]), fmt(routeData[1][2]), fmt(routeData[1][3]), fmt(routeData[1][4]),
                        fmt(routeData[2][0]), fmt(routeData[2][1]), fmt(routeData[2][2]), fmt(routeData[2][3]), fmt(routeData[2][4]),
                        fmt(routeData[3][0]), fmt(routeData[3][1]), fmt(routeData[3][2]), fmt(routeData[3][3]), fmt(routeData[3][4]),
                        String.valueOf(elapsed),
                        String.valueOf(feasible),
                        transferStr.toString()
                };
                results.add(row);

                // Console summary
                System.out.printf("  Result: profit=%.0f (baseline=%.0f, Δ%+.0f, %+.1f%%) | " +
                                "served=%d/%d | transfers=%d | time=%dms | feasible=%s%n",
                        alnsProfit, baselineProfit, improvement, improvementPct,
                        alnsServed, inst.getNumCustomers(), alnsTransfers, elapsed, feasible);
                System.out.printf("  Multi-start: best-of-%d seeds, winner=seed %d, profits=[%s]%n",
                        seeds.length, bestSeed, seedProfits);

                // ── Detailed route printout (always print for verification) ──
                printDetailedRoutes(best, inst);
                System.out.println();

            } catch (Exception e) {
                System.err.printf("  ERROR on %s: %s%n%n", fileName, e.getMessage());
                e.printStackTrace();

                // Record error row
                String[] errorRow = new String[header.length];
                Arrays.fill(errorRow, "");
                errorRow[0] = fileName;
                errorRow[header.length - 1] = "ERROR: " + e.getMessage();
                results.add(errorRow);
            }
        }

        // ── Write CSV ──
        writeCsv(outputCsvPath, results);
        System.out.println("\n" + "═".repeat(60));
        System.out.printf("  EXPERIMENT COMPLETE: %d instances processed%n", totalInstances);
        System.out.printf("  Results written to: %s%n", outputCsvPath);
        System.out.println("═".repeat(60));
    }

    // ── Detailed Route Printing ──

    /**
     * Prints a node-by-node breakdown of each route for visual feasibility verification.
     *
     * Format per route:
     *   depot(0) →[load/Q, t=0.0] node_id[action] →[load/Q, t=arr] ... → depot(0) [t=total]
     *
     * Actions: [S] = serve, [S:partial/full] = split serve, [P:qty] = pickup, [D:qty] = dropoff
     * Load shown on each arc (between nodes). Capacity Q and time Tmax shown for comparison.
     */
    private static void printDetailedRoutes(Solution sol, Instance inst) {
        double Q = inst.getMaxCapacity();
        double Tmax = inst.getMaxRouteDuration();
        double W = inst.getSyncWindow();

        System.out.println("  ┌─────────────────────────────────────────────────────────────┐");
        System.out.printf( "  │  DETAILED ROUTES  (Q=%.0f, Tmax=%.0f, W=%.0f)%n", Q, Tmax, W);
        System.out.println("  ├─────────────────────────────────────────────────────────────┤");

        for (Route r : sol.getRoutes()) {
            r.evaluate();
            System.out.printf("  │  Vehicle %d:  profit=%.0f  time=%.1f/%.0f  load=%.0f/%.0f  stops=%d%n",
                    r.getVehicleId(), r.getTotalProfit(), r.getTotalTime(), Tmax,
                    r.getInitialLoad(), Q, r.size());

            // Print node-by-node sequence
            StringBuilder seq = new StringBuilder();
            StringBuilder details = new StringBuilder();
            Node depot = inst.getDepot();

            seq.append(String.format("  │    0(depot)"));
            details.append(String.format("  │    t=%-6.1f load=%-5.0f ", 0.0, r.getArcLoad(0)));

            for (int i = 0; i < r.size(); i++) {
                RouteStop s = r.getStops().get(i);
                Node n = s.getNode();
                double arrTime = r.getArrivalTime(i + 1); // +1 because index 0 is depot
                double arcLoadAfter = (i + 1 < r.size() + 1) ? r.getArcLoad(i + 1) : 0;

                // Build action string
                StringBuilder action = new StringBuilder();
                if (s.isServed()) {
                    if (s.isSplitDelivery()) {
                        action.append(String.format("S:%.0f/%.0f", s.getDeliveryQty(), n.getDemand()));
                    } else {
                        action.append("S");
                    }
                }
                if (s.isPickup()) {
                    if (action.length() > 0) action.append("+");
                    action.append(String.format("P:%.0f", s.getPickupQty()));
                }
                if (s.isDropoff()) {
                    if (action.length() > 0) action.append("+");
                    action.append(String.format("D:%.0f", s.getDropoffQty()));
                }

                seq.append(String.format(" → %d[%s]", n.getId(), action));
                details.append(String.format("→ t=%-6.1f load=%-5.0f ", arrTime, arcLoadAfter));
            }

            // Return to depot
            double returnTime = r.getArrivalTime(r.size() + 1);
            seq.append(String.format(" → 0(depot)"));
            details.append(String.format("→ t=%-6.1f", returnTime));

            System.out.println(seq);
            System.out.println(details);

            // Feasibility check per route
            boolean timeFeasible = r.getTotalTime() <= Tmax + 1e-6;
            boolean capFeasible = true;
            for (int i = 0; i <= r.size(); i++) {
                if (r.getArcLoad(i) > Q + 1e-6 || r.getArcLoad(i) < -1e-6) {
                    capFeasible = false;
                    break;
                }
            }
            String status = (timeFeasible && capFeasible) ? "✓ FEASIBLE" :
                    (!timeFeasible && !capFeasible) ? "✗ TIME+CAP VIOLATED" :
                            (!timeFeasible) ? "✗ TIME VIOLATED" : "✗ CAPACITY VIOLATED";
            System.out.printf("  │    Status: %s%n", status);
            System.out.println("  │");
        }

        // Print transfer details
        if (!sol.getTransfers().isEmpty()) {
            System.out.println("  ├────────────────────────────────────────────────────────────┤");
            System.out.println("  │  TRANSFERS:");
            for (Transfer t : sol.getTransfers()) {
                try {
                    Route giverRoute = sol.getRouteByVehicleId(t.getGivingVehicleId());
                    Route receiverRoute = sol.getRouteByVehicleId(t.getReceivingVehicleId());
                    double giverArr = giverRoute.getArrivalTimeAtNode(t.getTransferNodeId());
                    double receiverArr = receiverRoute.getArrivalTimeAtNode(t.getTransferNodeId());
                    double syncGap = giverArr - receiverArr;
                    String syncStatus = (syncGap <= W + 1e-6) ? "✓" : "✗ SYNC VIOLATED";

                    System.out.printf("  │    Node %d: v%d(dropper, t=%.1f) → v%d(picker, t=%.1f) " +
                                    "qty=%.0f  gap=%.1f  %s%n",
                            t.getTransferNodeId(),
                            t.getGivingVehicleId(), giverArr,
                            t.getReceivingVehicleId(), receiverArr,
                            t.getQuantity(), syncGap, syncStatus);
                } catch (Exception e) {
                    System.out.printf("  │    Transfer at node %d: ERROR - %s%n",
                            t.getTransferNodeId(), e.getMessage());
                }
            }
        }

        // Overall feasibility
        Solution.FeasibilityReport report = sol.checkFeasibility();
        System.out.println("  ├─────────────────────────────────────────────────────────────┤");
        if (report.isFeasible()) {
            System.out.printf("  │  OVERALL: ✓ FEASIBLE  profit=%.0f  served=%d  transfers=%d%n",
                    sol.getTotalProfit(), sol.getNumServed(), sol.getTransfers().size());
        } else {
            System.out.printf("  │  OVERALL: ✗ INFEASIBLE  (%d violations)%n", report.getViolations().size());
            for (String v : report.getViolations()) {
                System.out.printf("  │    ✗ %s%n", v);
            }
        }
        System.out.println("  └─────────────────────────────────────────────────────────────┘");
    }

    // ── Utility ──

    private static String fmt(double val) {
        if (val == (long) val) return String.valueOf((long) val);
        return String.format("%.2f", val);
    }

    private static void writeCsv(String path, List<String[]> rows) throws IOException {
        try (PrintWriter pw = new PrintWriter(new FileWriter(path))) {
            for (String[] row : rows) {
                StringBuilder sb = new StringBuilder();
                for (int i = 0; i < row.length; i++) {
                    if (i > 0) sb.append(",");
                    // Escape commas and quotes in values
                    String val = row[i] != null ? row[i] : "";
                    if (val.contains(",") || val.contains("\"") || val.contains("\n")) {
                        val = "\"" + val.replace("\"", "\"\"") + "\"";
                    }
                    sb.append(val);
                }
                pw.println(sb.toString());
            }
        }
    }
}