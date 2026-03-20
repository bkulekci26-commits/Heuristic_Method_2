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
     *
     * @param instanceFolder  path to the directory containing instance .txt files
     * @param outputCsvPath   path for the output CSV report
     * @param alnsIterations  number of ALNS iterations per instance
     * @param syncWindow      W parameter for synchronization
     * @param seed            random seed for reproducibility
     */
    public static void run(String instanceFolder, String outputCsvPath,
                           int alnsIterations, double syncWindow, long seed) throws IOException {

        // ── Discover instance files ──
        List<File> instanceFiles = Files.list(Paths.get(instanceFolder))
                .filter(p -> p.toString().endsWith(".txt"))
                .map(Path::toFile)
                .sorted(Comparator.comparing(File::getName))
                .collect(Collectors.toList());

        System.out.printf("Found %d instance files in %s%n%n", instanceFiles.size(), instanceFolder);

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
                "Num_New_Bests", "Acceptance_Rate_Pct",
                "Construction_Profit",
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

                // ── ALNS ──
                long t0 = System.currentTimeMillis();
                ALNSEngine alns = new ALNSEngine(alnsIterations, seed);
                Solution best = alns.solve(inst);
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

                // ALNS statistics
                int numNewBests = alns.getNewBestCount();
                double acceptanceRate = alns.getAcceptanceRate();

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
                        String.valueOf(numNewBests),
                        fmt(acceptanceRate),
                        fmt(constructProfit),
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
                                "served=%d/%d | transfers=%d | time=%dms | feasible=%s%n%n",
                        alnsProfit, baselineProfit, improvement, improvementPct,
                        alnsServed, inst.getNumCustomers(), alnsTransfers, elapsed, feasible);

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