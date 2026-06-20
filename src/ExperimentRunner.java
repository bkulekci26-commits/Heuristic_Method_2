import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.stream.*;

public class ExperimentRunner {

    public static void run(String instanceFolder, String outputCsvPath, int alnsIterations, double syncWindow, long[] seeds) throws IOException {
        List<File> instanceFiles = Files.list(Paths.get(instanceFolder))
                .filter(p -> p.toString().endsWith(".txt"))
                .map(Path::toFile)
                .sorted(Comparator.comparing(File::getName))
                .collect(Collectors.toList());

        System.out.printf("Starting Experiment on %d instances...\n", instanceFiles.size());
        if (instanceFiles.isEmpty()) return;

        List<String[]> results = new ArrayList<>();
        // SIMPLIFIED HEADER
        results.add(new String[]{
                "Instance", "Customers", "Vehicles", "Capacity_Q", "Time_Tmax", "Sync_W",
                "Final_Profit", "Elapsed_ms", "Feasible", "Splits_Count", "Transfers_Count",
                "Split_Details", "Transfer_Details"
        });

        int completed = 0;
        for (File file : instanceFiles) {
            completed++;
            String fileName = file.getName().replace(".txt", "");
            System.out.printf("[%d/%d] Solving: %-20s ... ", completed, instanceFiles.size(), fileName);

            try {
                Instance inst = InstanceReader.read(file.getAbsolutePath(), syncWindow);
                inst.precomputeDistances();

                long t0 = System.currentTimeMillis();
                Solution best = null;
                double bestProfit = Double.NEGATIVE_INFINITY;

                for (long s : seeds) {
                    ALNSEngine alns = new ALNSEngine(alnsIterations, s);
                    Solution candidate = alns.solve(inst);

                    if (candidate.isFeasible() && candidate.getTotalProfit() > bestProfit) {
                        best = candidate;
                        bestProfit = candidate.getTotalProfit();
                    }
                }

                long elapsed = System.currentTimeMillis() - t0;

                // --- SAFETY NET: Check if a feasible solution was ever found ---
                if (best == null) {
                    results.add(new String[]{
                            fileName, String.valueOf(inst.getNumCustomers()), String.valueOf(inst.getMaxVehicles()),
                            String.valueOf(inst.getMaxCapacity()), String.valueOf(inst.getMaxRouteDuration()), String.valueOf(syncWindow),
                            "0", String.valueOf(elapsed), "false", "0", "0", "None", "None"
                    });
                    System.out.printf("Profit: FAILED | Time: %4dms (No feasible solution found)\n", elapsed);
                    continue; // Skip the rest of the loop for this instance
                }

                int splits = best.getSplitCustomers().size();
                int transfers = best.getTransfers().size();

                // Format Split Details
                StringBuilder splitStr = new StringBuilder();
                for (Map.Entry<Integer, List<int[]>> entry : best.getSplitCustomers().entrySet()) {
                    splitStr.append("N").append(entry.getKey()).append("(");
                    for (int[] v : entry.getValue()) splitStr.append("v").append(v[1]).append(",");
                    splitStr.append(") ");
                }

                // Format Transfer Details
                StringBuilder transferStr = new StringBuilder();
                for (Transfer t : best.getTransfers()) {
                    transferStr.append("N").append(t.getTransferNodeId())
                            .append(":v").append(t.getGivingVehicleId())
                            .append("->v").append(t.getReceivingVehicleId()).append(" ");
                }

                results.add(new String[]{
                        fileName, String.valueOf(inst.getNumCustomers()), String.valueOf(inst.getMaxVehicles()),
                        String.valueOf(inst.getMaxCapacity()), String.valueOf(inst.getMaxRouteDuration()), String.valueOf(syncWindow),
                        String.valueOf(bestProfit), String.valueOf(elapsed), String.valueOf(best.isFeasible()),
                        String.valueOf(splits), String.valueOf(transfers),
                        splitStr.toString().isEmpty() ? "None" : splitStr.toString(),
                        transferStr.toString().isEmpty() ? "None" : transferStr.toString()
                });

                System.out.printf("Profit: %6.0f | Splits: %d | Transfers: %d | Time: %4dms\n", bestProfit, splits, transfers, elapsed);

            } catch (Exception e) {
                System.out.println("ERROR: " + e.getMessage());
            }
        }
        writeCsv(outputCsvPath, results);
        System.out.println("\nExperiment finished. Results saved to " + outputCsvPath);
    }

    private static void writeCsv(String path, List<String[]> rows) throws IOException {
        try (PrintWriter pw = new PrintWriter(new FileWriter(path))) {
            for (String[] row : rows) pw.println(String.join(",", row));
        }
    }
}