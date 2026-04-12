import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.stream.*;

/**
 * Parameter Tuning Experiment for CTOP-T-Sync ALNS.
 *
 * Design: One-Factor-At-a-Time (OFAT)
 *   - Baseline configuration with default parameters
 *   - For each parameter: test several values while keeping others at default
 *   - Records performance metrics per (config, instance) pair
 *
 * Output: CSV file (openable in Excel) with all results.
 *
 * Usage:
 *   ParameterTuningRunner.run(instanceFolder, outputPath, syncWindow, seed);
 */
public class ParameterTuningRunner {

    // ══════════════════════════════════════════════════════════
    //  PARAMETER CONFIGURATION
    // ══════════════════════════════════════════════════════════

    /** A single parameter configuration to test */
    static class Config {
        String name;           // human-readable label
        String parameterName;  // which parameter is being varied
        int maxIterations;
        int segmentLength;
        double coolingRate;
        int betaMin;
        int betaMax;
        double reactionFactor;

        Config(String name, String parameterName,
               int maxIter, int segLen, double cooling,
               int bMin, int bMax, double reaction) {
            this.name = name;
            this.parameterName = parameterName;
            this.maxIterations = maxIter;
            this.segmentLength = segLen;
            this.coolingRate = cooling;
            this.betaMin = bMin;
            this.betaMax = bMax;
            this.reactionFactor = reaction;
        }
    }

    /**
     * Build the OFAT parameter tuning matrix.
     *
     * Baseline defaults (from Hammami 2024 sensitivity analysis):
     *   maxIterations = 5000
     *   segmentLength = 100
     *   coolingRate    = 0.9997
     *   betaMin        = 2
     *   betaMax        = 6
     *   reactionFactor = 0.8
     */
    private static List<Config> buildConfigurations() {
        List<Config> configs = new ArrayList<>();

        // Default values
        int DEF_ITER = 5000;
        int DEF_SEG  = 100;
        double DEF_COOL = 0.9997;
        int DEF_BMIN = 2;
        int DEF_BMAX = 6;
        double DEF_REACT = 0.8;

        // ── 0. BASELINE ──
        configs.add(new Config("BASELINE",
                "baseline", DEF_ITER, DEF_SEG, DEF_COOL, DEF_BMIN, DEF_BMAX, DEF_REACT));

        // ── 1. COOLING RATE (c) ── most impactful SA parameter
        //    Lower = faster cooling = more exploitation
        //    Higher = slower cooling = more exploration
        for (double c : new double[]{0.9990, 0.9995, 0.9999}) {
            configs.add(new Config(
                    String.format("c=%.4f", c),
                    "coolingRate", DEF_ITER, DEF_SEG, c, DEF_BMIN, DEF_BMAX, DEF_REACT));
        }

        // ── 2. MAX ITERATIONS (I) ── quality vs runtime
        for (int iter : new int[]{3000, 7500, 10000}) {
            configs.add(new Config(
                    String.format("I=%d", iter),
                    "maxIterations", iter, DEF_SEG, DEF_COOL, DEF_BMIN, DEF_BMAX, DEF_REACT));
        }

        // ── 3. BETA MAX (β_max) ── destruction magnitude
        //    Larger = more diversification per destroy-repair cycle
        for (int bmax : new int[]{4, 8, 10}) {
            configs.add(new Config(
                    String.format("bMax=%d", bmax),
                    "betaMax", DEF_ITER, DEF_SEG, DEF_COOL, DEF_BMIN, bmax, DEF_REACT));
        }

        // ── 4. SEGMENT LENGTH ── weight update frequency
        //    Shorter = faster adaptation, noisier weights
        for (int seg : new int[]{50, 200}) {
            configs.add(new Config(
                    String.format("seg=%d", seg),
                    "segmentLength", DEF_ITER, seg, DEF_COOL, DEF_BMIN, DEF_BMAX, DEF_REACT));
        }

        // ── 5. REACTION FACTOR (λ) ── weight memory
        //    Lower = faster adaptation to recent operator performance
        //    Higher = more stable weights, longer memory
        for (double r : new double[]{0.5, 0.95}) {
            configs.add(new Config(
                    String.format("lam=%.2f", r),
                    "reactionFactor", DEF_ITER, DEF_SEG, DEF_COOL, DEF_BMIN, DEF_BMAX, r));
        }

        return configs;
    }

    // ══════════════════════════════════════════════════════════
    //  MAIN ENTRY POINT
    // ══════════════════════════════════════════════════════════

    public static void run(String instanceFolder, String outputPath,
                           double syncWindow, long seed) throws IOException {

        List<Config> configs = buildConfigurations();

        // ── Discover instance files ──
        List<File> instanceFiles = Files.list(Paths.get(instanceFolder))
                .filter(p -> p.toString().endsWith(".txt"))
                .map(Path::toFile)
                .sorted(Comparator.comparing(File::getName))
                .collect(Collectors.toList());

        int nInstances = instanceFiles.size();
        int nConfigs = configs.size();
        int totalRuns = nInstances * nConfigs;

        System.out.println("╔══════════════════════════════════════════════════════════╗");
        System.out.println("║  CTOP-T-Sync: Parameter Tuning Experiment               ║");
        System.out.println("╠══════════════════════════════════════════════════════════╣");
        System.out.printf( "║  Instances:       %d%n", nInstances);
        System.out.printf( "║  Configurations:  %d%n", nConfigs);
        System.out.printf( "║  Total runs:      %d%n", totalRuns);
        System.out.printf( "║  Sync window (W): %.1f%n", syncWindow);
        System.out.printf( "║  Random seed:     %d%n", seed);
        System.out.printf( "║  Output:          %s%n", outputPath);
        System.out.println("╚══════════════════════════════════════════════════════════╝");
        System.out.println();

        // Print configuration table
        System.out.println("  Configurations to test:");
        System.out.println("  ─────────────────────────────────────────────────────────");
        System.out.printf("  %-3s  %-14s  %-14s  %5s  %3s  %8s  %4s  %4s  %5s%n",
                "#", "Name", "Parameter", "Iter", "Seg", "Cool", "bMin", "bMax", "λ");
        System.out.println("  ─────────────────────────────────────────────────────────");
        for (int i = 0; i < configs.size(); i++) {
            Config c = configs.get(i);
            System.out.printf("  %-3d  %-14s  %-14s  %5d  %3d  %.4f  %4d  %4d  %.2f%n",
                    i, c.name, c.parameterName, c.maxIterations, c.segmentLength,
                    c.coolingRate, c.betaMin, c.betaMax, c.reactionFactor);
        }
        System.out.println();

        // ── Prepare CSV ──
        PrintWriter csv = new PrintWriter(new BufferedWriter(new FileWriter(outputPath)));
        csv.println(String.join(",", new String[]{
                "Config_ID", "Config_Name", "Parameter_Name",
                "Iterations", "Segment_Length", "Cooling_Rate",
                "Beta_Min", "Beta_Max", "Reaction_Factor",
                "Instance", "Customers", "Vehicles", "Capacity_Q", "Time_Tmax", "Sync_W",
                "Profit", "Served", "Transfers",
                "TransferA_Count", "TransferB_Count",
                "Sync_Gaps",
                "Elapsed_ms", "Feasible",
                "New_Bests", "Acceptance_Rate_Pct"
        }));
        csv.flush();

        // ── Suppress verbose output during runs ──
        PrintStream originalOut = System.out;
        PrintStream nullStream = new PrintStream(new OutputStream() {
            public void write(int b) { }
        });

        // ── Run all (config, instance) pairs ──
        int runCount = 0;
        long experimentStart = System.currentTimeMillis();

        for (int ci = 0; ci < nConfigs; ci++) {
            Config config = configs.get(ci);
            long configStart = System.currentTimeMillis();

            originalOut.printf("  [Config %d/%d] %s  (%s)%n",
                    ci + 1, nConfigs, config.name, config.parameterName);

            for (int fi = 0; fi < nInstances; fi++) {
                File file = instanceFiles.get(fi);
                String fileName = file.getName().replace(".txt", "");
                runCount++;

                // Progress (overwrite line)
                if (fi % 10 == 0 || fi == nInstances - 1) {
                    originalOut.printf("\r    Instance %d/%d  (%s)  [run %d/%d]",
                            fi + 1, nInstances, fileName, runCount, totalRuns);
                    originalOut.flush();
                }

                try {
                    // Read instance
                    Instance inst = InstanceReader.read(file.getAbsolutePath(), syncWindow);
                    inst.precomputeDistances();

                    // Suppress console output during ALNS
                    System.setOut(nullStream);

                    // Run ALNS with this configuration
                    long t0 = System.currentTimeMillis();
                    ALNSEngine alns = new ALNSEngine(
                            config.maxIterations,
                            config.segmentLength,
                            50.0,   // initTemperature (auto-calibrated anyway)
                            config.coolingRate,
                            0.01,   // minTemperature
                            config.betaMin,
                            config.betaMax,
                            config.reactionFactor,
                            seed);
                    Solution best = alns.solve(inst);
                    long elapsed = System.currentTimeMillis() - t0;

                    // Restore output
                    System.setOut(originalOut);

                    // Count transfer types
                    int transferA = 0, transferB = 0;
                    for (Transfer t : best.getTransfers()) {
                        // If the picker route does NOT serve the transfer node → Archetype A
                        // If the picker route DOES serve (pickup only) → Archetype B
                        // Simple heuristic: count all as "transfer" for now
                        transferA++; // We'll refine later if needed
                    }

                    // Compute sync gaps for each transfer
                    // syncGap = dropper_arrival - picker_arrival
                    //   positive → picker waits for dropper (constrained by W)
                    //   negative → dropper arrived first, load waits (unconstrained)
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
                        } catch (Exception e) {
                            if (syncGaps.length() > 0) syncGaps.append(";");
                            syncGaps.append("ERR");
                        }
                    }
                    String syncGapStr = syncGaps.length() > 0 ? syncGaps.toString() : "";

                    // Write CSV row
                    csv.printf("%d,%s,%s,%d,%d,%.4f,%d,%d,%.2f,%s,%d,%d,%.0f,%.0f,%.1f,%.0f,%d,%d,%d,%d,\"%s\",%d,%s,%d,%.1f%n",
                            ci, config.name, config.parameterName,
                            config.maxIterations, config.segmentLength, config.coolingRate,
                            config.betaMin, config.betaMax, config.reactionFactor,
                            fileName,
                            inst.getNumCustomers(), inst.getNumVehicles(),
                            inst.getMaxCapacity(), inst.getMaxRouteDuration(), syncWindow,
                            best.getTotalProfit(), best.getNumServed(), best.getTransfers().size(),
                            transferA, transferB,
                            syncGapStr,
                            elapsed, best.isFeasible() ? "TRUE" : "FALSE",
                            alns.getNewBestCount(), alns.getAcceptanceRate());
                    csv.flush();

                } catch (Exception e) {
                    System.setOut(originalOut);
                    originalOut.printf("    ERROR on %s: %s%n", fileName, e.getMessage());

                    // Write error row
                    csv.printf("%d,%s,%s,%d,%d,%.4f,%d,%d,%.2f,%s,,,,,,,,,,,\"\",,FALSE,,%n",
                            ci, config.name, config.parameterName,
                            config.maxIterations, config.segmentLength, config.coolingRate,
                            config.betaMin, config.betaMax, config.reactionFactor,
                            fileName);
                    csv.flush();
                }
            }

            long configTime = System.currentTimeMillis() - configStart;
            originalOut.printf("%n    Config %s done in %.1fs%n%n",
                    config.name, configTime / 1000.0);
        }

        csv.close();
        long totalTime = System.currentTimeMillis() - experimentStart;

        originalOut.println();
        originalOut.println("════════════════════════════════════════════════════════════");
        originalOut.printf( "  PARAMETER TUNING COMPLETE%n");
        originalOut.printf( "  Total runs:  %d%n", runCount);
        originalOut.printf( "  Total time:  %.1f seconds (%.1f min)%n",
                totalTime / 1000.0, totalTime / 60000.0);
        originalOut.printf( "  Results:     %s%n", outputPath);
        originalOut.println("════════════════════════════════════════════════════════════");
    }
}