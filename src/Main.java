import java.util.Arrays;

/**
 * CTOP-T-Sync: Batch Experiment Runner
 *
 * Multi-start ALNS with tuned parameters from two-stage DOE:
 *   Stage 1 (OFAT):  maxIterations=10000 (resource param, monotonic improvement)
 *   Stage 2 (3x3):   coolingRate=0.9995, betaMax=6 (robust, not significant)
 *
 * Best-of-3 seeds strategy: runs ALNS 3 times per instance with different
 * random seeds and reports the best feasible solution.
 */
public class Main {

    public static void main(String[] args) throws Exception {

        String instanceFolder = "C:/Users/beste/Desktop/THESIS/VRP-T/Gurobi/Datasets/all";
        String outputCsv      = "experiment_results_restart.csv";
        int    alnsIterations  = 10000;
        double syncWindow      = 15.0;
        long[] seeds           = {16, 42L, 123L, 7L, 15};

        if (args.length >= 1) instanceFolder = args[0];
        if (args.length >= 2) outputCsv      = args[1];
        if (args.length >= 3) alnsIterations  = Integer.parseInt(args[2]);
        if (args.length >= 4) syncWindow      = Double.parseDouble(args[3]);

        System.out.println("╔══════════════════════════════════════════════════════════╗");
        System.out.println("║  CTOP-T-Sync: Batch Experiment (directional sync)       ║");
        System.out.println("╠══════════════════════════════════════════════════════════╣");
        System.out.printf( "║  Instance folder:  %s%n", instanceFolder);
        System.out.printf( "║  ALNS iterations:  %d%n", alnsIterations);
        System.out.printf( "║  Cooling rate:     0.9995 (tuned)%n");
        System.out.printf( "║  Beta max:         6 (default, robust)%n");
        System.out.printf( "║  Multi-start:      best-of-%d seeds %s%n", seeds.length, Arrays.toString(seeds));
        System.out.printf( "║  Sync window (W):  %.1f (directional: giver-receiver ≤ W)%n", syncWindow);
        System.out.println("╚══════════════════════════════════════════════════════════╝\n");

        long startTime = System.currentTimeMillis();
        ExperimentRunner.run(instanceFolder, outputCsv, alnsIterations, syncWindow, seeds);
        long totalTime = System.currentTimeMillis() - startTime;

        System.out.printf("%nTotal experiment time: %.1f seconds%n", totalTime / 1000.0);
    }
}