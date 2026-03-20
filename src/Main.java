/**
 * CTOP-T-Sync: Batch Experiment Runner
 *
 * Sync constraint (17) is now DIRECTIONAL:
 *   u_j,giver - u_j,receiver ≤ W
 *   Giver can arrive before receiver (goods wait at node).
 *   Receiver should not wait more than W for giver.
 */
public class Main {

    public static void main(String[] args) throws Exception {

        String instanceFolder = "C:/Users/beste/Desktop/THESIS/VRP-T/Gurobi/Datasets/archetti_original_2/sub";
        String outputCsv      = "experiment_results.csv";
        int    alnsIterations  = 5000;
        double syncWindow      = 15.0;
        long   seed            = 42L;

        if (args.length >= 1) instanceFolder = args[0];
        if (args.length >= 2) outputCsv      = args[1];
        if (args.length >= 3) alnsIterations  = Integer.parseInt(args[2]);
        if (args.length >= 4) syncWindow      = Double.parseDouble(args[3]);
        if (args.length >= 5) seed            = Long.parseLong(args[4]);

        System.out.println("╔══════════════════════════════════════════════════════════╗");
        System.out.println("║  CTOP-T-Sync: Batch Experiment (directional sync)       ║");
        System.out.println("╠══════════════════════════════════════════════════════════╣");
        System.out.printf( "║  Instance folder:  %s%n", instanceFolder);
        System.out.printf( "║  ALNS iterations:  %d%n", alnsIterations);
        System.out.printf( "║  Sync window (W):  %.1f (directional: giver-receiver ≤ W)%n", syncWindow);
        System.out.printf( "║  Random seed:      %d%n", seed);
        System.out.println("╚══════════════════════════════════════════════════════════╝\n");

        long startTime = System.currentTimeMillis();
        ExperimentRunner.run(instanceFolder, outputCsv, alnsIterations, syncWindow, seed);
        long totalTime = System.currentTimeMillis() - startTime;

        System.out.printf("%nTotal experiment time: %.1f seconds%n", totalTime / 1000.0);
    }
}