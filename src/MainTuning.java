/**
 * CTOP-T-Sync: Parameter Tuning Experiment Runner
 *
 * Runs all instances against multiple parameter configurations (OFAT design).
 * Results written to CSV (openable in Excel).
 *
 * Usage:
 *   java MainTuning [instanceFolder] [outputCsv] [syncWindow] [seed]
 *
 * Default:
 *   java MainTuning "path/to/instances" "tuning_results.csv" 15.0 42
 */
//public class MainTuning {
//
//    public static void main(String[] args) throws Exception {
//
//        String instanceFolder = "C:/Users/beste/Desktop/THESIS/VRP-T/Gurobi/Datasets/all";
//        String outputCsv      = "tuning_results.csv";
//        double syncWindow      = 15.0;
//        long   seed            = 42L;
//
//        if (args.length >= 1) instanceFolder = args[0];
//        if (args.length >= 2) outputCsv      = args[1];
//        if (args.length >= 3) syncWindow      = Double.parseDouble(args[2]);
//        if (args.length >= 4) seed            = Long.parseLong(args[3]);
//
//        ParameterTuningRunner.run(instanceFolder, outputCsv, syncWindow, seed);
//    }
//}

/**
 * CTOP-T-Sync: Stage 2 Factorial Experiment Runner
 *
 * Stage 1 (OFAT) identified coolingRate and betaMax as tunable parameters.
 * maxIterations fixed at 10000 (resource parameter, monotonic improvement).
 *
 * Design: 3x3 full factorial (coolingRate x betaMax) x 3 seeds
 *   Factor A: coolingRate  {0.9990, 0.9995, 0.9999}
 *   Factor B: betaMax      {4, 6, 10}
 *   Seeds:    {42, 123, 7}
 *
 * Usage:
 *   java MainTuning [instanceFolder] [outputCsv]
 */
public class MainTuning {

    public static void main(String[] args) throws Exception {

        // Point to folder with small instances (n=50, 75 only)
        String instanceFolder = "C:/Users/beste/Desktop/THESIS/VRP-T/Gurobi/Datasets/all";
        String outputCsv      = "stage2_factorial_results.csv";
        double syncWindow      = 15.0;

        // Three seeds for replication (statistical significance)
        long[] seeds = {42L, 123L, 7L};

        if (args.length >= 1) instanceFolder = args[0];
        if (args.length >= 2) outputCsv      = args[1];
        if (args.length >= 3) syncWindow      = Double.parseDouble(args[2]);

        ParameterTuningRunner.run(instanceFolder, outputCsv, syncWindow, seeds);
    }
}