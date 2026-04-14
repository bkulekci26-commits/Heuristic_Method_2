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
public class MainTuning {

    public static void main(String[] args) throws Exception {

        String instanceFolder = "C:/Users/beste/Desktop/THESIS/VRP-T/Gurobi/Datasets/all";
        String outputCsv      = "tuning_results.csv";
        double syncWindow      = 15.0;
        long   seed            = 42L;

        if (args.length >= 1) instanceFolder = args[0];
        if (args.length >= 2) outputCsv      = args[1];
        if (args.length >= 3) syncWindow      = Double.parseDouble(args[2]);
        if (args.length >= 4) seed            = Long.parseLong(args[3]);

        ParameterTuningRunner.run(instanceFolder, outputCsv, syncWindow, seed);
    }
}