import java.util.*;

/**
 * Adaptive Large Neighborhood Search (ALNS) engine for CTOP-T-Sync.
 *
 * Framework adapted from Hammami et al. (2024) HALNS for CTOP:
 *   - Multiple destroy/repair operators with adaptive weight selection
 *   - Simulated Annealing acceptance criterion
 *   - Periodic local search on promising solutions
 *   - Transfer optimization integrated into the improvement loop
 *
 * Algorithm outline:
 *   1. Construct initial solution
 *   2. Repeat for maxIterations:
 *      a. Select destroy and repair operators (roulette wheel on adaptive weights)
 *      b. Destroy: remove β nodes from current solution
 *      c. Repair: reinsert nodes
 *      d. Accept/reject via SA criterion
 *      e. If new best → apply local search + transfer optimization
 *      f. Update operator weights based on performance
 *   3. Return best solution found
 */
public class ALNSEngine {

    // ── ALNS Parameters ──
    private final int maxIterations;
    private final int segmentLength;       // iterations per segment (for weight updates)
    private final double initTemperature;
    private final double coolingRate;
    private final double minTemperature;
    private final int betaMin, betaMax;    // range for number of nodes to remove

    // ── Adaptive weight parameters (Ropke & Pisinger, 2006) ──
    private final double sigma1 = 33;  // reward: new global best found
    private final double sigma2 = 9;   // reward: better than current (accepted)
    private final double sigma3 = 3;   // reward: accepted but not improving
    private final double reactionFactor; // λ: how fast weights adapt (0.8 recommended)

    // ── Components ──
    private final DestroyOperators destroyOps;
    private final RepairOperators repairOps;
    private final LocalSearch localSearch;
    private final Random rng;

    // ── Adaptive weights ──
    private double[] destroyWeights;
    private double[] destroyScores;
    private int[] destroyUsageCounts;

    private double[] repairWeights;
    private double[] repairScores;
    private int[] repairUsageCounts;

    // ── Statistics ──
    private int totalIterations;
    private int acceptedCount;
    private int newBestCount;
    private long elapsedMs;

    // ══════════════════════════════════════════════════════════
    // CONSTRUCTORS
    // ══════════════════════════════════════════════════════════

    /**
     * Full constructor with all parameters.
     */
    public ALNSEngine(int maxIterations, int segmentLength,
                      double initTemperature, double coolingRate, double minTemperature,
                      int betaMin, int betaMax, double reactionFactor,
                      long seed) {
        this.maxIterations = maxIterations;
        this.segmentLength = segmentLength;
        this.initTemperature = initTemperature;
        this.coolingRate = coolingRate;
        this.minTemperature = minTemperature;
        this.betaMin = betaMin;
        this.betaMax = betaMax;
        this.reactionFactor = reactionFactor;

        this.rng = new Random(seed);
        this.destroyOps = new DestroyOperators(rng);
        this.repairOps = new RepairOperators(rng);
        this.localSearch = new LocalSearch();

        initWeights();
    }

    /**
     * Convenience constructor with sensible defaults.
     * Parameters calibrated based on Hammami (2024) sensitivity analysis.
     */
    public ALNSEngine(int maxIterations, long seed) {
        this(maxIterations,
                100,          // segment length
                50.0,         // initial temperature
                0.9997,       // cooling rate (slow cooling per Hammami: c=0.9997-0.9999)
                0.01,         // min temperature
                2,            // beta min
                6,            // beta max (up to 20% of served nodes)
                0.8,          // reaction factor λ
                seed);
    }

    /** Default: 5000 iterations */
    public ALNSEngine(long seed) {
        this(5000, seed);
    }

    // ══════════════════════════════════════════════════════════
    // MAIN SOLVE METHOD
    // ══════════════════════════════════════════════════════════

    /**
     * Runs the full ALNS procedure on the given instance.
     */
    public Solution solve(Instance instance) {
        long startTime = System.currentTimeMillis();

        // ── Phase 1: Construct initial solution ──
        GreedyConstructive constructor = new GreedyConstructive();
        Solution currentSol = constructor.construct(instance);
        localSearch.improve(currentSol);
        localSearch.postInsert(currentSol);

        Solution bestSol = new Solution(currentSol);
        Solution admissibleSol = new Solution(currentSol);
        double bestProfit = bestSol.getTotalProfit();

        // Transfer solution pool — filled only by TPO-MIP after ALNS completes
        Solution bestTransferSol = null;
        double bestTransferProfit = 0;

        System.out.printf("[ALNS] Initial: profit=%.0f, served=%d/%d%n",
                bestProfit, bestSol.getNumServed(), bestSol.getNumCustomers());

        // ── Auto-calibrate initial temperature ──
        // Temperature should accept a ~5% worsening move with ~50% probability
        // P = exp(-delta/T) = 0.5  →  T = -delta / ln(0.5)
        // delta = 5% of current profit
        double calibDelta = Math.max(bestProfit * 0.05, 1.0);
        double autoTemp = -calibDelta / Math.log(0.5);
        double temperature = Math.max(autoTemp, 1.0);
        double startTemp = temperature;

        System.out.printf("[ALNS] Auto-calibrated T0=%.2f (for delta=%.1f)%n", temperature, calibDelta);

        // ── Phase 2: ALNS iterations ──
        int iterWithoutImprovement = 0;
        int maxNoImprove = maxIterations / 3;

        acceptedCount = 0;
        newBestCount = 0;

        for (int iter = 0; iter < maxIterations; iter++) {

            // Adaptive β: scale with solution size
            int currentServed = admissibleSol.getNumServed();
            int adjustedBetaMax = Math.max(betaMax, (int)(0.25 * currentServed));
            int beta = betaMin + rng.nextInt(Math.max(1, adjustedBetaMax - betaMin + 1));

            // Select operators (adaptive roulette wheel)
            int dOp = selectOperator(destroyWeights);
            int rOp = selectOperator(repairWeights);

            // Copy current admissible solution
            Solution candidate = new Solution(admissibleSol);

            // Destroy
            List<Node> removed = destroyOps.apply(dOp, candidate, beta);

            // Repair
            int inserted = repairOps.apply(rOp, candidate, removed);

            // Clean up any stale transfers from the destroy/repair cycle
            candidate.cleanupStaleTransfers();

            // Evaluate
            for (Route r : candidate.getRoutes()) r.evaluate();

            if (!candidate.isFeasible()) {
                destroyUsageCounts[dOp]++;
                repairUsageCounts[rOp]++;
                updateWeightsIfSegmentEnd(iter);
                temperature = Math.max(temperature * coolingRate, minTemperature);
                continue;
            }

            double candProfit = candidate.getTotalProfit();
            double admProfit = admissibleSol.getTotalProfit();

            // ── SA Acceptance Criterion ──
            double delta = candProfit - admProfit;
            boolean accepted = false;

            if (delta > 0) {
                accepted = true;
            } else if (temperature > minTemperature) {
                double probability = Math.exp(delta / temperature);
                accepted = rng.nextDouble() < probability;
            }

            // ── Update scores and solution ──
            destroyUsageCounts[dOp]++;
            repairUsageCounts[rOp]++;

            if (accepted) {
                acceptedCount++;
                admissibleSol = candidate;

                if (candProfit > admProfit) {
                    // Better than admissible → apply local search
                    destroyScores[dOp] += sigma2;
                    repairScores[rOp] += sigma2;

                    localSearch.improve(admissibleSol);
                    localSearch.postInsert(admissibleSol);
                    candProfit = admissibleSol.getTotalProfit();
                } else {
                    destroyScores[dOp] += sigma3;
                    repairScores[rOp] += sigma3;
                }

                // Check if new global best
                if (candProfit > bestProfit) {
                    bestSol = new Solution(admissibleSol);
                    bestProfit = candProfit;
                    newBestCount++;
                    iterWithoutImprovement = 0;

                    destroyScores[dOp] += (sigma1 - sigma2);
                    repairScores[rOp] += (sigma1 - sigma2);

                    System.out.printf("[ALNS] iter %5d | NEW BEST profit=%.0f served=%d | " +
                                    "destroy=%s repair=%s β=%d T=%.2f%n",
                            iter, bestProfit, bestSol.getNumServed(),
                            destroyOps.getOperatorName(dOp),
                            repairOps.getOperatorName(rOp),
                            beta, temperature);
                } else {
                    iterWithoutImprovement++;
                }
            } else {
                iterWithoutImprovement++;
            }

            // ── Temperature cooling ──
            temperature = Math.max(temperature * coolingRate, minTemperature);

            // ── Weight update at segment boundaries ──
            updateWeightsIfSegmentEnd(iter);

            // ── Restart mechanism (if stuck) ──
            if (iterWithoutImprovement >= maxNoImprove) {
                admissibleSol = new Solution(bestSol);
                temperature = startTemp * 0.5; // reheat to half
                iterWithoutImprovement = 0;
            }
        }

        // ── Phase 3: Final polish ──
        localSearch.improve(bestSol);
        localSearch.postInsert(bestSol);

        // ── Phase 4: Transfer Post-Optimization MIP ──
        System.out.println("\n[ALNS] Phase 4: Transfer Post-Optimization MIP");
        Solution mipTransferSol = new Solution(bestSol);
        try {
            TransferMIP transferMIP = new TransferMIP();
            int mipNewCustomers = transferMIP.optimize(mipTransferSol);

            // Debug: print feasibility status
            Solution.FeasibilityReport fr = mipTransferSol.checkFeasibility();
            double mipProfit = mipTransferSol.getTotalProfit();
            System.out.printf("[ALNS] TPO-MIP result: profit=%.0f (was %.0f), feasible=%s, transfers=%d%n",
                    mipProfit, bestSol.getTotalProfit(), fr.isFeasible(), mipTransferSol.getTransfers().size());
            if (!fr.isFeasible()) {
                System.out.println("[ALNS] TPO-MIP infeasibility reasons:");
                for (String v : fr.getViolations())
                    System.out.println("  → " + v);
            }

            if (fr.isFeasible() && mipProfit > bestSol.getTotalProfit()) {
                // DO NOT run LocalSearch — it destroys transfer structures
                bestTransferSol = new Solution(mipTransferSol);
                bestTransferProfit = mipProfit;
                System.out.printf("[ALNS] TPO-MIP POOL: profit=%.0f transfers=%d customers_added=%d%n",
                        bestTransferProfit, bestTransferSol.getTransfers().size(), mipNewCustomers);
            } else if (!fr.isFeasible()) {
                System.out.println("[ALNS] TPO-MIP: solution infeasible after application");
            } else {
                System.out.println("[ALNS] TPO-MIP: no profit improvement");
            }
        } catch (Exception e) {
            System.out.println("[ALNS] TPO-MIP failed: " + e.getMessage());
            e.printStackTrace();
        }

        // ── Phase 5: Pool comparison — return the best overall ──
        if (!bestSol.isFeasible()) {
            bestSol.getTransfers().clear();
            bestSol.cleanupStaleTransfers();
            for (Route r : bestSol.getRoutes()) r.evaluate();
        }

        Solution winner = bestSol;

        if (bestTransferSol != null && bestTransferSol.isFeasible()) {
            if (bestTransferSol.getTotalProfit() > winner.getTotalProfit()) {
                winner = bestTransferSol;
                System.out.printf("[ALNS] TRANSFER solution wins: profit=%.0f > no-transfer=%.0f (transfers=%d)%n",
                        bestTransferSol.getTotalProfit(), bestSol.getTotalProfit(),
                        bestTransferSol.getTransfers().size());
            } else if (bestTransferSol.getTotalProfit() == winner.getTotalProfit()
                    && !bestTransferSol.getTransfers().isEmpty()) {
                winner = bestTransferSol;
                System.out.printf("[ALNS] TRANSFER solution selected (equal profit=%.0f, transfers=%d)%n",
                        bestTransferSol.getTotalProfit(), bestTransferSol.getTransfers().size());
            } else {
                System.out.printf("[ALNS] No-transfer solution wins: profit=%.0f ≥ transfer=%.0f%n",
                        winner.getTotalProfit(),
                        bestTransferSol != null ? bestTransferSol.getTotalProfit() : 0);
            }
        } else {
            System.out.println("[ALNS] No feasible transfer solution found");
        }

        totalIterations = maxIterations;
        elapsedMs = System.currentTimeMillis() - startTime;

        return winner;
    }

    // ══════════════════════════════════════════════════════════
    // ADAPTIVE WEIGHT MECHANISM
    // ══════════════════════════════════════════════════════════

    private void initWeights() {
        int nDestroy = DestroyOperators.NUM_OPERATORS;
        int nRepair = RepairOperators.NUM_OPERATORS;

        destroyWeights = new double[nDestroy];
        destroyScores = new double[nDestroy];
        destroyUsageCounts = new int[nDestroy];

        repairWeights = new double[nRepair];
        repairScores = new double[nRepair];
        repairUsageCounts = new int[nRepair];

        // Initialize all weights equally
        Arrays.fill(destroyWeights, 1.0);
        Arrays.fill(repairWeights, 1.0);
    }

    /**
     * Roulette wheel selection based on operator weights.
     */
    private int selectOperator(double[] weights) {
        double total = 0;
        for (double w : weights) total += w;
        if (total <= 0) return rng.nextInt(weights.length);

        double r = rng.nextDouble() * total;
        double cumulative = 0;
        for (int i = 0; i < weights.length; i++) {
            cumulative += weights[i];
            if (r <= cumulative) return i;
        }
        return weights.length - 1;
    }

    /**
     * Updates operator weights at segment boundaries.
     *
     * New weight = λ × old_weight + (1-λ) × (score / usage_count)
     *
     * Following Ropke & Pisinger (2006) adaptive mechanism.
     */
    private void updateWeightsIfSegmentEnd(int iter) {
        if ((iter + 1) % segmentLength != 0) return;

        // Update destroy weights
        for (int i = 0; i < destroyWeights.length; i++) {
            if (destroyUsageCounts[i] > 0) {
                double avgScore = destroyScores[i] / destroyUsageCounts[i];
                destroyWeights[i] = reactionFactor * destroyWeights[i]
                        + (1 - reactionFactor) * avgScore;
            }
            // Ensure minimum weight to avoid starvation
            destroyWeights[i] = Math.max(destroyWeights[i], 0.1);
            // Reset scores and counts for next segment
            destroyScores[i] = 0;
            destroyUsageCounts[i] = 0;
        }

        // Update repair weights
        for (int i = 0; i < repairWeights.length; i++) {
            if (repairUsageCounts[i] > 0) {
                double avgScore = repairScores[i] / repairUsageCounts[i];
                repairWeights[i] = reactionFactor * repairWeights[i]
                        + (1 - reactionFactor) * avgScore;
            }
            repairWeights[i] = Math.max(repairWeights[i], 0.1);
            repairScores[i] = 0;
            repairUsageCounts[i] = 0;
        }
    }

    // ══════════════════════════════════════════════════════════
    // REPORTING
    // ══════════════════════════════════════════════════════════

    /**
     * Prints a summary report of the ALNS run.
     */
    public void printReport(Solution bestSol) {
        System.out.println("\n┌──────────────────────────────────────────────┐");
        System.out.println("│             ALNS Run Report                  │");
        System.out.println("├──────────────────────────────────────────────┤");
        System.out.printf( "│  Iterations:       %,8d                 │%n", totalIterations);
        System.out.printf( "│  Accepted moves:   %,8d (%4.1f%%)          │%n",
                acceptedCount, 100.0 * acceptedCount / Math.max(1, totalIterations));
        System.out.printf( "│  New best found:   %,8d                 │%n", newBestCount);
        System.out.printf( "│  Elapsed time:     %,8d ms               │%n", elapsedMs);
        System.out.printf( "│  Best profit:      %,8.0f                 │%n", bestSol.getTotalProfit());
        System.out.printf( "│  Customers served: %4d / %-4d               │%n",
                bestSol.getNumServed(), bestSol.getNumCustomers());
        System.out.printf( "│  Transfers:        %4d                      │%n",
                bestSol.getTransfers().size());
        System.out.println("├──────────────────────────────────────────────┤");
        System.out.println("│  Final Operator Weights:                    │");

        System.out.print("│    Destroy: ");
        for (int i = 0; i < destroyWeights.length; i++) {
            System.out.printf("%s=%.1f ", destroyOps.getOperatorName(i).substring(0, Math.min(4, destroyOps.getOperatorName(i).length())), destroyWeights[i]);
        }
        System.out.println("│");

        System.out.print("│    Repair:  ");
        for (int i = 0; i < repairWeights.length; i++) {
            System.out.printf("%s=%.1f ", repairOps.getOperatorName(i).substring(0, Math.min(4, repairOps.getOperatorName(i).length())), repairWeights[i]);
        }
        System.out.println("│");

        System.out.println("└──────────────────────────────────────────────┘");
    }

    // ══════════════════════════════════════════════════════════
    // STATISTICS GETTERS
    // ══════════════════════════════════════════════════════════

    public int getTotalIterations()  { return totalIterations; }
    public int getAcceptedCount()    { return acceptedCount; }
    public int getNewBestCount()     { return newBestCount; }
    public long getElapsedMs()       { return elapsedMs; }

    public double getAcceptanceRate() {
        return totalIterations > 0 ? 100.0 * acceptedCount / totalIterations : 0;
    }
}