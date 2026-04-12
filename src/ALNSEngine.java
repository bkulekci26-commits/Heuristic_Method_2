import java.util.*;

/**
 * Adaptive Large Neighborhood Search (ALNS) engine for CTOP-T-Sync.
 *
 * Framework adapted from Hammami et al. (2024) HALNS for CTOP,
 * with transfer mechanism from Aguayo et al. (2025) VRP-T.
 *
 * Algorithm outline:
 *   Phase 1: Construct initial solution
 *   Phase 2: ALNS iterations (destroy/repair with SA acceptance)
 *     - On new best → TransformOperator (Aguayo-adapted transfers)
 *     - Periodic transfer attempts on admissible solution
 *   Phase 3: Final local search polish
 *   Phase 4: Transfer optimization (TransformOperator on best + admissible)
 *   Phase 5: Pool comparison → return best of {CTOP, CTOP-T-Sync}
 */
public class ALNSEngine {

    // ── ALNS Parameters ──
    private final int maxIterations;
    private final int segmentLength;
    private final double initTemperature;
    private final double coolingRate;
    private final double minTemperature;
    private final int betaMin, betaMax;

    // ── Adaptive weight parameters (Ropke & Pisinger, 2006) ──
    private final double sigma1 = 33;
    private final double sigma2 = 9;
    private final double sigma3 = 3;
    private final double reactionFactor;

    // ── Components ──
    private final DestroyOperators destroyOps;
    private final RepairOperators repairOps;
    private final LocalSearch localSearch;
    private final TransformOperator transformOperator;
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
        this.transformOperator = new TransformOperator();

        initWeights();
    }

    public ALNSEngine(int maxIterations, long seed) {
        this(maxIterations, 100, 50.0, 0.9997, 0.01, 2, 6, 0.8, seed);
    }

    public ALNSEngine(long seed) {
        this(5000, seed);
    }

    // ══════════════════════════════════════════════════════════
    // MAIN SOLVE METHOD
    // ══════════════════════════════════════════════════════════

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

        // Transfer solution pool
        Solution bestTransferSol = null;
        double bestTransferProfit = 0;

        System.out.printf("[ALNS] Initial: profit=%.0f, served=%d/%d%n",
                bestProfit, bestSol.getNumServed(), bestSol.getNumCustomers());

        // Auto-calibrate initial temperature
        double calibDelta = Math.max(bestProfit * 0.05, 1.0);
        double autoTemp = -calibDelta / Math.log(0.5);
        double temperature = Math.max(autoTemp, 1.0);
        double startTemp = temperature;

        System.out.printf("[ALNS] Auto-calibrated T0=%.2f (for delta=%.1f)%n", temperature, calibDelta);

        // ── Phase 2: ALNS iterations ──
        int iterWithoutImprovement = 0;
        int maxNoImprove = maxIterations / 3;
        int transferInterval = Math.max(200, maxIterations / 25);

        acceptedCount = 0;
        newBestCount = 0;

        for (int iter = 0; iter < maxIterations; iter++) {

            int currentServed = admissibleSol.getNumServed();
            int adjustedBetaMax = Math.max(betaMax, (int)(0.25 * currentServed));
            int beta = betaMin + rng.nextInt(Math.max(1, adjustedBetaMax - betaMin + 1));

            int dOp = selectOperator(destroyWeights);
            int rOp = selectOperator(repairWeights);

            Solution candidate = new Solution(admissibleSol);

            // Destroy
            List<Node> removed = destroyOps.apply(dOp, candidate, beta);

            // Repair
            repairOps.apply(rOp, candidate, removed);

            // Clean up stale transfers
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

            // SA Acceptance
            double delta = candProfit - admProfit;
            boolean accepted = false;

            if (delta > 0) {
                accepted = true;
            } else if (temperature > minTemperature) {
                double probability = Math.exp(delta / temperature);
                accepted = rng.nextDouble() < probability;
            }

            destroyUsageCounts[dOp]++;
            repairUsageCounts[rOp]++;

            if (accepted) {
                acceptedCount++;
                admissibleSol = candidate;

                if (candProfit > admProfit) {
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
                    // Apply TransformOperator on new best
                    Solution preTransfer = new Solution(admissibleSol);
                    transformOperator.optimize(admissibleSol);
                    localSearch.postInsert(admissibleSol);

                    if (!admissibleSol.isFeasible()) {
                        admissibleSol = preTransfer;
                    }

                    candProfit = admissibleSol.getTotalProfit();

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

                    // Save to transfer pool if solution has transfers
                    if (!admissibleSol.getTransfers().isEmpty()
                            && admissibleSol.isFeasible()
                            && admissibleSol.getTotalProfit() > bestTransferProfit) {
                        bestTransferSol = new Solution(admissibleSol);
                        bestTransferProfit = admissibleSol.getTotalProfit();
                        System.out.printf("[ALNS] iter %5d | TRANSFER POOL updated: profit=%.0f transfers=%d%n",
                                iter, bestTransferProfit, bestTransferSol.getTransfers().size());
                    }
                } else {
                    iterWithoutImprovement++;
                }
            } else {
                iterWithoutImprovement++;
            }

            // ── Periodic transfer attempt on admissible solution ──
            if (iter > 0 && iter % transferInterval == 0) {
                Solution transferTest = new Solution(admissibleSol);
                int tCount = transformOperator.optimize(transferTest);
                if (tCount > 0 && transferTest.isFeasible()) {
                    localSearch.postInsert(transferTest);
                    localSearch.improve(transferTest);
                    localSearch.postInsert(transferTest);
                    if (transferTest.isFeasible()) {
                        double tProfit = transferTest.getTotalProfit();

                        if (!transferTest.getTransfers().isEmpty()
                                && tProfit > bestTransferProfit) {
                            bestTransferSol = new Solution(transferTest);
                            bestTransferProfit = tProfit;
                            System.out.printf("[ALNS] iter %5d | TRANSFER POOL updated: profit=%.0f transfers=%d (periodic)%n",
                                    iter, bestTransferProfit, bestTransferSol.getTransfers().size());
                        }

                        if (tProfit > admissibleSol.getTotalProfit()) {
                            admissibleSol = transferTest;
                            if (tProfit > bestProfit) {
                                bestSol = new Solution(transferTest);
                                bestProfit = tProfit;
                                newBestCount++;
                                iterWithoutImprovement = 0;
                                System.out.printf("[ALNS] iter %5d | NEW BEST profit=%.0f served=%d (periodic transform)%n",
                                        iter, bestProfit, bestSol.getNumServed());
                            }
                        }
                    }
                }
            }

            // ── Temperature cooling ──
            temperature = Math.max(temperature * coolingRate, minTemperature);

            // ── Weight update at segment boundaries ──
            updateWeightsIfSegmentEnd(iter);

            // ── Restart mechanism ──
            if (iterWithoutImprovement >= maxNoImprove) {
                admissibleSol = new Solution(bestSol);
                temperature = startTemp * 0.5;
                iterWithoutImprovement = 0;
            }
        }

        // ── Phase 3: Final polish ──
        localSearch.improve(bestSol);
        localSearch.postInsert(bestSol);

        // ── Phase 4: Transfer Optimization (Aguayo Transform) ──
        System.out.println("\n[ALNS] Phase 4: Transfer Optimization (Aguayo Transform)");

        // 4a: Transform on best solution
        Solution transformBest = new Solution(bestSol);
        int tCount = transformOperator.optimize(transformBest);
        if (tCount > 0 && transformBest.isFeasible()) {
            localSearch.postInsert(transformBest);
            if (transformBest.isFeasible() && !transformBest.getTransfers().isEmpty()
                    && transformBest.getTotalProfit() > bestTransferProfit) {
                bestTransferSol = new Solution(transformBest);
                bestTransferProfit = transformBest.getTotalProfit();
                System.out.printf("[ALNS] Transform on best: profit=%.0f transfers=%d%n",
                        bestTransferProfit, bestTransferSol.getTransfers().size());
            }
        }

        // 4b: Transform on admissible solution (if different from best)
        if (admissibleSol.getTotalProfit() != bestSol.getTotalProfit()) {
            Solution transformAdm = new Solution(admissibleSol);
            tCount = transformOperator.optimize(transformAdm);
            if (tCount > 0 && transformAdm.isFeasible()) {
                localSearch.postInsert(transformAdm);
                if (transformAdm.isFeasible() && !transformAdm.getTransfers().isEmpty()
                        && transformAdm.getTotalProfit() > bestTransferProfit) {
                    bestTransferSol = new Solution(transformAdm);
                    bestTransferProfit = transformAdm.getTotalProfit();
                    System.out.printf("[ALNS] Transform on admissible: profit=%.0f transfers=%d%n",
                            bestTransferProfit, bestTransferSol.getTransfers().size());
                }
            }
        }

        if (bestTransferSol == null || bestTransferProfit <= bestSol.getTotalProfit()) {
            System.out.println("[ALNS] No profitable transfer solution found");
        }

        // ── Phase 5: Pool comparison ──
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
                        bestTransferSol.getTotalProfit());
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

        Arrays.fill(destroyWeights, 1.0);
        Arrays.fill(repairWeights, 1.0);
    }

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

    private void updateWeightsIfSegmentEnd(int iter) {
        if ((iter + 1) % segmentLength != 0) return;

        for (int i = 0; i < destroyWeights.length; i++) {
            if (destroyUsageCounts[i] > 0) {
                double avgScore = destroyScores[i] / destroyUsageCounts[i];
                destroyWeights[i] = reactionFactor * destroyWeights[i]
                        + (1 - reactionFactor) * avgScore;
            }
            destroyWeights[i] = Math.max(destroyWeights[i], 0.1);
            destroyScores[i] = 0;
            destroyUsageCounts[i] = 0;
        }

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
            System.out.printf("%s=%.1f ",
                    destroyOps.getOperatorName(i).substring(0, Math.min(4, destroyOps.getOperatorName(i).length())),
                    destroyWeights[i]);
        }
        System.out.println("│");

        System.out.print("│    Repair:  ");
        for (int i = 0; i < repairWeights.length; i++) {
            System.out.printf("%s=%.1f ",
                    repairOps.getOperatorName(i).substring(0, Math.min(4, repairOps.getOperatorName(i).length())),
                    repairWeights[i]);
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