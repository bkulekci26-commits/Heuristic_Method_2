import java.util.*;

public class ALNSEngine {

    private final int maxIterations;
    private final int segmentLength;
    private final double initTemperature;
    private final double coolingRate;
    private final double minTemperature;
    private final int betaMin, betaMax;
    private final double sigma1 = 33;
    private final double sigma2 = 9;
    private final double sigma3 = 3;
    private final double reactionFactor;

    private final DestroyOperators destroyOps;
    private final RepairOperators repairOps;
    private final LocalSearch localSearch;
    private final TransformOperator transformOperator;
    private final Random rng;

    private double[] destroyWeights, destroyScores;
    private int[] destroyUsageCounts;
    private double[] repairWeights, repairScores;
    private int[] repairUsageCounts;

    private int totalIterations, acceptedCount, newBestCount;
    private long elapsedMs;

    public ALNSEngine(int maxIterations, int segmentLength, double initTemperature, double coolingRate, double minTemperature, int betaMin, int betaMax, double reactionFactor, long seed) {
        this.maxIterations = maxIterations; this.segmentLength = segmentLength; this.initTemperature = initTemperature;
        this.coolingRate = coolingRate; this.minTemperature = minTemperature; this.betaMin = betaMin; this.betaMax = betaMax; this.reactionFactor = reactionFactor;
        this.rng = new Random(seed); this.destroyOps = new DestroyOperators(rng); this.repairOps = new RepairOperators(rng);
        this.localSearch = new LocalSearch(); this.transformOperator = new TransformOperator();
        initWeights();
    }

    public ALNSEngine(int maxIterations, long seed) { this(maxIterations, 100, 50.0, 0.9997, 0.01, 2, 6, 0.8, seed); }

    public Solution solve(Instance instance) {
        long startTime = System.currentTimeMillis();

        GreedyConstructive constructor = new GreedyConstructive();
        Solution currentSol = constructor.construct(instance);
        localSearch.improve(currentSol);
        localSearch.postInsert(currentSol);

        Solution bestSol = new Solution(currentSol);
        Solution admissibleSol = new Solution(currentSol);
        double bestProfit = bestSol.getTotalProfit();
        Solution bestTransferSol = null;
        double bestTransferProfit = 0;

        double calibDelta = Math.max(bestProfit * 0.05, 1.0);
        double temperature = Math.max(-calibDelta / Math.log(0.5), 1.0);
        double startTemp = temperature;

        int iterWithoutImprovement = 0;
        int maxNoImprove = maxIterations / 3;
        int transferInterval = Math.max(200, maxIterations / 25);
        acceptedCount = 0; newBestCount = 0;

        for (int iter = 0; iter < maxIterations; iter++) {
            int adjustedBetaMax = Math.max(betaMax, (int)(0.25 * admissibleSol.getNumServed()));
            int beta = betaMin + rng.nextInt(Math.max(1, adjustedBetaMax - betaMin + 1));

            int dOp = selectOperator(destroyWeights);
            int rOp = selectOperator(repairWeights);

            Solution candidate = new Solution(admissibleSol);
            List<Node> removed = destroyOps.apply(dOp, candidate, beta);
            repairOps.apply(rOp, candidate, removed);

            candidate.cleanupFailedSplits();
            candidate.cleanupStaleTransfers();
            for (Route r : candidate.getRoutes()) r.evaluate();

            if (!candidate.isFeasible()) {
                destroyUsageCounts[dOp]++; repairUsageCounts[rOp]++;
                updateWeightsIfSegmentEnd(iter);
                temperature = Math.max(temperature * coolingRate, minTemperature);
                continue;
            }

            double candProfit = candidate.getTotalProfit();
            double admProfit = admissibleSol.getTotalProfit();
            double delta = candProfit - admProfit;
            boolean accepted = delta > 0 || (temperature > minTemperature && rng.nextDouble() < Math.exp(delta / temperature));

            if (!accepted) {
                if (candidate.getTransfers().size() > admissibleSol.getTransfers().size()) {
                    accepted = true;
                } else if (candidate.getSplitCustomers().size() > admissibleSol.getSplitCustomers().size()) {
                    accepted = true;
                }
            }
            destroyUsageCounts[dOp]++; repairUsageCounts[rOp]++;

            if (accepted) {
                acceptedCount++;
                admissibleSol = candidate;

                if (candProfit > admProfit) {
                    destroyScores[dOp] += sigma2; repairScores[rOp] += sigma2;
                    localSearch.improve(admissibleSol);
                    localSearch.postInsert(admissibleSol);
                    candProfit = admissibleSol.getTotalProfit();
                } else {
                    destroyScores[dOp] += sigma3; repairScores[rOp] += sigma3;
                }

                if (candProfit > bestProfit) {
                    Solution preTransfer = new Solution(admissibleSol);
                    transformOperator.optimize(admissibleSol);
                    localSearch.postInsert(admissibleSol);
                    if (!admissibleSol.isFeasible()) admissibleSol = preTransfer;
                    candProfit = admissibleSol.getTotalProfit();

                    bestSol = new Solution(admissibleSol);
                    bestProfit = candProfit;
                    newBestCount++; iterWithoutImprovement = 0;
                    destroyScores[dOp] += (sigma1 - sigma2); repairScores[rOp] += (sigma1 - sigma2);

                    if (!admissibleSol.getTransfers().isEmpty() && admissibleSol.isFeasible() && admissibleSol.getTotalProfit() > bestTransferProfit) {
                        bestTransferSol = new Solution(admissibleSol);
                        bestTransferProfit = admissibleSol.getTotalProfit();
                    }
                } else { iterWithoutImprovement++; }
            } else { iterWithoutImprovement++; }

            if (iter > 0 && iter % transferInterval == 0) {
                Solution transferTest = new Solution(admissibleSol);
                if (transformOperator.optimize(transferTest) > 0 && transferTest.isFeasible()) {
                    localSearch.postInsert(transferTest);
                    localSearch.improve(transferTest);
                    localSearch.postInsert(transferTest);
                    if (transferTest.isFeasible()) {
                        double tProfit = transferTest.getTotalProfit();
                        if (!transferTest.getTransfers().isEmpty() && tProfit > bestTransferProfit) {
                            bestTransferSol = new Solution(transferTest);
                            bestTransferProfit = tProfit;
                        }
                        if (tProfit > admissibleSol.getTotalProfit()) {
                            admissibleSol = transferTest;
                            if (tProfit > bestProfit) {
                                bestSol = new Solution(transferTest);
                                bestProfit = tProfit;
                                newBestCount++; iterWithoutImprovement = 0;
                            }
                        }
                    }
                }
            }

            temperature = Math.max(temperature * coolingRate, minTemperature);
            updateWeightsIfSegmentEnd(iter);

            if (iterWithoutImprovement >= maxNoImprove) {
                admissibleSol = new Solution(bestSol);
                temperature = startTemp * 0.5;
                iterWithoutImprovement = 0;
            }
        }

        localSearch.improve(bestSol);
        localSearch.postInsert(bestSol);

        Solution transformBest = new Solution(bestSol);
        if (transformOperator.optimize(transformBest) > 0 && transformBest.isFeasible()) {
            localSearch.postInsert(transformBest);
            if (transformBest.isFeasible() && !transformBest.getTransfers().isEmpty() && transformBest.getTotalProfit() > bestTransferProfit) {
                bestTransferSol = new Solution(transformBest);
                bestTransferProfit = transformBest.getTotalProfit();
            }
        }

        if (!bestSol.isFeasible()) {
            bestSol.getTransfers().clear(); bestSol.cleanupStaleTransfers();
            for (Route r : bestSol.getRoutes()) r.evaluate();
        }

        Solution winner = bestSol;
        if (bestTransferSol != null && bestTransferSol.isFeasible() && bestTransferSol.getTotalProfit() >= winner.getTotalProfit()) {
            winner = bestTransferSol;
        }

        totalIterations = maxIterations; elapsedMs = System.currentTimeMillis() - startTime;
        return winner;
    }

    private void initWeights() {
        int nDestroy = DestroyOperators.NUM_OPERATORS; int nRepair = RepairOperators.NUM_OPERATORS;
        destroyWeights = new double[nDestroy]; destroyScores = new double[nDestroy]; destroyUsageCounts = new int[nDestroy];
        repairWeights = new double[nRepair]; repairScores = new double[nRepair]; repairUsageCounts = new int[nRepair];
        Arrays.fill(destroyWeights, 1.0); Arrays.fill(repairWeights, 1.0);
    }

    private int selectOperator(double[] weights) {
        double total = 0; for (double w : weights) total += w;
        if (total <= 0) return rng.nextInt(weights.length);
        double r = rng.nextDouble() * total, cumulative = 0;
        for (int i = 0; i < weights.length; i++) { cumulative += weights[i]; if (r <= cumulative) return i; }
        return weights.length - 1;
    }

    private void updateWeightsIfSegmentEnd(int iter) {
        if ((iter + 1) % segmentLength != 0) return;
        for (int i = 0; i < destroyWeights.length; i++) {
            if (destroyUsageCounts[i] > 0) destroyWeights[i] = Math.max(0.1, reactionFactor * destroyWeights[i] + (1 - reactionFactor) * (destroyScores[i] / destroyUsageCounts[i]));
            destroyScores[i] = 0; destroyUsageCounts[i] = 0;
        }
        for (int i = 0; i < repairWeights.length; i++) {
            if (repairUsageCounts[i] > 0) repairWeights[i] = Math.max(0.1, reactionFactor * repairWeights[i] + (1 - reactionFactor) * (repairScores[i] / repairUsageCounts[i]));
            repairScores[i] = 0; repairUsageCounts[i] = 0;
        }
    }

    public void printReport(Solution bestSol) { } // Keeping omitted for brevity (doesn't span console)

    public int getTotalIterations()  { return totalIterations; }
    public int getAcceptedCount()    { return acceptedCount; }
    public int getNewBestCount()     { return newBestCount; }
    public long getElapsedMs()       { return elapsedMs; }
    public double getAcceptanceRate() { return totalIterations > 0 ? 100.0 * acceptedCount / totalIterations : 0; }
}