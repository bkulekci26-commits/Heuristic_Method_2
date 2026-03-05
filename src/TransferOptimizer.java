import java.util.*;

/**
 * Transfer Optimizer for CTOP-T-Sync (v2: Remove-Transfer-Insert).
 *
 * The fundamental challenge: after local search, routes are packed to capacity.
 * No vehicle has spare capacity to carry extra transfer load. Therefore, we must
 * CREATE capacity by removing a low-value node from the giver route.
 *
 * Combined move (Remove-Transfer-Insert):
 *   1. REMOVE a low-profit served node 's' from giver route k1
 *      → Frees capacity = demand(s)
 *   2. k1 uses freed capacity to carry transfer load to node 'j' (still in k1's route)
 *      k1 DROPS OFF transferQty at node j
 *   3. Receiver route k2 PICKS UP transferQty at node j (detour or existing visit)
 *   4. k2 uses the gained capacity to INSERT a high-profit unserved node 'u'
 *
 * Accept if: profit(u) - profit(s) - α × Δdistance > 0
 *
 * Synchronization: |arrivalTime_k1(j) - arrivalTime_k2(j)| ≤ W
 *
 * This mechanism is adapted from:
 *   - Aguayo et al. (2025): VRP-T Transform step (drop remaining capacity)
 *   - Cortes & Suzuki (2020): Mid-route consolidation at customer locations
 *
 * Novel element: Temporal synchronization constraint (17) from our MIP model.
 */
public class TransferOptimizer {

    private final double objectiveAlpha;

    public TransferOptimizer(double objectiveAlpha) {
        this.objectiveAlpha = objectiveAlpha;
    }

    // ══════════════════════════════════════════════════════════
    // MAIN ENTRY POINT
    // ══════════════════════════════════════════════════════════

    public int optimize(Solution solution) {
        int totalTransfers = 0;
        boolean improved = true;

        while (improved) {
            improved = false;
            TransferCandidate best = findBestRemoveTransferInsert(solution);

            if (best != null) {
                executeTransfer(solution, best);
                totalTransfers++;
                improved = true;

                System.out.printf("[Transfer] v%d removes node %d (p=%.0f,d=%.0f), " +
                                "drops %.0f at node %d → v%d picks up → inserts node %d (p=%.0f,d=%.0f) | gain=%.2f%n",
                        best.giverRouteIdx, best.removedNodeId,
                        best.removedProfit, best.removedDemand,
                        best.transferQty, best.transferNodeId,
                        best.receiverRouteIdx, best.newCustomerId,
                        best.newCustProfit, best.newCustDemand,
                        best.objGain);
            }
        }

        return totalTransfers;
    }

    // ══════════════════════════════════════════════════════════
    // FIND BEST REMOVE-TRANSFER-INSERT MOVE
    // ══════════════════════════════════════════════════════════

    private TransferCandidate findBestRemoveTransferInsert(Solution solution) {
        Instance inst = solution.getInstance();
        List<Route> routes = solution.getRoutes();
        List<Node> unserved = solution.getUnservedNodes();

        if (unserved.isEmpty()) return null;

        double currentObj = computeObjective(solution);
        TransferCandidate best = null;
        double bestGain = 1e-6; // must be strictly positive

        // Pre-sort unserved by profit descending for early termination
        unserved.sort(Comparator.comparingDouble(Node::getProfit).reversed());

        for (int gi = 0; gi < routes.size(); gi++) {
            Route giver = routes.get(gi);
            if (giver.size() < 2) continue; // need at least 2 stops: one to remove, one as transfer point
            giver.evaluate();

            // Try removing each served node from giver
            for (int sPos = 0; sPos < giver.size(); sPos++) {
                RouteStop removeStop = giver.getStops().get(sPos);
                if (!removeStop.isServed()) continue;

                Node removedNode = removeStop.getNode();
                double freedCapacity = removedNode.getDemand();

                // Quick filter: is there any unserved node with higher profit?
                // (otherwise removing this node can't be beneficial)
                boolean anyBetter = false;
                for (Node u : unserved) {
                    if (u.getProfit() > removedNode.getProfit()) { anyBetter = true; break; }
                }
                if (!anyBetter) continue;

                // Remove the node from giver (temporarily)
                giver.removeStop(sPos);
                giver.evaluate();

                // Now try each REMAINING node in giver as a transfer point
                for (int jPos = 0; jPos < giver.size(); jPos++) {
                    RouteStop transferStop = giver.getStops().get(jPos);
                    Node transferNode = transferStop.getNode();
                    int transferNodeId = transferNode.getId();

                    // Try each receiver route
                    for (int ri = 0; ri < routes.size(); ri++) {
                        if (ri == gi) continue;
                        Route receiver = routes.get(ri);
                        receiver.evaluate();

                        double receiverSlack = inst.getMaxCapacity() - receiver.getInitialLoad();

                        // Try each unserved customer
                        for (Node newCust : unserved) {
                            // The receiver needs enough capacity for the new customer
                            // Capacity comes from: receiverSlack + transferQty (from pickup)
                            double neededFromTransfer = newCust.getDemand() - receiverSlack;
                            if (neededFromTransfer <= 0) continue; // doesn't need transfer
                            if (neededFromTransfer > freedCapacity) continue; // giver can't provide enough

                            double transferQty = neededFromTransfer;

                            // === Test giver with dropoff ===
                            Route giverTest = new Route(giver); // copy of giver with node removed
                            RouteStop origStop = giverTest.getStops().get(jPos);

                            // Add dropoff to the transfer node
                            RouteStop modifiedStop;
                            if (origStop.isServed() && !origStop.isDropoff()) {
                                modifiedStop = RouteStop.serveAndDropoff(transferNode, transferQty);
                            } else if (origStop.isServed() && origStop.isDropoff()) {
                                modifiedStop = new RouteStop(transferNode, true, false, true,
                                        0, origStop.getDropoffQty() + transferQty);
                            } else {
                                // Transfer-only stop (shouldn't happen normally)
                                modifiedStop = RouteStop.dropoff(transferNode, transferQty);
                            }

                            giverTest.getStops().set(jPos, modifiedStop);
                            giverTest.evaluate();

                            if (!giverTest.isFeasible()) continue;

                            double giverArrivalAtJ = giverTest.getArrivalTimeAtNode(transferNodeId);

                            // === Test receiver: pickup at j + insert new customer ===

                            // Does receiver already visit the transfer node?
                            int existingIdx = receiver.findStopIndex(transferNodeId);

                            if (existingIdx >= 0) {
                                // Receiver already visits this node
                                TransferCandidate candidate = tryReceiverWithExistingVisit(
                                        solution, gi, giverTest, ri, receiver,
                                        transferNodeId, transferQty, existingIdx,
                                        newCust, giverArrivalAtJ, inst,
                                        currentObj, removedNode, sPos, jPos,
                                        bestGain, best);
                                if (candidate != null && candidate.objGain > bestGain) {
                                    bestGain = candidate.objGain;
                                    best = candidate;
                                }

                            } else {
                                // Receiver must detour to transfer node
                                TransferCandidate candidate = tryReceiverWithDetour(
                                        solution, gi, giverTest, ri, receiver,
                                        transferNode, transferQty,
                                        newCust, giverArrivalAtJ, inst,
                                        currentObj, removedNode, sPos, jPos,
                                        bestGain);
                                if (candidate != null && candidate.objGain > bestGain) {
                                    bestGain = candidate.objGain;
                                    best = candidate;
                                }
                            }
                        }
                    }
                }

                // Restore giver
                giver.insertStop(sPos, removeStop);
                giver.evaluate();
            }
        }

        return best;
    }

    // ══════════════════════════════════════════════════════════
    // RECEIVER: EXISTING VISIT (already visits transfer node)
    // ══════════════════════════════════════════════════════════

    private TransferCandidate tryReceiverWithExistingVisit(
            Solution solution, int gi, Route giverTest, int ri, Route receiver,
            int transferNodeId, double transferQty, int existingIdx,
            Node newCust, double giverArrivalAtJ, Instance inst,
            double currentObj, Node removedNode, int removedPos, int transferPos,
            double bestGain, TransferCandidate currentBest) {

        Route recTest = new Route(receiver);

        // Modify existing stop to include pickup
        RouteStop existingStop = recTest.getStops().get(existingIdx);
        RouteStop updatedStop;
        if (existingStop.isServed()) {
            updatedStop = RouteStop.serveAndPickup(
                    inst.getNodeById(transferNodeId), transferQty);
        } else {
            updatedStop = RouteStop.pickup(
                    inst.getNodeById(transferNodeId), transferQty);
        }
        recTest.getStops().set(existingIdx, updatedStop);
        recTest.evaluate();

        if (!recTest.isFeasible()) return null;

        // Check sync
        double recArrival = recTest.getArrivalTimeAtNode(transferNodeId);
        if (Math.abs(giverArrivalAtJ - recArrival) > inst.getSyncWindow() + 1e-6) return null;

        // Try inserting new customer at each position
        TransferCandidate best = null;
        for (int nPos = 0; nPos <= recTest.size(); nPos++) {
            recTest.insertStop(nPos, RouteStop.serve(newCust));
            recTest.evaluate();

            if (recTest.isFeasible()) {
                // Re-check sync after insertion (arrival times may shift)
                double recArrival2 = recTest.getArrivalTimeAtNode(transferNodeId);
                if (Math.abs(giverArrivalAtJ - recArrival2) <= inst.getSyncWindow() + 1e-6) {
                    double newObj = computeObjWith(solution, gi, giverTest, ri, recTest);
                    double gain = newObj - currentObj;

                    if (gain > bestGain) {
                        best = new TransferCandidate(
                                gi, ri, transferNodeId, transferQty,
                                removedNode.getId(), removedNode.getProfit(), removedNode.getDemand(),
                                removedPos, transferPos,
                                newCust.getId(), newCust.getProfit(), newCust.getDemand(),
                                existingIdx, nPos, gain, false);
                        bestGain = gain;
                    }
                }
            }

            recTest.removeStop(nPos);
            recTest.evaluate();
        }

        return best;
    }

    // ══════════════════════════════════════════════════════════
    // RECEIVER: DETOUR (must add visit to transfer node)
    // ══════════════════════════════════════════════════════════

    private TransferCandidate tryReceiverWithDetour(
            Solution solution, int gi, Route giverTest, int ri, Route receiver,
            Node transferNode, double transferQty,
            Node newCust, double giverArrivalAtJ, Instance inst,
            double currentObj, Node removedNode, int removedPos, int transferPos,
            double bestGain) {

        TransferCandidate best = null;

        // Try each position for the pickup detour
        for (int pPos = 0; pPos <= receiver.size(); pPos++) {
            Route recTest = new Route(receiver);

            recTest.insertStop(pPos, RouteStop.pickup(transferNode, transferQty));
            recTest.evaluate();

            if (!recTest.isFeasible()) continue;

            // Check sync
            double recArrival;
            try {
                recArrival = recTest.getArrivalTimeAtNode(transferNode.getId());
            } catch (Exception e) { continue; }

            if (Math.abs(giverArrivalAtJ - recArrival) > inst.getSyncWindow() + 1e-6) continue;

            // Try inserting new customer at each position
            for (int nPos = 0; nPos <= recTest.size(); nPos++) {
                recTest.insertStop(nPos, RouteStop.serve(newCust));
                recTest.evaluate();

                if (recTest.isFeasible()) {
                    // Re-check sync
                    double recArrival2;
                    try {
                        recArrival2 = recTest.getArrivalTimeAtNode(transferNode.getId());
                    } catch (Exception e) { recTest.removeStop(nPos); recTest.evaluate(); continue; }

                    if (Math.abs(giverArrivalAtJ - recArrival2) <= inst.getSyncWindow() + 1e-6) {
                        double newObj = computeObjWith(solution, gi, giverTest, ri, recTest);
                        double gain = newObj - currentObj;

                        if (gain > bestGain) {
                            best = new TransferCandidate(
                                    gi, ri, transferNode.getId(), transferQty,
                                    removedNode.getId(), removedNode.getProfit(), removedNode.getDemand(),
                                    removedPos, transferPos,
                                    newCust.getId(), newCust.getProfit(), newCust.getDemand(),
                                    pPos, nPos, gain, true);
                            bestGain = gain;
                        }
                    }
                }

                recTest.removeStop(nPos);
                recTest.evaluate();
            }
        }

        return best;
    }

    // ══════════════════════════════════════════════════════════
    // EXECUTE THE TRANSFER
    // ══════════════════════════════════════════════════════════

    private void executeTransfer(Solution solution, TransferCandidate tc) {
        Instance inst = solution.getInstance();
        List<Route> routes = solution.getRoutes();
        Route giver = routes.get(tc.giverRouteIdx);
        Route receiver = routes.get(tc.receiverRouteIdx);

        Node transferNode = inst.getNodeById(tc.transferNodeId);
        Node newCust = inst.getNodeById(tc.newCustomerId);

        // 1. Remove the low-profit node from giver
        giver.removeStop(tc.removedPos);
        giver.evaluate();

        // 2. Add dropoff at transfer node in giver
        // Find the transfer node's new position (may have shifted after removal)
        int jPos = giver.findStopIndex(tc.transferNodeId);
        if (jPos < 0) {
            System.err.println("[Transfer] ERROR: transfer node not found after removal!");
            return;
        }

        RouteStop origStop = giver.getStops().get(jPos);
        RouteStop newGiverStop;
        if (origStop.isServed() && !origStop.isDropoff()) {
            newGiverStop = RouteStop.serveAndDropoff(transferNode, tc.transferQty);
        } else if (origStop.isServed() && origStop.isDropoff()) {
            newGiverStop = new RouteStop(transferNode, true, false, true,
                    0, origStop.getDropoffQty() + tc.transferQty);
        } else {
            newGiverStop = RouteStop.dropoff(transferNode, tc.transferQty);
        }
        giver.getStops().set(jPos, newGiverStop);
        giver.evaluate();

        // 3. Add pickup at transfer node in receiver
        if (tc.isDetour) {
            receiver.insertStop(tc.receiverPickupPos,
                    RouteStop.pickup(transferNode, tc.transferQty));
            receiver.evaluate();

            // 4. Insert new customer in receiver
            receiver.insertStop(tc.newCustInsertPos, RouteStop.serve(newCust));
        } else {
            // Receiver already visits — update existing stop
            RouteStop existingStop = receiver.getStops().get(tc.receiverPickupPos);
            RouteStop updatedStop;
            if (existingStop.isServed()) {
                updatedStop = RouteStop.serveAndPickup(transferNode, tc.transferQty);
            } else {
                updatedStop = RouteStop.pickup(transferNode, tc.transferQty);
            }
            receiver.getStops().set(tc.receiverPickupPos, updatedStop);
            receiver.evaluate();

            receiver.insertStop(tc.newCustInsertPos, RouteStop.serve(newCust));
        }
        receiver.evaluate();

        // 5. Register transfer
        Transfer transfer = new Transfer(tc.transferNodeId,
                giver.getVehicleId(), receiver.getVehicleId(), tc.transferQty);
        solution.addTransfer(transfer);
    }

    // ══════════════════════════════════════════════════════════
    // UTILITIES
    // ══════════════════════════════════════════════════════════

    private double computeObjWith(Solution sol, int gi, Route giverNew,
                                  int ri, Route receiverNew) {
        double profit = 0, dist = 0;
        List<Route> routes = sol.getRoutes();
        for (int i = 0; i < routes.size(); i++) {
            Route r = (i == gi) ? giverNew : (i == ri) ? receiverNew : routes.get(i);
            profit += r.getTotalProfit();
            dist += r.getTotalDistance();
        }
        return profit - objectiveAlpha * dist;
    }

    private double computeObjective(Solution sol) {
        double profit = 0, dist = 0;
        for (Route r : sol.getRoutes()) {
            profit += r.getTotalProfit();
            dist += r.getTotalDistance();
        }
        return profit - objectiveAlpha * dist;
    }

    // ══════════════════════════════════════════════════════════
    // CANDIDATE DATA
    // ══════════════════════════════════════════════════════════

    private static class TransferCandidate {
        final int giverRouteIdx, receiverRouteIdx;
        final int transferNodeId;
        final double transferQty;

        // Removed node info
        final int removedNodeId;
        final double removedProfit, removedDemand;
        final int removedPos;
        final int transferPos; // position of transfer node in giver (before removal)

        // New customer info
        final int newCustomerId;
        final double newCustProfit, newCustDemand;

        // Receiver positions
        final int receiverPickupPos;
        final int newCustInsertPos;

        final double objGain;
        final boolean isDetour;

        TransferCandidate(int giverRouteIdx, int receiverRouteIdx,
                          int transferNodeId, double transferQty,
                          int removedNodeId, double removedProfit, double removedDemand,
                          int removedPos, int transferPos,
                          int newCustomerId, double newCustProfit, double newCustDemand,
                          int receiverPickupPos, int newCustInsertPos,
                          double objGain, boolean isDetour) {
            this.giverRouteIdx = giverRouteIdx;
            this.receiverRouteIdx = receiverRouteIdx;
            this.transferNodeId = transferNodeId;
            this.transferQty = transferQty;
            this.removedNodeId = removedNodeId;
            this.removedProfit = removedProfit;
            this.removedDemand = removedDemand;
            this.removedPos = removedPos;
            this.transferPos = transferPos;
            this.newCustomerId = newCustomerId;
            this.newCustProfit = newCustProfit;
            this.newCustDemand = newCustDemand;
            this.receiverPickupPos = receiverPickupPos;
            this.newCustInsertPos = newCustInsertPos;
            this.objGain = objGain;
            this.isDetour = isDetour;
        }
    }
}