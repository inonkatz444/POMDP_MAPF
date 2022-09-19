package pomdp;

import pomdp.environments.Grid;
import pomdp.utilities.*;

import java.util.*;
import java.util.stream.Collectors;

public class Env {

    private static boolean AllDone(List<GridAgent> agents) {
        return !agents.stream().map(GridAgent::isDone).collect(Collectors.toList()).contains(false);
    }

    private static double runOfflineJoint(List<GridAgent> agents, String sMethodName, String sModelName, int maxIterations, int maxSteps) {
        agents.forEach(a -> a.initRun(sModelName));
        GridJointAgent jointAgent = new GridJointAgent();
        jointAgent.initOfflineRun(agents);
        return jointAgent.solve(sMethodName, 100.0, maxIterations, maxSteps);
    }

    private static double runMiniGrids(List<GridAgent> agents, String sMethodName, String sModelName, int maxIterations, int maxSteps) {
        List<GridAgent> agents_copy = agents.subList(0, agents.size());
        int iStep;
        PotentialCollisionData potentialCollision = null;
        boolean collisionDetected;
        for (GridAgent agent : agents) {
            agent.initRun(sModelName);
            agent.solve(sMethodName, 100.0, maxIterations, maxSteps);
        }
        agents.forEach(agent -> System.out.println("Agent " + agent.getID() + " starts at " + agent.getStartStateString()));
        iStep = 0;
        TrackLogger.getInstance().freeLog(0, printLetteredEnv(agents));
        System.out.println(printColoredEnv(agents));
        while ((iStep < maxSteps) && !AllDone(agents)) {
            potentialCollision = mightCollidingAgents(agents);
            if (!potentialCollision.isEmpty()) {
                GridJointAgent jointAgent = new GridJointAgent();
                jointAgent.initOnlineRun(potentialCollision);
                jointAgent.solve(sMethodName, 100.0, maxIterations, maxSteps);
                boolean jointDone = false;
                while (iStep < maxSteps && !jointDone) {
                    jointDone = jointAgent.step();
                    iStep++;
                    TrackLogger.getInstance().freeLog(0, printLetteredEnv(agents));
                    System.out.println(printColoredEnv(agents));
                }
                jointAgent = null;
                Runtime.getRuntime().gc();
                System.out.println("The joint Agent is done... ");
            }
            else {
                for (GridAgent agent : agents) {
                    agent.step();
                }
                iStep++;
                TrackLogger.getInstance().freeLog(0, printLetteredEnv(agents));
                System.out.println(printColoredEnv(agents));
            }
            agents = agents.stream().filter(agent -> !agent.isDone()).collect(Collectors.toList());
        }

        for (GridAgent a : agents) {
            if (!a.isDone()) {
                a.addPunishment();
            }
        }
        TrackLogger.getInstance().log("Env", 0, "runMiniGrid", true, "All agents are done, Sum of discounted rewards: " + agents_copy.stream().reduce(0.0, (acc, agent) -> acc + agent.getSumOfDiscountedRewards(), Double::sum));

        return agents_copy.stream().reduce(0.0, (acc, agent) -> acc + agent.getSumOfDiscountedRewards(), Double::sum);
    }

    public static double runForbidden(List<GridAgent> agents, String sMethodName, String sModelName, int solverIterations, int maxSteps, int initialTimer) {
        PotentialCollisionData mightCollide;
        List<GridAgent> agents_copy = agents.subList(0, agents.size());

        for (GridAgent agent : agents) {
            agent.initRun(sModelName);
            agent.solve(sMethodName, 100.0, solverIterations, maxSteps);
        }
        agents.forEach(agent -> agent.log("Env", 0, "runForbidden", true, "Agent " + agent.getID() + " starts at " + agent.getStartStateString()));

        for (int iStep = 0; iStep < maxSteps && !AllDone(agents); iStep++) {
            TrackLogger.getInstance().freeLog(0, printLetteredEnv(agents));
            System.out.println(printColoredEnv(agents));

            do {
                for (GridAgent agent : agents) {
                    if (agent.finishEscaping()) {
                        agent.clearForbiddenStates();
                        agent.initRun(sModelName);
                        agent.solve(sMethodName, 100.0, solverIterations, maxSteps);
                    }
                }

                mightCollide = mightCollidingAgents(agents);
                if (!mightCollide.isEmpty()) {
                    double solvedADR;
                    GridAgent nonDominant;
//                Set<Integer> collisionStates = mightCollide.getCollisionStates();
                    do {
                        nonDominant = mightCollide.getNextNonDominant();
                        nonDominant.saveForbiddenStates();
                        nonDominant.clearForbiddenStates();

                        // adds all the dominant's possible states to nonDominant's forbidden states
                        for (GridAgent a: agents) {
                            if (a != nonDominant) {
                                a.clearExpandedBeliefs();
                                for (int iPotentialStep = 0; iPotentialStep <= mightCollide.getCollisionStep(); iPotentialStep++) {
                                    nonDominant.addForbiddenStates(a.getPossibleStates());
                                    a.expandBeliefsStep(iPotentialStep);
                                }
                            }
                        }
                        nonDominant.initRun(sModelName);
                        solvedADR = nonDominant.solve(sMethodName, 100.0, solverIterations, maxSteps);
                        if (solvedADR < 0) {
                            nonDominant.restoreForbiddenStates();
                        }
                    } while (solvedADR < 0 && mightCollide.hasNextNonDominant());
                    if (solvedADR >= 0) {
                        nonDominant.setForbiddenTimer(initialTimer);
                        nonDominant.log("Env", 0, "runForbidden", false, "finished finding forbidden states");
                        break;
                    }
                    else {
                        TrackLogger.getInstance().log("Env", 0, "runForbidden", true, "Localizing...");
                        iStep += agents.get(agents.size()-1).localize();
                        for (int i = agents.size()-2; i >= 0; i--) {
                            agents.get(i).localize();
                        }
                        TrackLogger.getInstance().freeLog(0, printLetteredEnv(agents));
                        System.out.println(printColoredEnv(agents));
//                        mightCollide.getMostNonDominant().setNullPolicy();
//                        break;
                    }
                }
            }
            while (!mightCollide.isEmpty() && iStep < maxSteps);
            if (!(iStep < maxSteps)) {
                break;
            }
            for (GridAgent agent : agents) {
                agent.step();
            }
            agents = agents.stream().filter(agent -> !agent.isDone()).collect(Collectors.toList());
        }
        for (GridAgent a : agents) {
            if (!a.isDone()) {
                a.addPunishment();
            }
        }

        TrackLogger.getInstance().log("Env", 0, "runForbidden", true, "All agents are done, Sum of discounted rewards: " + agents_copy.stream().reduce(0.0, (acc, agent) -> acc + agent.getSumOfDiscountedRewards(), Double::sum));
        return agents_copy.stream().reduce(0.0, (acc, agent) -> acc + agent.getSumOfDiscountedRewards(), Double::sum);
    }

    private static GridAgent chooseNonDominant(PotentialCollisionData mightCollide) {
        int nonDominantID = mightCollide.getAgents().get(0).distanceToGoal() < mightCollide.getAgents().get(1).distanceToGoal() ? 1 : 0;
        if (mightCollide.getAgents().get(nonDominantID).isTimerSet() && !mightCollide.getAgents().get(1 - nonDominantID).isTimerSet()) {
            nonDominantID = 1 - nonDominantID;
        }
        return mightCollide.getAgents().get(nonDominantID);
    }

    private static PotentialCollisionData mightCollidingAgents(List<GridAgent> agents) {
        PotentialCollisionData potentialCollision = new PotentialCollisionData();
//        for (int iAgent = 0; iAgent < agents.size(); iAgent++) {
//            GridAgent agent = agents.get(iAgent);
//            for (int iOtherAgent = iAgent+1; iOtherAgent < agents.size(); iOtherAgent++) {
//                if (iOtherAgent != iAgent) {
//                    GridAgent otherAgent = agents.get(iOtherAgent);
//                    if (agent.isClose(otherAgent)) {
//                        Set<Integer> collisionStates;
//                        agent.clearExpandedBeliefs();
//                        otherAgent.clearExpandedBeliefs();
//                        int agentMovingActionCounter = 0, otherMovingActionCounter = 0;
//                        int iStep = 0;
//                        while (Math.min(agentMovingActionCounter, otherMovingActionCounter) <= agent.getDistanceThreshold() && Math.max(agentMovingActionCounter, otherMovingActionCounter) <= 2 * agent.getDistanceThreshold() && iStep <= TIMEOUT) {
//                            collisionStates = getCollisionStates(agent, otherAgent);
//                            if (collisionStates.size() > 0) {
//                                for (int collisionState : collisionStates) {
//                                    agent.log("Env", 0, "mightCollidingAgents", true, "Agent " + agent.getID() + " and agent " + otherAgent.getID() + " may collide in " + agent.getGrid().parseState(collisionState));
//                                    otherAgent.getAgentLogger().log("Env", 0, "mightCollidingAgents", false, "Agent " + agent.getID() + " and agent " + otherAgent.getID() + " may collide in " + agent.getGrid().parseState(collisionState));
//                                }
//                                potentialCollision.addAgent(agent);
//                                potentialCollision.addAgent(otherAgent);
//                                potentialCollision.addCollisionStates(collisionStates);
//                                potentialCollision.setCollisionStep(iStep);
//                                return potentialCollision;
//                            }
//                            if (agent.expandBeliefsStep(iStep)) {
//                                agentMovingActionCounter++;
//                            }
//                            if (otherAgent.expandBeliefsStep(iStep)) {
//                                otherMovingActionCounter++;
//                            }
//                            iStep++;
//                        }
//                    }
//                }
//            }
//        }

        for (int k = agents.size(); k > 1; k--) {
            potentialCollision = processSubsets(agents.toArray(new GridAgent[0]), k);
            if (!potentialCollision.isEmpty()) {
                return potentialCollision;
            }
        }

        return potentialCollision;
    }

    static PotentialCollisionData CheckMultiCollision(GridAgent[] subsetAgents) {
        // TODO: Check if those agents are close enough to collide in the k-step future
        PotentialCollisionData potentialCollision = new PotentialCollisionData();
        for (GridAgent gridAgent : subsetAgents) {
            gridAgent.clearExpandedBeliefs();
        }
        int[] movingActionCounter = new int[subsetAgents.length];
        int iStep = 0;
        final int TIMEOUT = 10;
        int minMovingActionCounter = 0, maxMovingActionCounter = 0;
        while (minMovingActionCounter <= subsetAgents[0].getDistanceThreshold() && maxMovingActionCounter <= 2 * subsetAgents[0].getDistanceThreshold() && iStep < TIMEOUT) {
            Set<Integer> collisionStates = getCollisionStates(subsetAgents);
            if (collisionStates.size() > 0) {
                String ids = String.join(", ", Arrays.stream(subsetAgents).map(a -> "" + a.getID()).toArray(String[]::new));
                for (int collisionState : collisionStates) {
                    subsetAgents[0].log("Env", 0, "mightCollidingAgents", true, "Agents " + ids + " may collide in " + subsetAgents[0].getGrid().parseState(collisionState));
                    for (int i = 1; i < subsetAgents.length; i++) {
                        subsetAgents[i].getAgentLogger().log("Env", 0, "mightCollidingAgents", false, "Agents " + ids + " may collide in " + subsetAgents[i].getGrid().parseState(collisionState));
                    }
                }
                potentialCollision.addAgents(subsetAgents);
                potentialCollision.addCollisionStates(collisionStates);
                potentialCollision.setCollisionStep(iStep);
                return potentialCollision;
            }

            for (int i = 0; i < subsetAgents.length; i++) {
                if (subsetAgents[i].expandBeliefsStep(iStep)) {
                    movingActionCounter[i]++;
                }
            }

            minMovingActionCounter = Integer.MAX_VALUE;
            maxMovingActionCounter = Integer.MIN_VALUE;
            for (int i = 0; i < subsetAgents.length; i++) {
                if (movingActionCounter[i] < minMovingActionCounter) {
                    minMovingActionCounter = movingActionCounter[i];
                }
                if (movingActionCounter[i] > maxMovingActionCounter) {
                    maxMovingActionCounter = movingActionCounter[i];
                }
            }

            iStep++;
        }
        return potentialCollision;
    }

    static PotentialCollisionData processSubsets(GridAgent[] agents, int k) {
        GridAgent[] subsetAgents = new GridAgent[k];
        return processLargerSubsets(agents, subsetAgents, 0, 0);
    }

    static PotentialCollisionData processLargerSubsets(GridAgent[] agents, GridAgent[] subsetAgents, int subsetSize, int nextIndex) {
        if (subsetSize == subsetAgents.length) {
            return CheckMultiCollision(subsetAgents);
        } else {
            PotentialCollisionData ret;
            for (int j = nextIndex; j < agents.length; j++) {
                subsetAgents[subsetSize] = agents[j];
                ret = processLargerSubsets(agents, subsetAgents, subsetSize + 1, j + 1);
                if (ret != null) {
                    return ret;
                }
            }
        }
        return null;
    }

    private static Set<Integer> getCollisionStates(GridAgent[] agents) {
        Set<Integer> collisionStates = agents[0].getPossibleStates();
        System.out.println("Agent " + agents[0].getID() + " possible states: " + collisionStates.stream().map(state -> agents[0].getGrid().parseState(state)).collect(Collectors.toList()).toString());
        for (int i = 1; i < agents.length; i++) {
            Set<Integer> possibleStates = agents[i].getPossibleStates();
            int finalI = i;
            System.out.println("Agent " + agents[i].getID() + " possible states: " + possibleStates.stream().map(state -> agents[finalI].getGrid().parseState(state)).collect(Collectors.toList()).toString());
            collisionStates.retainAll(possibleStates);
        }
        return collisionStates;
    }

    private static String printLetteredEnv(List<GridAgent> agents) {
        int rows = agents.get(0).getGrid().getRows();
        int cols = agents.get(0).getGrid().getCols();
        StringBuilder env = new StringBuilder();

        env.append("#".repeat(Math.max(0, cols + 2)));
        env.append('\n');

        for (int i = 0; i < rows; i++) {
            env.append("#");
            for (int j = 0; j < cols; j++) {
                if (agents.get(0).getGrid().locationToState(i, j) == agents.get(0).getGrid().HOLE) {
                    env.append("#");
                    continue;
                }

                int state = agents.get(0).getGrid().locationToState(i, j);
                boolean foundAgent = false;
                boolean foundChar = false;
                for (GridAgent agent : agents) {
                    if (agent.stateProb(state) > 0) {
                        if (!foundAgent) {
                            env.append(agent.getID());
                            foundAgent = true;
                            foundChar = true;
                        } else {
                            throw new RuntimeException("Collision in state " + agents.get(0).getGrid().stateToLocation(state));
                        }
                    } else if (!foundChar && state == agent.getEndState()) {
                        env.append((char) (agent.getID() - 0x20));
                        foundChar = true;
                    }
                }
                if (!foundChar) {
                    env.append(" ");
                }
            }
            env.append("#\n");
        }

        env.append("#".repeat(Math.max(0, cols + 2)));
        env.append('\n');

        return env.toString();
    }

    private static String printColoredEnv(List<GridAgent> agents) {
        int rows = agents.get(0).getGrid().getRows();
        int cols = agents.get(0).getGrid().getCols();
        String reset = "\u001B[0m";
        StringBuilder env = new StringBuilder();

        env.append("#".repeat(Math.max(0, cols + 2)));
        env.append('\n');

        for (int i = 0; i < rows; i++) {
            env.append("#");
            for (int j = 0; j < cols; j++) {
                if (agents.get(0).getGrid().locationToState(i, j) == agents.get(0).getGrid().HOLE) {
                    env.append("#");
                    continue;
                }

                int state = agents.get(0).getGrid().locationToState(i, j);
                boolean foundAgent = false;
                boolean foundChar = false;
                // TODO: Separate the background color from the letter.
                for (GridAgent agent : agents) {
                    if (agent.stateProb(state) > 0) {
                        env.append("\u001B[4").append((char) (agent.getID() - 0x30)).append('m');
                        if (!foundAgent) {
                            foundAgent = true;
                        } else {
                            throw new RuntimeException("Collision in state " + agents.get(0).getGrid().stateToLocation(state));
                        }
                    }
                    if (!foundChar && state == agent.getCurrentState()) {
                        env.append(agent.getID()).append(reset);
                        foundChar = true;
                    }
                    else if (!foundChar && state == agent.getEndState() && !agent.isDone()) {
                        env.append("\u001B[3").append((char) (agent.getID() - 0x30)).append('m').append((char) (agent.getID() - 0x20)).append(reset);
                        foundChar = true;
                    }
                }
                if (!foundChar) {
                    for (Beacon b : agents.get(0).getGrid().getBeacons()){
                        if (b.getLoc().equals(Point.getPoint(i, j))) {
                            env.append(b.getRange()).append(reset);
                            foundChar = true;
                            break;
                        }
                    }
                }
                if (!foundChar) {
                    env.append(" ").append(reset);
                }
            }
            env.append("#\n");
        }

        env.append("#".repeat(Math.max(0, cols + 2)));
        env.append('\n');

        return env.toString();
    }

    public static void main(String[] args) {
        JProf.getCurrentThreadCpuTimeSafe();
//        String sModelName = "two_paths_one_beacon";
//        String sModelName = "room";

//        String sModelName = "easy_5";
//        String sModelName = "easy_10";

        String sModelName = "medium_5";
//        String sModelName = "medium_10";

//        String sModelName = "hard_5";
//        String sModelName = "hard_10";

//        String sModelName = "switch_5";
//        String sModelName = "switch_10_19";

//        String sModelName = "hallways_14_11";

//        String sModelName = "joint_test";
//        String sMethodName = "Perseus";
        String sMethodName = "FSVI";
//        String sMethodName = "HSVI";
        int distanceThreshold = 1;
        int initialTimer = 3;
        int num_of_agents = 2;
        double ADR = 0;

        try{
            Logger.getInstance().setOutputStream( sModelName + "_" + sMethodName + ".txt" );
            TrackLogger.getInstance().setOutputStream(sModelName + "_" + sMethodName + "_track.txt");
        }
        catch( Exception e ){
            System.err.println( e );
        }

        List<GridAgent> agents = new ArrayList<>();

        for (int i = 0; i < num_of_agents; i++) {
            GridAgent agent = new GridAgent(distanceThreshold);
            agent.setLogger(sModelName + "_" + sMethodName + "_" + agent.getID() + "_track.txt");
            agents.add(agent);
        }

//        long startTime = System.currentTimeMillis();
//        ADR = runOfflineJoint(agents, sMethodName, sModelName, 115, 200);
//        long endTime = System.currentTimeMillis();
//        TrackLogger.getInstance().log("Env", 0, "main", true, "ADR: " + ADR);
//        TrackLogger.getInstance().log("Env", 0, "main", true, "Time elapsed (s): " + ((endTime-startTime) / 1000));

        int numOfTests = 50;
        double[] ADRs = new double[numOfTests];
        long totalTime = 0, startTime, endTime;
        double percentageSolved = 0;
        for (int i = 0; i < numOfTests; i++) {
            int finalI = i;
            agents.forEach(a -> a.log("Env", 0, "main", false, "Test " + finalI));
            System.out.println("Test " + i);
            startTime = System.currentTimeMillis();
            double sumOfDiscountedRewards = runForbidden(agents, sMethodName, sModelName, 100, 200, initialTimer);
            endTime = System.currentTimeMillis();
            ADR += sumOfDiscountedRewards;
            ADRs[i] = Math.round(sumOfDiscountedRewards * 1000) / 1000.0;
            TrackLogger.getInstance().log("Env", 0, "main", true, "Time of iteration " + i + ": " + (endTime - startTime) / 1000);
            totalTime += (endTime - startTime);
            if (agents.stream().allMatch(GridAgent::isSolved)) {
                percentageSolved++;
            }
            agents.forEach(GridAgent::reset);
        }
        ADR = ADR / numOfTests;

        double std = 0;
        for (double ADR1 : ADRs) {
            std += Math.pow(ADR1 - ADR, 2);
        }
        std = Math.sqrt(std / numOfTests-1) / Math.sqrt(numOfTests);

        TrackLogger.getInstance().log("Env", 0, "main", true, "Sum of discounted rewards: " + Arrays.toString(ADRs));
        TrackLogger.getInstance().log("Env", 0, "main", true, "ADR: " + ADR);
        TrackLogger.getInstance().log("Env", 0, "main", true, "std: " + std);
        TrackLogger.getInstance().log("Env", 0, "main", true, (int)(percentageSolved * 100 / numOfTests) + "% Solved");
        TrackLogger.getInstance().log("Env", 0, "main", true, "Time elapsed (s): " + (totalTime / 1000));
        TrackLogger.getInstance().log("Env", 0, "main", true, "Average runtime (s): " + (totalTime / 1000) * 1.0 / numOfTests);
        TrackLogger.getInstance().log("Env", 0, "main", true, "parameters:");
        TrackLogger.getInstance().log("Env", 0, "main", true, "\tinitialTimer: " + initialTimer);
        TrackLogger.getInstance().log("Env", 0, "main", true, "\tdistanceThreshold: " + distanceThreshold);
    }
}
