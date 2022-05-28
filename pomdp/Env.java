package pomdp;

import pomdp.utilities.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;

public class Env {

    private static boolean AllDone(List<GridAgent> agents) {
        return !agents.stream().map(GridAgent::isDone).toList().contains(false);
    }

    private static boolean runMiniGrids(List<GridAgent> agents, String sMethodName, String sModelName, int maxSteps) {
        int iStep, n = agents.size();
        PotentialCollisionData potentialCollision = null;
        boolean collisionDetected;
        for (GridAgent agent : agents) {
            agent.initRun(sModelName);
            agent.solve(sMethodName, 100.0, 17, maxSteps);
            if (!agent.hasConverged()) {
                System.out.println("DEBUG: agent " + agent.getID() + " timed out!");
            }
        }
        agents.forEach(agent -> System.out.println("Agent " + agent.getID() + " starts at " + agent.getStartStateString()));
        collisionDetected = false;
        iStep = 0;
        while ((iStep < maxSteps) && !AllDone(agents)) {
            for (int i = 0; i < n; i++) {
                GridAgent agent = agents.get(i);
                if (!agent.isDone()) {
                    potentialCollision = mightCollidingAgents(i, agents);
                    collisionDetected = !potentialCollision.isEmpty();
                    if (collisionDetected) {

                        break;
                    }
                    agent.step();
                }
            }
            System.out.println();
            if (collisionDetected) {
                GridJointAgent jointAgent = new GridJointAgent();
                jointAgent.initRun(potentialCollision);
                jointAgent.solve(sMethodName, 100.0, 17, maxSteps);
                boolean jointDone = false;
                while (iStep < maxSteps && !jointDone) {
                    jointDone = jointAgent.step();
                    iStep++;
                }
                System.out.println("The joint Agent is done... ");
                collisionDetected = false;
            }
            else {
                iStep++;
            }
        }
        return AllDone(agents);
    }

    public static boolean runForbidden(List<GridAgent> agents, String sMethodName, String sModelName, int maxHighLevelIterations, int maxSteps) {
        int iStep, n = agents.size();
        boolean collisionDetected = true;
        PotentialCollisionData mightCollide;

        for (int iIteration = 1; (iIteration <= maxHighLevelIterations) && collisionDetected; iIteration++) {
            System.out.println("High Level Iteration: " + iIteration);
            collisionDetected = false;
            for (GridAgent agent : agents) {
                if (agent.needsRetrain()) {
                    agent.initRun(sModelName);
                    agent.solve(sMethodName, 100.0, 17, maxSteps);
                    if (!agent.hasConverged()) {
                        System.out.println("DEBUG: agent " + agent.getID() + " timed out!");
                    }
                }
            }
            for (GridAgent agent : agents) {
                System.out.println("Agent " + agent.getID() + " forbidden states: " + agent.getForbiddenStates().stream().map(s -> agents.get(0).getGrid().parseState(s)).toList().toString());
            }
            agents.forEach(agent -> System.out.println("Agent " + agent.getID() + " starts at " + agent.getStartStateString()));
            for (iStep = 0; (iStep < maxSteps) && !collisionDetected && !AllDone(agents); iStep++) {
                for (int i = 0; i < n; i++) {
                    GridAgent agent = agents.get(i);
                    if (!agent.isDone()) {
                        mightCollide = mightCollidingAgents(i, agents);
                        collisionDetected = !mightCollide.isEmpty();
                        if (collisionDetected) {
                            Set<Integer> collisionStates = getCollisionStates(mightCollide.getAgents().get(0), mightCollide.getAgents().get(1));
                            GridAgent nonDominant = mightCollide.getAgents().get(0).distanceToGoal() < mightCollide.getAgents().get(1).distanceToGoal() ? mightCollide.getAgents().get(1) : mightCollide.getAgents().get(0);
                            for (int interState : collisionStates) {
                                if (!nonDominant.isForbidden(interState)) {
                                    nonDominant.retrain();
                                    nonDominant.addForbiddenState(interState);
                                }
                            }
                            break;
                        }
                        agent.step();
                    }
                }
                System.out.println();
            }
        }

        return !collisionDetected && AllDone(agents);
    }

    private static PotentialCollisionData mightCollidingAgents(int iAgent, List<GridAgent> agents) {
        PotentialCollisionData potentialCollision = new PotentialCollisionData();
        GridAgent agent = agents.get(iAgent);
        for (int iOtherAgent = 0; iOtherAgent < agents.size(); iOtherAgent++) {
            if (iOtherAgent != iAgent) {
                GridAgent otherAgent = agents.get(iOtherAgent);
                if (agent.isClose(otherAgent)) {
                    System.out.println("Agent " + agent.getID() + " and agent " + otherAgent.getID() + " are close!");
                    agent.expandBeliefs();
                    otherAgent.expandBeliefs();
                    Set<Integer> collisionStates = getCollisionStates(agent, otherAgent);
                    if (collisionStates.size() > 0) {
                        for (int collisionState : collisionStates) {
                            System.out.println("Agent " + agent.getID() + " and agent " + otherAgent.getID() + " may collide in " + agent.getGrid().parseState(collisionState));
                        }
                        potentialCollision.addAgent(agent);
                        potentialCollision.addAgent(otherAgent);
                        potentialCollision.addCollisionStates(collisionStates);
                        break;
                    }
                }
            }
        }

        return potentialCollision;
    }

    private static Set<Integer> getCollisionStates(GridAgent agent1, GridAgent agent2) {
        Set<Integer> agent1States = agent1.getPossibleStates();
        System.out.println("Agent " + agent1.getID() + " possible states: " + agent1States.stream().map(state -> agent1.getGrid().parseState(state)).toList().toString());
        Set<Integer> agent2States = agent2.getPossibleStates();
        System.out.println("Agent " + agent2.getID() + " possible states: " + agent2States.stream().map(state -> agent2.getGrid().parseState(state)).toList().toString());
        agent1States.retainAll(agent2States);
        return agent1States;
    }

    private static boolean checkCollision(GridAgent agent1, GridAgent agent2) {
        boolean isCollided = false;
        GridAgent nonDominant;
        Set<Integer> collisionStates = getCollisionStates(agent1, agent2);
        if (collisionStates.size() > 0) {
            nonDominant = agent1.distanceToGoal() < agent2.distanceToGoal() ? agent2 : agent1;
            for (int interState : collisionStates) {
                if (!nonDominant.isForbidden(interState)) {
                    isCollided = true;
                    nonDominant.retrain();
                    nonDominant.addForbiddenState(interState);
                    System.out.println("Agent " + agent1.getID() + " and agent " + agent2.getID() + " may collide in " + agent1.getGrid().parseState(interState));
                }
            }
        }
        return isCollided;
    }

    public static void main(String[] args) {
        JProf.getCurrentThreadCpuTimeSafe();
//        String sModelName = "straight_line_side_beacon_15x23";
//        String sModelName = "straight_line_side_beacon_9x15";
//        String sModelName = "short_hallway_side_beacon";
//        String sModelName = "two_paths_one_beacon";
//        String sModelName = "room";
//        String sModelName = "open_world_9_9_5";
        String sModelName = "open_world_5_5_0";
//        String sModelName = "test_grid";
        String sMethodName = "Perseus";
        int distanceThreshold = 2;
        int num_of_agents = 2;

        try{
            Logger.getInstance().setOutputStream( sModelName + "_" + sMethodName + ".txt" );
        }
        catch( Exception e ){
            System.err.println( e );
        }

        List<GridAgent> agents = new ArrayList<>();

        for (int i = 0; i < num_of_agents; i++) {
            GridAgent agent = new GridAgent(distanceThreshold);
            agents.add(agent);
        }

//        boolean succeeded = runForbidden(agents, sMethodName, sModelName, 1, 150);
        boolean succeeded = runMiniGrids(agents, sMethodName, sModelName, 150);

        if (succeeded) {
            System.out.println("Solution found!");
        }
        else {
            System.out.println("Solution was not found...");
        }

//        GridJointAgent a = new GridJointAgent();
////        String sModelName = "open_world_5_5_0";
////        String sModelName = "dummy_world";
//        String sModelName = "joint_test";
//        String sMethodName = "Perseus";
//        a.initRun(sModelName);
//        a.solve(sMethodName, 100, 20, 150);
    }
}
