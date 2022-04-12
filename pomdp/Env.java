package pomdp;

import pomdp.algorithms.AlgorithmsFactory;
import pomdp.algorithms.ValueIteration;
import pomdp.environments.Grid;
import pomdp.utilities.*;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class Env {

    public static boolean runAll(List<GridAgent> agents, String sMethodName, String sModelName, int maxIterations, int maxSteps) {
        int iStep, n = agents.size();
        boolean collisionDetected = true;

        for (int iIteration = 0; (iIteration < maxIterations) && collisionDetected; iIteration++) {
            collisionDetected = false;
            for (GridAgent agent : agents) {
                if (agent.needsRetrain()) {
                    agent.initRun(sModelName);
                    agent.solve(sMethodName, 100.0, 18, maxSteps);
                }
            }
            for (GridAgent agent : agents) {
                System.out.println("Agent " + agent.getID() + " forbidden states: " + agent.getForbiddenStates().stream().map(s -> agents.get(0).getGrid().parseState(s)).toList().toString());
            }
            agents.forEach(agent -> System.out.println("Agent " + agent.getID() + " starts at " + agent.getStartStateString()));
            for (iStep = 0; (iStep < maxSteps) && !collisionDetected && agents.stream().map(GridAgent::isDone).toList().contains(false); iStep++) {
                for (int i = 0; i < n && !collisionDetected; i++) {
                    GridAgent agent = agents.get(i);
                    if (!agent.isDone()) {
                        agent.step();
                        collisionDetected = isCollisionAvailable(i, agents);
                    }
                }
                System.out.println();
            }
        }

        return !collisionDetected && !agents.stream().map(GridAgent::isDone).toList().contains(false);
    }

    private static boolean isCollisionAvailable(int iAgent, List<GridAgent> agents) {
        boolean collisionDetected = false;
        boolean mainAgentExpanded = false;
        GridAgent agent = agents.get(iAgent);
        for (int iOtherAgent = 0; iOtherAgent < agents.size(); iOtherAgent++) {
            if (iOtherAgent != iAgent) {
                GridAgent otherAgent = agents.get(iOtherAgent);
                if (agent.isClose(otherAgent)) {
                    System.out.println("Agent " + agent.getID() + " and agent " + otherAgent.getID() + " are close!");
                    if (!mainAgentExpanded) {
                        agent.expandBeliefs();
                        mainAgentExpanded = true;
                    }
                    otherAgent.expandBeliefs();
                    collisionDetected = collisionDetected || checkCollision(agent, otherAgent);
                }
            }
        }

        return collisionDetected;
    }

    private static boolean checkCollision(GridAgent agent1, GridAgent agent2) {
        boolean isCollided = false;
        GridAgent forbiddenAgent;
        Set<Integer> agent1States = agent1.getPossibleStates();
        System.out.println("Agent " + agent1.getID() + " possible states: " + agent1States.stream().map(state -> agent1.getGrid().parseState(state)).toList().toString());
        Set<Integer> agent2States = agent2.getPossibleStates();
        System.out.println("Agent " + agent2.getID() + " possible states: " + agent2States.stream().map(state -> agent2.getGrid().parseState(state)).toList().toString());
        agent1States.retainAll(agent2States);
        if (agent1States.size() > 0) {
            forbiddenAgent = agent1.distanceToGoal() < agent2.distanceToGoal() ? agent1 : agent2;
            for (int interState : agent1States) {
                forbiddenAgent.addForbiddenState(interState);
                forbiddenAgent.retrain();
                isCollided = true;
                System.out.println("Agent " + agent1.getID() + " and agent " + agent2.getID() + " may collide in " + agent1.getGrid().parseState(interState));
            }
        }
        return isCollided;
    }

    public static void main(String[] args) {
        JProf.getCurrentThreadCpuTimeSafe();
//        String sModelName = "straight_line_side_beacon_15x23";
//        String sModelName = "straight_line_side_beacon_9x15";
//        String sModelName = "short_hallway_side_beacon";
        String sModelName = "two_paths_one_beacon";
//        String sModelName = "test_grid";
        String sMethodName = "Perseus";
        boolean multiAgent = true;
        int distanceThreshold = 3;

        try{
            Logger.getInstance().setOutputStream( sModelName + "_" + sMethodName + ".txt" );
        }
        catch( Exception e ){
            System.err.println( e );
        }

        GridAgent agent1 = new GridAgent(distanceThreshold, 134, 25);
        GridAgent agent2 = new GridAgent(distanceThreshold, 25, 134);

        List<GridAgent> agents = new ArrayList<>();
        agents.add(agent1);
        agents.add(agent2);
        boolean succeeded = runAll(agents, sMethodName, sModelName,2, 150);

        if (succeeded) {
            System.out.println("Solution found!");
        }
        else {
            System.out.println("Solution was not found...");
        }
    }
}
