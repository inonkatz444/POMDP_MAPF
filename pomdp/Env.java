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

    public static boolean runAll(List<GridAgent> agents, String sMethodName, String sModelName, int maxIterations, int maxSteps, int distanceThreshold) {
        int iStep = 0, n = agents.size();
        boolean collisionDetected = true;

        for (int iIteration = 0; (iIteration < maxIterations) && collisionDetected; iIteration++) {
            collisionDetected = false;
            agents.forEach(agent -> {
                agent.initRun(sModelName);
                agent.solve(sMethodName, 100.0, 22);
            });
            agents.forEach(agent -> System.out.println("Agent " + agent.getID() + " starts at " + agent.getStartStateString()));
            for (iStep = 0; (iStep < maxSteps) && !collisionDetected; iStep++) {
                for (int i = 0; i < agents.size() && !collisionDetected; i++) {
                    GridAgent agent = agents.get(i);
                    if (!agent.isDone()) {
                        agent.step();
                        collisionDetected = isCollisionAvailable(i, agents, distanceThreshold);
                    }
                }
                System.out.println();
            }
        }

        return !collisionDetected;
    }

    private static boolean isCollisionAvailable(int iAgent, List<GridAgent> agents, int distanceThreshold) {
        boolean collisionDetected = false;
        GridAgent agent = agents.get(iAgent);
        for (int iOtherAgent = 0; iOtherAgent < agents.size(); iOtherAgent++) {
            if (iOtherAgent != iAgent) {
                GridAgent otherAgent = agents.get(iOtherAgent);
                if (agent.isClose(otherAgent, distanceThreshold)) {
                    agent.expandBeliefs();
                    otherAgent.expandBeliefs();
                    System.out.println("Agent " + agent.getID() + " and agent " + otherAgent.getID() + " are close!");
                    collisionDetected = collisionDetected || checkCollision(agent, otherAgent);
                }
            }
        }

        return collisionDetected;
    }

    private static boolean checkCollision(GridAgent agent1, GridAgent agent2) {
        boolean isCollided = false;
        Set<Integer> agent1States = agent1.getPossibleStates();
        System.out.println("Agent " + agent1.getID() + " possible states: " + agent1States.stream().map(state -> agent1.getGrid().parseState(state)).toList().toString());
        Set<Integer> agent2States = agent2.getPossibleStates();
        System.out.println("Agent " + agent2.getID() + " possible states: " + agent2States.stream().map(state -> agent2.getGrid().parseState(state)).toList().toString());
        agent1States.retainAll(agent2States);
        for (int interState : agent1States) {
            agent1.addForbiddenState(interState);
            agent2.addForbiddenState(interState);
            isCollided = true;
            System.out.println("Agent " + agent1.getID() + " and agent " + agent2.getID() + " may collide in " + agent1.getGrid().parseState(interState));
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

        GridAgent agent1 = new GridAgent(sModelName, multiAgent, distanceThreshold, 134, 25);
        GridAgent agent2 = new GridAgent(sModelName, multiAgent, distanceThreshold, 25, 134);

        List<GridAgent> agents = new ArrayList<>();
        agents.add(agent1);
        agents.add(agent2);
        boolean succeeded = runAll(agents, sMethodName, sModelName,2, 150, distanceThreshold);
        System.out.println(succeeded);
    }
}
