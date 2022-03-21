package pomdp;

import pomdp.algorithms.AlgorithmsFactory;
import pomdp.algorithms.ValueIteration;
import pomdp.environments.Grid;
import pomdp.utilities.*;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class Env {

    public static boolean runAll(List<GridAgent> agents, String sMethodName, int maxIterations, int maxSteps) {
        int iStep = 0, n = agents.size();
        boolean collisionDetected = true;

        for (int iIteration = 0; (iIteration < maxIterations) && collisionDetected; iIteration++) {
            collisionDetected = false;
            agents.forEach(agent -> {
                agent.initRun();
                agent.solve(sMethodName, 100.0, 25);
            });
            for (iStep = 0; (iStep < maxSteps) && !collisionDetected; iStep++) {
                for (GridAgent agent : agents) {
                    if (!agent.isDone()) {
                        agent.step();
                    }
                }
                collisionDetected = isCollisionAvailable(agents);
            }
        }

        return !collisionDetected;
    }

    private static boolean isCollisionAvailable(List<GridAgent> agents) {
        int numAgentsInState;
        int firstPresent = 0;
        boolean collisionDetected = false;
        for (int iState = 0; iState < agents.get(0).getStateCount(); iState++) {
            numAgentsInState = 0;
            for (int iAgent = 0; iAgent < agents.size(); iAgent++) {
                if (agents.get(iAgent).getCurrentBelief().valueAt(iState) > 0) {
                    numAgentsInState++;
                    if (numAgentsInState >= 2) {
                        agents.get(iAgent).addForbiddenState(iState);
                        collisionDetected = true;
                    }
                    else {
                        firstPresent = iAgent;
                    }
                }
            }

            if (numAgentsInState >= 2) {
                agents.get(firstPresent).addForbiddenState(iState);
            }
        }

        return collisionDetected;
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

        try{
            Logger.getInstance().setOutputStream( sModelName + "_" + sMethodName + ".txt" );
        }
        catch( Exception e ){
            System.err.println( e );
        }

        GridAgent agent = new GridAgent(sModelName, multiAgent);
        try {
            agent.load(ExecutionProperties.getPath() + sModelName + ".POMDP");
            if (multiAgent) {
                agent.initAgentWise(117, 8);
            }
        } catch (InvalidModelFileFormatException | IOException e) {
            e.printStackTrace();
        }

        List<GridAgent> agents = new ArrayList<>();
        agents.add(agent);
        boolean succeeded = runAll(agents, sMethodName, 1, 150);
        System.out.println(succeeded);
    }
}
