package pomdp;

import pomdp.algorithms.AlgorithmsFactory;
import pomdp.algorithms.PolicyStrategy;
import pomdp.algorithms.ValueIteration;
import pomdp.environments.BeaconDistanceGrid;
import pomdp.environments.Grid;
import pomdp.utilities.*;

import java.io.IOException;

public class GridAgent {
    private final Grid grid;
    private PolicyStrategy policy;

    public GridAgent(String sModelName) {
        if (sModelName.equals("two_paths_one_beacon")) {
            grid = new BeaconDistanceGrid();
        }
        else {
            grid = new Grid();
        }
    }

    public void load(String fileName) throws InvalidModelFileFormatException, IOException {
        grid.load(fileName);
    }

    public BeliefState getInitialBeliefState() {
        return grid.getBeliefStateFactory().getInitialBeliefState();
    }

    public BeliefState step(BeliefState bs) {
        int action = policy.getAction(bs);
        int observation = grid.observe(bs, action);
        return bs.nextBeliefState(action, observation);
    }

    public PolicyStrategy solve(String methodName, double dTargetADR, int cMaxIterations) {
        if( methodName.equals( "QMDP" ) ){
            MDPValueFunction vfQMDP = grid.getMDPValueFunction();
            vfQMDP.persistQValues( true );
            vfQMDP.valueIteration( 100, 0.001 );
            double dDiscountedReward = grid.computeAverageDiscountedReward( 1000, 100, vfQMDP );
            Logger.getInstance().log( "POMDPSolver", 0, "main", "ADR = " + dDiscountedReward );
            this.policy = vfQMDP;
            return vfQMDP;
        }

        ValueIteration viAlgorithm = AlgorithmsFactory.getAlgorithm( methodName, grid );
        try{
            assert viAlgorithm != null;
            viAlgorithm.valueIteration( cMaxIterations, ExecutionProperties.getEpsilon(), dTargetADR );
            double dDiscountedReward = grid.computeAverageDiscountedReward( 500, 150, viAlgorithm );
            Logger.getInstance().log( "POMDPSolver", 0, "main", "ADR = " + dDiscountedReward );
            this.policy = viAlgorithm;
        }

        catch( Exception e ){
            System.out.println( e );
            e.printStackTrace();
        }
        catch( Error err ){
            Runtime rtRuntime = Runtime.getRuntime();
            System.out.println( "POMDPSolver: " + err +
                    " allocated " + ( rtRuntime.totalMemory() - rtRuntime.freeMemory() ) / 1000000 +
                    " free " + rtRuntime.freeMemory() / 1000000 +
                    " max " + rtRuntime.maxMemory() / 1000000 );
            System.out.print( "Stack trace: " );
            err.printStackTrace();
        }
        return viAlgorithm;
    }
}
