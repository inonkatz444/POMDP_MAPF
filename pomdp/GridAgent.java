package pomdp;

import pomdp.algorithms.AlgorithmsFactory;
import pomdp.algorithms.PolicyStrategy;
import pomdp.algorithms.ValueIteration;
import pomdp.environments.BeaconDistanceGrid;
import pomdp.environments.Grid;
import pomdp.environments.POMDP;
import pomdp.utilities.*;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class GridAgent {
    private final Grid grid;
    private PolicyStrategy policy;
    private Map<Integer, Boolean> forbiddenStates;
    private BeliefState currentBelief;
    private int currentState, currentNextState, currentObservation;
    private int cSameStates;

    private final double SUCCESS_REWARD = 10.0;
    private final double INTER_REWARD = -0.04;

    public GridAgent(String sModelName, boolean multiAgent) {
        if (sModelName.equals("two_paths_one_beacon")) {
            grid = new BeaconDistanceGrid(multiAgent);
        }
        else {
            grid = new Grid(multiAgent);
        }

        forbiddenStates = new HashMap<>();
    }

    public void clearForbiddenStates() {
        forbiddenStates.clear();
    }

    public void addForbiddenState(int iState) {
        forbiddenStates.put(iState, true);
    }

    public void addForbiddenStates(List<Integer> iStates) {
        iStates.forEach(iState -> forbiddenStates.put(iState, true));
    }

    public boolean isForbidden(int iState) {
        return forbiddenStates.getOrDefault(iState, false);
    }

    public void load(String fileName) throws InvalidModelFileFormatException, IOException {
        grid.load(fileName);
    }

    public void initAgentWise(int startState, int endState) throws InvalidModelFileFormatException{
        grid.setRewardType( POMDP.RewardType.StateActionState );
        grid.addTerminalState(endState);

        if (startState < 0 || startState >= grid.getStateCount()) {
            throw new InvalidModelFileFormatException("Start: must be valid state - between 0 and " + grid.getStateCount());
        }

        if (endState < 0 || endState >= grid.getStateCount()) {
            throw new InvalidModelFileFormatException("Terminal: must be valid state - between 0 and " + grid.getStateCount());
        }

        for (int iStartState = 0; iStartState < grid.getStateCount(); iStartState++) {
            grid.setStartStateProb(iStartState, iStartState == startState ? 1.0 : 0.0);

            for (int iAction = 0; iAction < grid.getActionCount(); iAction++) {

                for (int iEndState = 0; iEndState < grid.getStateCount(); iEndState++) {
                    grid.setReward( iStartState, iAction, iEndState, iEndState == endState ? SUCCESS_REWARD : INTER_REWARD );
                    grid.setMinimalReward( iAction, iEndState == endState ? SUCCESS_REWARD : INTER_REWARD );
                }
            }
        }

        grid.initStoredRewards();
    }

    public BeliefState getInitialBeliefState() {
        return grid.getBeliefStateFactory().getInitialBeliefState();
    }

    public boolean step() {
        boolean done = false;
        BeliefState bsNext;
        int iAction = policy.getAction(currentBelief);
        if( iAction == -1 )
            throw new Error( "Could not find optimal action for bs " + currentBelief );

        currentNextState = grid.execute( iAction, currentState );
        currentObservation = grid.observe( iAction, currentNextState );

        done = grid.endADR(currentNextState, 0);

        bsNext = currentBelief.nextBeliefState(iAction, currentObservation);

        if( currentState != currentNextState )
            cSameStates = 0;
        else
            cSameStates++;

        done = done || (bsNext == null || ( bsNext.valueAt( currentNextState ) == 0 || ( cSameStates > 10 ) ));

        currentState = currentNextState;
        currentBelief.release();
        currentBelief = bsNext;

        return done;
    }

    public boolean isDone() {
        return grid.isTerminalState(currentState);
    }

    public int chooseStartState() {
        return grid.chooseStartState();
    }

    public BeliefState getCurrentBelief() {
        return currentBelief;
    }

    public int getStateCount() {
        return grid.getStateCount();
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
            // TODO: include forbidden states in the solution
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

    public void initRun() {
        currentBelief = getInitialBeliefState();
        currentState = chooseStartState();
        currentObservation = 0;
        currentNextState = 0;
        cSameStates = 0;
    }
}
