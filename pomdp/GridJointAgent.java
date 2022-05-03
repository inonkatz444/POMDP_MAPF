package pomdp;

import pomdp.algorithms.AlgorithmsFactory;
import pomdp.algorithms.ValueIteration;
import pomdp.environments.JointBeaconDistanceGrid;
import pomdp.environments.POMDP;
import pomdp.utilities.ExecutionProperties;
import pomdp.utilities.InvalidModelFileFormatException;
import pomdp.utilities.JointGridLoader;
import pomdp.utilities.Logger;
import pomdp.utilities.datastructures.Function;

import java.io.IOException;
import java.util.*;

public class GridJointAgent {
    private JointBeaconDistanceGrid grid;
    private JointGridLoader p;

    private final double SUCCESS_REWARD = 10.0;
    private final double INTER_REWARD = -0.04;

    public GridJointAgent() {

    }

    public void solve(String methodName, double dTargetADR, int cMaxIterations, int maxSteps) {
        ValueIteration viAlgorithm = AlgorithmsFactory.getAlgorithm( methodName, grid );
        try{
            assert viAlgorithm != null;
            viAlgorithm.valueIteration( cMaxIterations, ExecutionProperties.getEpsilon(), dTargetADR );
            double dDiscountedReward = grid.computeAverageDiscountedReward( 200, maxSteps, viAlgorithm, true , ExecutionProperties.useHighLevelMultiThread() || ExecutionProperties.useMultiThread() );
            Logger.getInstance().log( "GridJointAgent", 0, "main", "ADR = " + dDiscountedReward );
        }

        catch( Exception e ){
            System.out.println( e );
            e.printStackTrace();
        }
        catch( Error err ){
            Runtime rtRuntime = Runtime.getRuntime();
            System.out.println( "GridJointAgent: " + err +
                    " allocated " + ( rtRuntime.totalMemory() - rtRuntime.freeMemory() ) / 1000000 +
                    " free " + rtRuntime.freeMemory() / 1000000 +
                    " max " + rtRuntime.maxMemory() / 1000000 );
            System.out.print( "Stack trace: " );
            err.printStackTrace();
        }
    }

    public Function load(String sFileName) throws InvalidModelFileFormatException, IOException {
        Function fTransition = p.load(sFileName);
        grid.getMDPValueFunction();
        grid.initBeliefStateFactory();
        return fTransition;
    }

    public void initRun(String sModelName) {
        grid = new JointBeaconDistanceGrid(2);
        p = new JointGridLoader(grid);
        try {
            Function singleTransitions = load(ExecutionProperties.getPath() + sModelName + ".POMDP");
            createTransitionsAndRewards(singleTransitions);
            verifyTransitions();
//            printGrid();
        }
        catch (InvalidModelFileFormatException | IOException e) {
            e.printStackTrace();
        }
        Runtime.getRuntime().gc();
    }

    private void verifyTransitions() {
        int iStartState = 0, iAction = 0, iEndState = 0, iObservation = 0;
        Iterator<Map.Entry<Integer,Double>> itNonZero = null;
        Map.Entry<Integer,Double> e = null;
        double dTr = 0.0, dSumTr = 0.0, dO = 0.0, dSumO = 0.0, dPr = 0.0, dSumPr = 0.0;
        boolean bFixed = false;
        int cStates = numOfSingleStates();
        int cActions = numOfSingleActions();

        bFixed = false;
        for( iStartState = 0 ; iStartState < cStates ; iStartState++ ){
            for( iAction = 0 ; iAction < cActions ; iAction++ ){
                dSumTr = 0.0;
                itNonZero = grid.getNonZeroTransitions( iStartState, iAction );
                while( itNonZero.hasNext() ){
                    e = itNonZero.next();
                    iEndState = e.getKey();
                    dTr = e.getValue();
                    dSumTr += dTr;
                }

                if( dSumTr == 0.0 ){
                    grid.setTransition( iStartState, iAction, iStartState, 1.0 );
                    dSumTr = 1.0;
                    bFixed = true;
                }

                if( Math.abs( dSumTr - 1.0 ) > 0.0001 )
                    System.out.println( "sum tr( " + grid.getStateName( iStartState ) + ", " + iAction + ", * ) = " + dSumTr );
            }
        }
        if( bFixed ){
            System.out.println( "Model file corrupted - needed to fix some transition values" );
        }

        if (grid.getStateToLocation() == null) {
            System.out.println( "Grid model missing rows and/or cols entries");
        }
    }

    public int numOfSingleStates() {
        return p.getNumOfSingleStates();
    }

    public int numOfSingleActions() {
        return p.getNumOfSingleActions();
    }

    private void createTransitionsAndRewards(Function singleTransitions) {
        int numOfAgents = grid.getNumOfAgents();
        double prob;
        double reward;

        int maxStateValue = numOfSingleStates()-1;
        List<Integer> stateValues = new ArrayList<>();

        for (int i = 0; i < numOfAgents; i++) {
            stateValues.add(0);
        }

        int stateChecksum;
        int encodedState;
        boolean stateDone = false;
        boolean stateChange;
        int stateR;

        int startState = grid.encodeStates(grid.getAllStartStates());
        int endState = grid.encodeStates(grid.getAllEndStates());

        grid.setRewardType( POMDP.RewardType.StateActionState );
        grid.addTerminalState(endState);

        int maxActionValue = numOfSingleActions()-1;
        List<Integer> actionValues = new ArrayList<>();

        for (int i = 0; i < numOfAgents; i++) {
            actionValues.add(0);
        }

        int actionChecksum;
        int encodedAction;
        boolean actionDone = false;
        boolean actionChange;
        int actionR;


        List<Integer> nextStateValues = new ArrayList<>();

        for (int i = 0; i < numOfAgents; i++) {
            nextStateValues.add(0);
        }

        int nextStateChecksum;
        int encodedNextState;
        boolean nextStateDone = false;
        boolean nextStateChange;
        int nextStateR;

        while (!stateDone) {
            stateChecksum = 0;
            for (int i = 0; i < numOfAgents; i++) {
                stateChecksum += stateValues.get(i);
            }
            if (stateChecksum == numOfAgents*(maxStateValue)){
                stateDone = true;
            }

            encodedState = grid.encodeStates(stateValues);

            // init start state:
            grid.setStartStateProb(encodedState, encodedState == startState ? 1.0 : 0.0);

            actionDone = false;

            while (!actionDone) {
                actionChecksum = 0;
                for (int i = 0; i < numOfAgents; i++) {
                    actionChecksum += actionValues.get(i);
                }
                if (actionChecksum == numOfAgents*(maxActionValue)){
                    actionDone = true;
                }

                encodedAction = grid.encodeActions(actionValues);

                nextStateDone = false;

                while (!nextStateDone) {
                    nextStateChecksum = 0;
                    for (int i = 0; i < numOfAgents; i++) {
                        nextStateChecksum += nextStateValues.get(i);
                    }
                    if (nextStateChecksum == numOfAgents*(maxStateValue)){
                        nextStateDone = true;
                    }

                    encodedNextState = grid.encodeStates(nextStateValues);

                    // Insert here
                    prob = 1;
                    for (int iAgent = 0; iAgent < numOfAgents; iAgent++) {
                        prob *= singleTransitions.valueAt(stateValues.get(iAgent), actionValues.get(iAgent), nextStateValues.get(iAgent));
                    }
                    if (prob != 0) {
                        reward = encodedNextState == endState ? SUCCESS_REWARD : numOfAgents*INTER_REWARD;
                        grid.setTransition(encodedState, encodedAction, encodedNextState, prob);
                        grid.setReward(encodedState, encodedAction, encodedNextState, reward);
                        grid.setMinimalReward(encodedAction, reward);
                    }

                    nextStateChange = true;
                    nextStateR = 0;
                    while (nextStateChange && nextStateR < numOfAgents) {
                        nextStateValues.set(nextStateR, nextStateValues.get(nextStateR)+1);
                        if (nextStateValues.get(nextStateR) > maxStateValue) {
                            nextStateValues.set(nextStateR, 0);
                        }
                        else {
                            nextStateChange = false;
                        }
                        nextStateR++;
                    }
                }

                actionChange = true;
                actionR = 0;
                while (actionChange && actionR < numOfAgents) {
                    actionValues.set(actionR, actionValues.get(actionR)+1);
                    if (actionValues.get(actionR) > maxActionValue) {
                        actionValues.set(actionR, 0);
                    }
                    else {
                        actionChange = false;
                    }
                    actionR++;
                }
            }

            stateChange = true;
            stateR = 0;
            while (stateChange && stateR < numOfAgents) {
                stateValues.set(stateR, stateValues.get(stateR)+1);
                if (stateValues.get(stateR) > maxStateValue) {
                    stateValues.set(stateR, 0);
                }
                else {
                    stateChange = false;
                }
                stateR++;
            }
        }

        grid.initStoredRewards();
    }

}
