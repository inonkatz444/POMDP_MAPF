package pomdp;

import pomdp.algorithms.AlgorithmsFactory;
import pomdp.algorithms.PolicyStrategy;
import pomdp.algorithms.ValueIteration;
import pomdp.environments.BeaconDistanceGrid;
import pomdp.environments.JointBeaconDistanceGrid;
import pomdp.environments.POMDP;
import pomdp.utilities.*;
import pomdp.utilities.datastructures.Function;

import java.io.IOException;
import java.util.*;

public class GridJointAgent {
    private JointBeaconDistanceGrid grid;
    private JointGridLoader p;
    private List<Integer> END_STATES;
    private List<GridAgent> agents;
    private BeliefState currentJointBelief;
    private PolicyStrategy policy;
    private int currentState;
    private int cSameStates;
    private int numOfAgents;
    private List<BeaconDistanceGrid> singleGrids;
    private boolean[] isDone;

    private final int NUM_OF_ACTIONS = 5;
    private final double DISCOUNT_FACTOR = 0.99;
    private final double INTER_REWARD = -0.04;
    private final Map<Integer, String> idxToSSingleAction = new HashMap<>();

    public GridJointAgent() {
        idxToSSingleAction.put(0, "n");
        idxToSSingleAction.put(1, "s");
        idxToSSingleAction.put(2, "e");
        idxToSSingleAction.put(3, "w");
        idxToSSingleAction.put(4, "noop");
        END_STATES = new ArrayList<>();
    }

    public void solve(String methodName, double dTargetADR, int cMaxIterations, int maxSteps) {
        ValueIteration viAlgorithm = AlgorithmsFactory.getAlgorithm( methodName, grid );
        try{
            assert viAlgorithm != null;
            viAlgorithm.valueIteration( cMaxIterations, ExecutionProperties.getEpsilon(), dTargetADR );
            double dDiscountedReward = grid.computeAverageDiscountedReward( 200, maxSteps, viAlgorithm, true , ExecutionProperties.useHighLevelMultiThread() || ExecutionProperties.useMultiThread() );
            Logger.getInstance().log( "GridJointAgent", 0, "main", "ADR = " + dDiscountedReward );
            this.policy = viAlgorithm;
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

//    public void initRun(String sModelName) {
//        grid = new JointBeaconDistanceGrid(2);
//        p = new JointGridLoader(grid);
//        try {
//            Function singleTransitions = load(ExecutionProperties.getPath() + sModelName + ".POMDP");
//            int start_state = grid.encodeStates(grid.getAllStartStates());
//            double[] belief_values = new double[grid.getStateCount()];
//            belief_values[start_state] = 1.0;
//            currentJointBelief = grid.getBeliefStateFactory().newBeliefState(belief_values);
//            createTransitionsAndRewards(singleTransitions);
//            verifyTransitions();
////            printGrid();
//        }
//        catch (InvalidModelFileFormatException | IOException e) {
//            e.printStackTrace();
//        }
//        Runtime.getRuntime().gc();
//    }

    public void initRun(PotentialCollisionData data) {
        this.agents = data.getAgents();
        singleGrids = agents.stream().map(GridAgent::getGrid).toList();
        BeaconDistanceGrid singleGrid = singleGrids.get(0);
        isDone = new boolean[agents.size()];
        int rows, cols;
//        Function singleTransitions;
        grid = new JointBeaconDistanceGrid(agents);
        numOfAgents = agents.size();
        Point[] boundaries = singleGrid.getBoundaryPoints(agents.stream().map(GridAgent::getCurrentBelief).toList(), data.getCollisionStates());
        rows = boundaries[1].first() - boundaries[0].first() + 1;
        cols = boundaries[1].second() - boundaries[0].second() + 1;
        grid.setRows(rows);
        grid.setCols(cols);
        grid.setNumOfSingleActions(NUM_OF_ACTIONS);
        grid.setDiscountFactor(DISCOUNT_FACTOR);
        grid.setOrigin(boundaries[0]);
        grid.setName(singleGrid.getName() + "_" + grid.getOrigin());

        List<Point> remainHoles = new ArrayList<>();
        for (Point hole : singleGrid.getHoles()) {
            if (hole.inBound(grid)) {
                remainHoles.add(hole.relativeTo(grid.getOrigin()));
            }
        }
        grid.setHoles(remainHoles);

        grid.setNumOfSingleStates(rows*cols - remainHoles.size() + 1);  // +1 for DONE
        grid.setStateCount( (int)Math.pow(grid.getNumOfSingleStates(), grid.getNumOfAgents()));
        System.out.print( "|S| = " + grid.getStateCount() );

        setActions();

        grid.setBeacons(singleGrid.getBeacons());

        int numOfSingleObservations = agents.get(0).getGrid().getObservationCount();
        grid.setNumOfSingleObservations(numOfSingleObservations);
        grid.setObservationCount((int)Math.pow(numOfSingleObservations, numOfAgents));
        grid.initDynamicsFunctions();
        grid.setMaxDist(singleGrid.getMaxDist());

        try {
            grid.initGrid();
        } catch (InvalidModelFileFormatException e) {
            e.printStackTrace();
        }

        grid.setRewardType( POMDP.RewardType.StateActionState );
        grid.initStoredRewards();

//        int[] transitionDims = {grid.getNumOfSingleStates(), grid.getNumOfSingleActions(), grid.getNumOfSingleStates()};
//        singleTransitions = new SparseTabularFunction( transitionDims );
//        Map.Entry<Integer, Double> eTrans;
//        int iEndState;
//        double dTrans;
//        boolean found_trans;
//        for (int iStartState = 0; iStartState < singleGrid.getStateCount(); iStartState++) {
//            if (iStartState == singleGrid.DONE || singleGrid.stateToLocation(iStartState).inBound(grid)) {
//                for (int iAction = 0; iAction < singleGrid.getActionCount(); iAction++) {
//                    found_trans = false;
//                    for (int iAgent = 0; iAgent < numOfAgents; iAgent++) {
//                        Iterator<Map.Entry<Integer, Double>> test_it = singleGrids.get(iAgent).getNonZeroTransitions(iStartState, iAction);
//                        iEndState = test_it.next().getKey();
//                        if (iEndState == 25) {
//                            System.out.println();
//                        }
//                        if (iEndState != singleGrids.get(iAgent).DONE) {
//                            for (Iterator<Map.Entry<Integer, Double>> it = singleGrids.get(iAgent).getNonZeroTransitions(iStartState, iAction); it.hasNext(); ) {
//                                eTrans = it.next();
//                                iEndState = eTrans.getKey();
//                                dTrans = eTrans.getValue();
//                                if (iEndState == singleGrids.get(iAgent).DONE || !singleGrids.get(iAgent).stateToLocation(iEndState).inBound(grid) || grid.isInBorder(toSmallGrid(iStartState))) {
//                                    iEndState = singleGrids.get(iAgent).DONE;
//                                }
//                                singleTransitions.setValue(toSmallGrid(iStartState), iAction, toSmallGrid(iEndState),
//                                        singleTransitions.valueAt(toSmallGrid(iStartState), iAction, toSmallGrid(iEndState)) + dTrans);
//                            }
//                            found_trans = true;
//                            break;
//                        }
//                    }
//                    if (!found_trans) {
//                        singleTransitions.setValue(toSmallGrid(iStartState), iAction, grid.SINGLE_DONE, 1.0);
//                    }
//                }
//            }
//        }

        END_STATES = calculateIntermediateEndStates();

        grid.initBeliefStateFactory();

        currentJointBelief = joinBeliefs(agents.stream().map(GridAgent::getCurrentBelief).toList());
        for (GridAgent agent : agents) {
            if (!agent.getMDPValueFunction().hasConverged()) {
                System.out.println("Solving Value Iteration for agent " + agent.getID() + ":");
                agent.getMDPValueFunction().valueIteration(30, ExecutionProperties.getEpsilon());
            }
        }

        for (int i = 0; i < numOfAgents; i++) {
            isDone[i] = false;
        }

        setStartStateProb();

        System.out.println(grid.getName());
        System.out.println("Created grid with " + rows + " rows and " + cols + " columns");
    }

    private List<Integer> calculateIntermediateEndStates() {
        int maxStateValue = numOfSingleStates()-1;
        List<Integer> stateValues = new ArrayList<>();
        boolean[] canDone = new boolean[numOfAgents];
        int numOfTerminalAgents;

        for (int i = 0; i < numOfAgents; i++) {
            stateValues.add(0);
            canDone[i] = agents.get(i).canDone(grid);
        }

        int stateChecksum;
        int encodedState;
        boolean stateDone = false;
        boolean stateChange;
        int stateR;

        while (!stateDone) {
            stateChecksum = 0;
            for (int i = 0; i < numOfAgents; i++) {
                stateChecksum += stateValues.get(i);
            }
            if (stateChecksum == numOfAgents * (maxStateValue)) {
                stateDone = true;
            }

            encodedState = grid.encodeStates(stateValues);
            numOfTerminalAgents = 0;
            for (int iAgent = 0; iAgent < numOfAgents; iAgent++) {
                if (stateValues.get(iAgent) == grid.SINGLE_DONE) {
                    numOfTerminalAgents++;
                }
                else if (!canDone[iAgent] && grid.isInBorder(stateValues.get(iAgent)) && agents.get(iAgent).getGrid().tr(stateValues.get(iAgent), -1, agents.get(iAgent).getGrid().DONE) == 0) {
                    numOfTerminalAgents++;
                }
            }
            if (numOfTerminalAgents >= numOfAgents - 1) {
                END_STATES.add(encodedState);
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

        END_STATES.forEach(s -> grid.addTerminalState(s));

        return END_STATES;
    }

    private BeliefState joinBeliefs(List<BeliefState> singleBeliefs) {
        List<double[]> newBSValues = new ArrayList<>();
        int iState;
        for (BeliefState bs : singleBeliefs) {
            double[] newBS = new double[grid.getNumOfSingleStates()];
            for (Map.Entry<Integer, Double> iStateD : bs.getNonZeroEntries()) {
                iState = iStateD.getKey();
                if (iState != singleGrids.get(0).DONE) {
                    Point loc = singleGrids.get(0).stateToLocation(iStateD.getKey());
                    int newState = grid.locationToState(loc);
                    newBS[newState] = iStateD.getValue();
                }
            }
            newBSValues.add(newBS);
        }

        double[] jointProbs = new double[(int)Math.pow(grid.getNumOfSingleStates(), numOfAgents)];

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

        while (!stateDone) {
            stateChecksum = 0;
            for (int i = 0; i < numOfAgents; i++) {
                stateChecksum += stateValues.get(i);
            }
            if (stateChecksum == numOfAgents * (maxStateValue)) {
                stateDone = true;
            }

            double dBelief = 1.0;
            for (int i = 0; i < numOfAgents; i++) {
                dBelief *= newBSValues.get(i)[stateValues.get(i)];
            }
            encodedState = grid.encodeStates(stateValues);
            jointProbs[encodedState] = dBelief;

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

        jointProbs[grid.getStateCount()-1] = 0;

        return grid.getBeliefStateFactory().newBeliefState(jointProbs);
    }

    private int toSmallGrid(int iState) {
        if (iState == singleGrids.get(0).DONE) {
            return grid.SINGLE_DONE;
        }
        return grid.locationToState(singleGrids.get(0).stateToLocation(iState));
    }

    private int toBigGrid(int iState) {
        if (iState == grid.SINGLE_DONE) {
            return singleGrids.get(0).DONE;
        }
        return singleGrids.get(0).locationToState(grid.stateToLocation(iState).add(grid.getOrigin()));
    }

    private void setActions() {
        int maxActionValue = grid.getNumOfSingleActions()-1;
        List<Integer> actionValues = new ArrayList<>();

        for (int i = 0; i < numOfAgents; i++) {
            actionValues.add(0);
        }

        int actionChecksum;
        boolean actionDone = false;
        boolean actionChange;
        int actionR;
        StringBuilder sAction;

        while (!actionDone) {
            actionChecksum = 0;
            for (int i = 0; i < numOfAgents; i++) {
                actionChecksum += actionValues.get(i);
            }
            if (actionChecksum == numOfAgents*(maxActionValue)){
                actionDone = true;
            }

            // add actions here
            sAction = new StringBuilder("(");
            for (int i = 0; i < numOfAgents; i++) {
                sAction.append(idxToSSingleAction.get(actionValues.get(i))).append(", ");
            }
            sAction.delete(sAction.length()-2, sAction.length());
            sAction.append(")");
            grid.addAction(sAction.toString());
            //

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
        System.out.print( "|A| = " + grid.getActionCount() );
        System.out.println();

    }

    public boolean step() {
        int currentNextState, currentObservation;
        BeliefState bsNext;
        int iAction = policy.getAction(currentJointBelief);
        if( iAction == -1 )
            throw new Error( "Could not find optimal action for bs " + currentJointBelief );

        currentNextState = grid.execute( iAction, currentState );
        currentObservation = grid.observe( iAction, currentNextState );

        boolean done = grid.endADR(currentNextState, 0);

        bsNext = currentJointBelief.nextBeliefState(iAction, currentObservation);

        if( currentState != currentNextState )
            cSameStates = 0;
        else
            cSameStates++;

        List<Integer> iActions = grid.decodeAction(iAction);
        List<Integer> iNextStates = grid.decodeState(currentNextState);
        List<Integer> iObservations = grid.decodeObservation(currentObservation);

        for (int iAgent = 0; iAgent < numOfAgents; iAgent++) {
            if (!isDone[iAgent]) {
                isDone[iAgent] = agents.get(iAgent).step(iActions.get(iAgent), toBigGrid(iNextStates.get(iAgent)), iObservations.get(iAgent));
            }
        }

        done = done || (bsNext == null || ( bsNext.valueAt( currentNextState ) == 0 || ( cSameStates > 10 ) || !agents.stream().map(GridAgent::isDone).toList().contains(false)));

        currentState = currentNextState;
        currentJointBelief.release();
        currentJointBelief = bsNext;

        return done;
    }

    public int numOfSingleStates() {
        return grid.getNumOfSingleStates();
    }

    public int numOfSingleActions() {
        return grid.getNumOfSingleActions();
    }

    private void setStartStateProb() {

        for (int iState = 0; iState < grid.getStateCount(); iState++) {
            grid.setStartStateProb(iState, currentJointBelief.valueAt(iState));
        }

        currentState = grid.encodeStates(agents.stream().map(a -> toSmallGrid(a.getCurrentState())).toList());
    }
}
