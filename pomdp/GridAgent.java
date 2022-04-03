package pomdp;

import pomdp.algorithms.AlgorithmsFactory;
import pomdp.algorithms.PolicyStrategy;
import pomdp.algorithms.ValueIteration;
import pomdp.environments.BeaconDistanceGrid;
import pomdp.environments.Grid;
import pomdp.environments.POMDP;
import pomdp.utilities.*;

import java.io.IOException;
import java.util.*;

public class GridAgent {
    private static int autoInc = 0;
    private BeaconDistanceGrid grid;
    private PolicyStrategy policy;
    private Map<Integer, Boolean> forbiddenStates;
    private BeliefState currentBelief;
    private int currentState, currentNextState, currentObservation;
    private int cSameStates;
    private int distanceThreshold;
    private final int id;
    private List<BeliefState> expandedBeliefs;
    private final int startState, endState;

    private final double SUCCESS_REWARD = 10.0;
    private final double INTER_REWARD = -0.04;

    public GridAgent(String sModelName, boolean multiAgent, int distanceThreshold, int startState, int endState) {
        // The forbidden states kept in the POMDP
        grid = new BeaconDistanceGrid(multiAgent);

        forbiddenStates = new HashMap<>();
        this.distanceThreshold = distanceThreshold;
        id = autoInc++;
        expandedBeliefs = new ArrayList<>();
        this.startState = startState;
        this.endState = endState;
    }

    public int getID() {
        return id;
    }

    public String getStartStateString() {
        return grid.parseState(startState);
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

    public List<BeliefState> expandBeliefs() {
        List<BeliefState> beliefs = new ArrayList<>();
        Queue<BeliefState> temp = new ArrayDeque<>();
        beliefs.add(currentBelief);
        for (int iExpandStep = 1; iExpandStep <= distanceThreshold; iExpandStep++) {
            temp.addAll(beliefs);
            while (!temp.isEmpty()) {
                BeliefState belief = temp.remove();
                beliefs.addAll(belief.computeSuccessors());
            }
        }
        expandedBeliefs = beliefs;
        return beliefs;
    }

    public boolean hasExpandedBelief() {
        return expandedBeliefs.size() > 0;
    }

    public Set<Integer> getPossibleStates() {
        Set<Integer> possibleStates = new HashSet<>();
        for (int iState = 0; iState < grid.getStateCount(); iState++) {
            for (BeliefState belief : expandedBeliefs) {
                if (belief.valueAt(iState) > 0) {
                    possibleStates.add(iState);
                    break;
                }
            }
        }
        return possibleStates;
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

        // TODO: what to do if iAction leads to forbidden state?

        done = grid.endADR(currentNextState, 0);

        bsNext = currentBelief.nextBeliefState(iAction, currentObservation);

        if( currentState != currentNextState )
            cSameStates = 0;
        else
            cSameStates++;

        done = done || (bsNext == null || ( bsNext.valueAt( currentNextState ) == 0 || ( cSameStates > 10 ) ));

        System.out.println("Agent " + id + ": " + grid.getActionName(iAction) + " -> " + grid.parseState(currentNextState));

        currentState = currentNextState;
        currentBelief.release();
        currentBelief = bsNext;

        System.out.println("Agent " + id + " current belief: " + currentBelief.toString());

        return done;
    }

    public BeaconDistanceGrid getGrid() {
        return grid;
    }

    public boolean isClose(GridAgent other, int distanceThreshold) {
        Pair<Integer, Integer> thisLoc = grid.stateToLocation(this.currentState);
        Pair<Integer, Integer> otherLoc = other.grid.stateToLocation(other.currentState);

        return Math.abs(thisLoc.first() - otherLoc.first()) + Math.abs(thisLoc.second() - otherLoc.second()) <= distanceThreshold;
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
        System.out.println("Agent " + id + " solves:");
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
            double dDiscountedReward = grid.computeAverageDiscountedReward( 500, 150, viAlgorithm, true , ExecutionProperties.useHighLevelMultiThread() || ExecutionProperties.useMultiThread() );
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
        grid.clearStatistics();
        return viAlgorithm;
    }

    public void initRun(String sModelName) {
        grid = new BeaconDistanceGrid(true);
        try {
            load(ExecutionProperties.getPath() + sModelName + ".POMDP");
            initAgentWise(startState, endState);
        }
        catch (InvalidModelFileFormatException | IOException e) {
            e.printStackTrace();
        }
        policy = null;
        currentBelief = getInitialBeliefState();
        currentState = chooseStartState();
        currentObservation = 0;
        currentNextState = 0;
        cSameStates = 0;
        expandedBeliefs = new ArrayList<>();
        grid.setForbiddenStates(forbiddenStates);
        Runtime.getRuntime().gc();
        System.out.println("Agent " + id + " initialized");
    }
}
