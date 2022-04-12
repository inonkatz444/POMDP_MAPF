package pomdp;

import pomdp.algorithms.AlgorithmsFactory;
import pomdp.algorithms.PolicyStrategy;
import pomdp.algorithms.ValueIteration;
import pomdp.environments.BeaconDistanceGrid;
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
    private int startState;
    private final int START_STATE, END_STATE;
    private boolean retrain;

    private BeliefState[] stepBeliefs;
    private int[] stepStates;

    private final double SUCCESS_REWARD = 10.0;
    private final double INTER_REWARD = -0.04;

    public GridAgent(int distanceThreshold, int startState, int endState) {
        forbiddenStates = new HashMap<>();
        this.distanceThreshold = distanceThreshold;
        id = autoInc++;
        expandedBeliefs = new ArrayList<>();
        this.startState = startState;
        START_STATE = startState;
        END_STATE = endState;
        currentState = startState;
        currentBelief = null;
        retrain = true;
    }

    public int getID() {
        return id;
    }

    public String getStartStateString() {
        return grid.parseState(currentState);
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

    public void retrain() {
        retrain = true;
        startState = currentState;
    }

    public boolean needsRetrain() {
        return retrain;
    }

    public void load(String fileName) throws InvalidModelFileFormatException, IOException {
        grid.load(fileName);
    }

    // TODO: Change the observation generation in the belief expansion
    public List<BeliefState> expandBelief(BeliefState bs) {
        int iObservation = 0, iAction;
        BeliefState bsSuccessor = null;
        double dProb = 0.0;
        List<BeliefState> unfolded = new ArrayList<>();

        for( iObservation = 0 ; iObservation < grid.getObservationCount() ; iObservation++ ){
            iAction = policy.getAction(bs);
            dProb = bs.probabilityOGivenA( iAction, iObservation );
            if( dProb > 0 ){
                bsSuccessor = bs.nextBeliefState( iAction, iObservation );
                unfolded.add(bsSuccessor);
            }
        }
        return unfolded;
    }

    public List<BeliefState> expandBeliefs() {
        Set<BeliefState> beliefs = new HashSet<>();
        Queue<BeliefState> temp = new ArrayDeque<>();
        beliefs.add(currentBelief);
        for (int iExpandStep = 1; iExpandStep <= distanceThreshold; iExpandStep++) {
            temp.addAll(beliefs);
            beliefs.clear();
            while (!temp.isEmpty()) {
                BeliefState belief = temp.remove();
                beliefs.addAll(expandBelief(belief));
            }
        }
        expandedBeliefs = beliefs.stream().toList();
        return expandedBeliefs;
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

    public void initAgentWise() throws InvalidModelFileFormatException{
        grid.setRewardType( POMDP.RewardType.StateActionState );
        grid.addTerminalState(END_STATE);

        if (startState < 0 || startState >= grid.getStateCount()) {
            throw new InvalidModelFileFormatException("Start: must be valid state - between 0 and " + grid.getStateCount());
        }

        if (END_STATE < 0 || END_STATE >= grid.getStateCount()) {
            throw new InvalidModelFileFormatException("Terminal: must be valid state - between 0 and " + grid.getStateCount());
        }

        for (int iStartState = 0; iStartState < grid.getStateCount(); iStartState++) {
            grid.setStartStateProb(iStartState, iStartState == startState ? 1.0 : 0.0);

            for (int iAction = 0; iAction < grid.getActionCount(); iAction++) {

                for (int iEndState = 0; iEndState < grid.getStateCount(); iEndState++) {
                    grid.setReward( iStartState, iAction, iEndState, iEndState == END_STATE ? SUCCESS_REWARD : INTER_REWARD );
                    grid.setMinimalReward( iAction, iEndState == END_STATE ? SUCCESS_REWARD : INTER_REWARD );
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

    public boolean isClose(GridAgent other) {
        BeliefState otherBelief = other.getCurrentBelief();
        Point thisLoc, otherLoc;

        for (Map.Entry<Integer, Double> doubleEntry : currentBelief.getNonZeroEntries()) {
            thisLoc = grid.stateToLocation(doubleEntry.getKey());
            for (Map.Entry<Integer, Double> integerDoubleEntry : otherBelief.getNonZeroEntries()) {
                otherLoc = other.getGrid().stateToLocation(integerDoubleEntry.getKey());
                if (thisLoc.distance(otherLoc) <= distanceThreshold + 1 && !other.isDone()) {
                    return true;
                }
            }
        }

        return false;
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

    public void solve(String methodName, double dTargetADR, int cMaxIterations, int maxSteps) {
        System.out.println("Agent " + id + " solves:");
        if( methodName.equals( "QMDP" ) ){
            MDPValueFunction vfQMDP = grid.getMDPValueFunction();
            vfQMDP.persistQValues( true );
            vfQMDP.valueIteration( 100, 0.001 );
            System.out.println("Forbidden states: " + getForbiddenStates().stream().map(s -> grid.parseState(s)).toList().toString());
            double dDiscountedReward = grid.computeAverageDiscountedReward( 1000, 100, vfQMDP );
            Logger.getInstance().log( "POMDPSolver", 0, "main", "ADR = " + dDiscountedReward );
            this.policy = vfQMDP;
        }

        ValueIteration viAlgorithm = AlgorithmsFactory.getAlgorithm( methodName, grid );
        try{
            assert viAlgorithm != null;
            viAlgorithm.valueIteration( cMaxIterations, ExecutionProperties.getEpsilon(), dTargetADR );
            double dDiscountedReward = grid.computeAverageDiscountedReward( 200, 150, viAlgorithm, true , ExecutionProperties.useHighLevelMultiThread() || ExecutionProperties.useMultiThread() );
            Logger.getInstance().log( "POMDPSolver", 0, "main", "ADR = " + dDiscountedReward );
            this.policy = viAlgorithm;
            retrain = false;
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
    }

    public int distanceToGoal() {
        Iterator<Map.Entry<Integer,Double>> thisStateid = currentBelief.getNonZeroEntries().iterator();
        int min_distance = grid.getRows() + grid.getCols();
        Point thisLoc, goalLoc = grid.stateToLocation(END_STATE);

        while (thisStateid.hasNext()) {
            thisLoc = grid.stateToLocation(thisStateid.next().getKey());
            min_distance = Math.min(min_distance, thisLoc.distance(goalLoc));
        }

        return min_distance;
    }

    public List<Integer> getForbiddenStates() {
        List<Integer> forbidden = new ArrayList<>();
        forbiddenStates.forEach((s, isForbidden) -> {
            if(isForbidden) forbidden.add(s);
        });
        return forbidden;
    }

    public void initRun(String sModelName) {
        grid = new BeaconDistanceGrid(true);
        try {
            load(ExecutionProperties.getPath() + sModelName + ".POMDP");
            initAgentWise();
        }
        catch (InvalidModelFileFormatException | IOException e) {
            e.printStackTrace();
        }
        if (currentBelief == null) {
            currentBelief = getInitialBeliefState();
        }
        policy = null;
        cSameStates = 0;
        currentObservation = 0;
        currentNextState = 0;
        grid.setForbiddenStates(forbiddenStates);
        Runtime.getRuntime().gc();
        System.out.println("Agent " + id + " initialized");
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        GridAgent agent = (GridAgent) o;
        return id == agent.id;
    }

    @Override
    public int hashCode() {
        return Objects.hash(id);
    }
}
