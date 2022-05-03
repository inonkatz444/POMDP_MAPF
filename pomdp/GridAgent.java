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
    private static char autoInc = 'a';
    private BeaconDistanceGrid grid;
    private PolicyStrategy policy;
    private Map<Integer, Boolean> forbiddenStates;
    private BeliefState currentBelief;
    private int currentState, currentNextState, currentObservation;
    private int cSameStates;
    private int distanceThreshold;
    private final char id;
    private Set<BeliefState> expandedBeliefs;
    private int START_STATE, END_STATE;
    private boolean retrain, isExpandedBeliefsRelevant;

    private final double SUCCESS_REWARD = 10.0;
    private final double INTER_REWARD = -0.04;

    public GridAgent(int distanceThreshold) {
        forbiddenStates = new HashMap<>();
        this.distanceThreshold = distanceThreshold;
        id = autoInc++;
        expandedBeliefs = new HashSet<>();
        currentBelief = null;
        retrain = true;
        isExpandedBeliefsRelevant = false;
    }

    public char getID() {
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
    }

    public boolean needsRetrain() {
        return retrain;
    }

    public boolean isExpandedBeliefRelevant() {
        return isExpandedBeliefsRelevant;
    }

    public void irrelevantExpandedBeliefs() {
        isExpandedBeliefsRelevant = false;
    }

    public void load(String fileName) throws InvalidModelFileFormatException, IOException {
        grid.load(fileName);
        START_STATE = grid.getStartState(id);
        END_STATE = grid.getEndState(id);
    }

    // TODO: Change the observation generation in the belief expansion
    public Set<BeliefState> expandBelief(BeliefState bs) {
        int iObservation = 0, iAction;
        BeliefState bsSuccessor = null;
        double dProb = 0.0;
        Set<BeliefState> unfolded = new HashSet<>();

        iAction = policy.getAction(bs);
        if (iAction == -1) {
            return unfolded;
        }

        for( iObservation = 0 ; iObservation < grid.getObservationCount() ; iObservation++ ){
            dProb = bs.probabilityOGivenA( iAction, iObservation );
            if( dProb > 0 ){
                bsSuccessor = bs.nextBeliefState( iAction, iObservation );
                unfolded.add(bsSuccessor);
            }
        }
        return unfolded;
    }

    public void expandBeliefs() {
        if (!isExpandedBeliefRelevant()) {
            expandedBeliefs.clear();
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
            expandedBeliefs = beliefs;
            isExpandedBeliefsRelevant = true;
        }
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

        if (currentState < 0 || currentState >= grid.getStateCount()) {
            throw new InvalidModelFileFormatException("Start: must be valid state - between 0 and " + grid.getStateCount());
        }

        if (END_STATE < 0 || END_STATE >= grid.getStateCount()) {
            throw new InvalidModelFileFormatException("Terminal: must be valid state - between 0 and " + grid.getStateCount());
        }

        for (int iStartState = 0; iStartState < grid.getStateCount(); iStartState++) {
            grid.setStartStateProb(iStartState, currentBelief.valueAt(iStartState));

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

        irrelevantExpandedBeliefs();

        return done;
    }

    public BeaconDistanceGrid getGrid() {
        return grid;
    }

    public boolean hasConverged() {
        return policy.hasConverged();
    }

    public boolean isClose(GridAgent other) {
        BeliefState otherBelief = other.getCurrentBelief();
        Point thisLoc, otherLoc;

        for (Map.Entry<Integer, Double> doubleEntry : currentBelief.getNonZeroEntries()) {
            thisLoc = grid.stateToLocation(doubleEntry.getKey());
            for (Map.Entry<Integer, Double> integerDoubleEntry : otherBelief.getNonZeroEntries()) {
                otherLoc = other.getGrid().stateToLocation(integerDoubleEntry.getKey());
                if (thisLoc.distance(otherLoc) < 2 * distanceThreshold && !other.isDone()) {
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
        double dProb = grid.getRandomGenerator().nextDouble();
        int iState = 0;
        Map.Entry<Integer, Double> e;
        Iterator<Map.Entry<Integer, Double>> beliefStateIterator = currentBelief.getNonZeroEntries().iterator();
        while (dProb > 0) {
            e = beliefStateIterator.next();
            iState = e.getKey();
            dProb -= e.getValue();
        }

        return iState;
    }

    public BeliefState getCurrentBelief() {
        return currentBelief;
    }

    public int getStateCount() {
        return grid.getStateCount();
    }

    public void solve(String methodName, double dTargetADR, int cMaxIterations, int maxSteps) {
        System.out.println("Agent " + id + " solves:");
//        if( methodName.equals( "QMDP" ) ){
//            MDPValueFunction vfQMDP = grid.getMDPValueFunction();
//            vfQMDP.persistQValues( true );
//            vfQMDP.valueIteration( 100, 0.001 );
//            System.out.println("Forbidden states: " + getForbiddenStates().stream().map(s -> grid.parseState(s)).toList().toString());
//            double dDiscountedReward = grid.computeAverageDiscountedReward( 1000, 100, vfQMDP );
//            Logger.getInstance().log( "POMDPSolver", 0, "main", "ADR = " + dDiscountedReward );
//            this.policy = vfQMDP;
//        }

        ValueIteration viAlgorithm = AlgorithmsFactory.getAlgorithm( methodName, grid );
        try{
            assert viAlgorithm != null;
            viAlgorithm.valueIteration( cMaxIterations, ExecutionProperties.getEpsilon(), dTargetADR );
            double dDiscountedReward = grid.computeAverageDiscountedReward( 200, maxSteps, viAlgorithm, true , ExecutionProperties.useHighLevelMultiThread() || ExecutionProperties.useMultiThread() );
            Logger.getInstance().log( "GridAgent", 0, "main", "ADR = " + dDiscountedReward );
            this.policy = viAlgorithm;
            retrain = false;
        }

        catch( Exception e ){
            System.out.println( e );
            e.printStackTrace();
        }
        catch( Error err ){
            Runtime rtRuntime = Runtime.getRuntime();
            System.out.println( "GridAgent: " + err +
                    " allocated " + ( rtRuntime.totalMemory() - rtRuntime.freeMemory() ) / 1000000 +
                    " free " + rtRuntime.freeMemory() / 1000000 +
                    " max " + rtRuntime.maxMemory() / 1000000 );
            System.out.print( "Stack trace: " );
            err.printStackTrace();
        }
    }

    public int distanceTo(int state) {
        return grid.distance(currentBelief, state);
    }

    public int distanceToGoal() {
        return distanceTo(END_STATE);
    }

    public List<Integer> getForbiddenStates() {
        List<Integer> forbidden = new ArrayList<>();
        forbiddenStates.forEach((s, isForbidden) -> {
            if(isForbidden) forbidden.add(s);
        });
        return forbidden;
    }

    public void printGrid() {
        int rows = grid.getRows();
        int cols = grid.getCols();

        System.out.println("Legend:");
        System.out.println("    #: Wall");
        System.out.println("    <number>: beacon with influence range <number>");
        System.out.println("    <lower-case letter>: agent <lower-case letter> current state");
        System.out.println("    <upper-case letter>: agent <lower-case letter> goal");
        System.out.println("    *: forbidden state\n");

        System.out.print("    #");
        for (int j = 0; j < cols+1; j++) {
            System.out.print("#");
        }
        System.out.println();

        for (int i = 0; i < rows; i++) {
            System.out.print("    #");
            for (int j = 0; j < cols; j++) {
                boolean clear = true;
                if (grid.locationToState(i, j) == grid.HOLE) {
                    System.out.print("#");
                    clear = false;
                }
                else if (grid.locationToState(i, j) == grid.getEndState(id)) {
                    System.out.print(Character.toUpperCase(id));
                    clear = false;
                }
                else if (grid.locationToState(i, j) == currentState) {
                    System.out.print(Character.toLowerCase(id));
                    clear = false;
                }
                else {
                    for (Beacon b : grid.getBeacons()){
                        if (b.getLoc().equals(new Point(i, j))) {
                            System.out.print(b.getRange());
                            clear = false;
                            break;
                        }
                    }
                }
                if (clear) {
                    if (isForbidden(grid.locationToState(i, j))) {
                        System.out.print("*");
                    }
                    else {
                        System.out.print(" ");
                    }
                }
            }
            System.out.println("#");
        }

        System.out.print("    #");
        for (int j = 0; j < cols+1; j++) {
            System.out.print("#");
        }
        System.out.println("\n");
    }

    public void initRun(String sModelName) {
        grid = new BeaconDistanceGrid(2);
        try {
            load(ExecutionProperties.getPath() + sModelName + ".POMDP");
            if (currentBelief == null) {
                currentState = START_STATE;
                double[] newBelief = new double[grid.getStateCount()];
                newBelief[currentState] = 1.0;
                currentBelief = grid.getBeliefStateFactory().newBeliefState(newBelief);
            }
            initAgentWise();
            printGrid();
        }
        catch (InvalidModelFileFormatException | IOException e) {
            e.printStackTrace();
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
