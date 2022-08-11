package pomdp;

import pomdp.algorithms.AlgorithmsFactory;
import pomdp.algorithms.PolicyStrategy;
import pomdp.algorithms.ValueIteration;
import pomdp.environments.BeaconDistanceGrid;
import pomdp.environments.JointBeaconDistanceGrid;
import pomdp.environments.POMDP;
import pomdp.utilities.*;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.*;

public class GridAgent implements Comparable<GridAgent>{
    private static char autoInc = 'a';
    private BeaconDistanceGrid grid;
    private PolicyStrategy mainPolicy, escapePolicy, tempPolicy;
    private Map<Integer, Boolean> forbiddenStates, tempForbiddenStates;
    private BeliefState currentBelief;
    private int currentState;
    private int cSameStates;
    private final int distanceThreshold;
    private final char id;
    private Set<BeliefState> expandedBeliefs;
    private int START_STATE, END_STATE;
    private boolean isExpandedBeliefsRelevant;
    private int forbiddenTimer;
    private boolean isTimed;

    private double sumOfDiscountedRewards;
    private double discountFactor;
    private final double gamma;

    private TrackLogger agentLogger;

    public GridAgent(int distanceThreshold) {
        forbiddenStates = new HashMap<>();
        tempForbiddenStates = new HashMap<>();
        this.distanceThreshold = distanceThreshold;
        id = autoInc++;
        expandedBeliefs = new HashSet<>();
        currentBelief = null;
        isExpandedBeliefsRelevant = false;
        forbiddenTimer = 0;
        isTimed = false;
        sumOfDiscountedRewards = 0;
        discountFactor = 1;
        gamma = 0.99;
    }

    // resets the agent to its initial state
    public void reset() {
        currentState = START_STATE;
        currentBelief = null;
        cSameStates = 0;
        expandedBeliefs.clear();
        forbiddenTimer = 0;
        isTimed = false;
        sumOfDiscountedRewards = 0;
        discountFactor = 1;
        forbiddenStates.clear();
    }

    public double getSumOfDiscountedRewards() {
        return sumOfDiscountedRewards;
    }

    public void setLogger(String filename) {
        try {
            agentLogger = TrackLogger.getAgentInstance(id);
            agentLogger.setOutputStream(filename);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }

    public char getID() {
        return id;
    }

    public int getDistanceThreshold() {
        return distanceThreshold;
    }

    public TrackLogger getAgentLogger() {
        return agentLogger;
    }

    public void log(String sClassName, int iLevel, String sMethodName, boolean printToSystem, String sMessage) {
        agentLogger.log(sClassName, iLevel, sMethodName, printToSystem, sMessage);
        TrackLogger.getInstance().log(sClassName, iLevel, sMethodName, false, sMessage);
    }

    public int getForbiddenTimer() {
        return forbiddenTimer;
    }

    private PolicyStrategy getRelevantPolicy() {
        return isTimerSet() ? escapePolicy : mainPolicy;
    }

    private PolicyStrategy getRelevantFuturePolicy(int iStep) {
        if (!isTimerSet() || getForbiddenTimer() - iStep < 0)
            return mainPolicy;
        return escapePolicy;
    }

    private void decreaseTimer() {
        if (isTimerSet()) {
            setForbiddenTimer(getForbiddenTimer() - 1);
            log("GridAgent", 0, "decreaseTimer", true, "Agent " + id + " timer: " + getForbiddenTimer());
        }
    }

    public void setForbiddenTimer(int forbiddenTimer) {
        this.forbiddenTimer = forbiddenTimer;
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

    public void addForbiddenStates(Set<Integer> iStates) {
        iStates.forEach(iState -> forbiddenStates.put(iState, true));
    }

    public boolean isForbidden(int iState) {
        return forbiddenStates.getOrDefault(iState, false);
    }

    public boolean isExpandedBeliefRelevant() {
        return isExpandedBeliefsRelevant;
    }

    public void irrelevantExpandedBeliefs() {
        isExpandedBeliefsRelevant = false;
    }

    public void load(String fileName) throws InvalidModelFileFormatException, IOException {
        grid.load(fileName, id);
        grid.setNOOP(grid.getActionIndex("noop"));
        START_STATE = grid.getStartState(id);
        END_STATE = grid.getEndState();
    }

    public Set<BeliefState> expandBelief(BeliefState bs, int iAction) {
        int iObservation = 0;
        BeliefState bsSuccessor = null;
        double dProb = 0.0;
        Set<BeliefState> unfolded = new HashSet<>();

        for( iObservation = 0 ; iObservation < grid.getObservationCount() ; iObservation++ ){
            dProb = bs.probabilityOGivenA( iAction, iObservation );
            if( dProb > 0 ){
                bsSuccessor = bs.nextBeliefState( iAction, iObservation );
                unfolded.add(bsSuccessor);
            }
        }
        return unfolded;
    }

    public void clearExpandedBeliefs() {
        expandedBeliefs.clear();
        expandedBeliefs.add(currentBelief);
    }

    // returns if the step changed the agent's belief state
    public boolean expandBeliefsStep(int iStep) {
        boolean hasMoved = false;
        int iAction;
        Queue<BeliefState> temp = new ArrayDeque<>(expandedBeliefs);
        expandedBeliefs.clear();
        while (!temp.isEmpty()) {
            BeliefState belief = temp.remove();
            iAction = getRelevantFuturePolicy(iStep).getAction(belief);
            if (iAction == -1) {
                continue;
            }
            // if the action is movement
            if (iAction <= 3) {
                hasMoved = true;
            }
            expandedBeliefs.addAll(expandBelief(belief, iAction));
        }
        return hasMoved || expandedBeliefs.stream().allMatch(b -> b.valueAt(grid.DONE) > 0);
    }

    public void expandBeliefs(int steps) {
        if (!isExpandedBeliefRelevant()) {
            expandedBeliefs.clear();
            Set<BeliefState> beliefs = new HashSet<>();
            Queue<BeliefState> temp = new ArrayDeque<>();
            beliefs.add(currentBelief);
            for (int iExpandStep = 1; iExpandStep <= steps; iExpandStep++) {
                temp.addAll(beliefs);
                beliefs.clear();
                while (!temp.isEmpty()) {
                    BeliefState belief = temp.remove();
                    beliefs.addAll(expandBelief(belief, iExpandStep));
                }
            }
            expandedBeliefs = beliefs;
            isExpandedBeliefsRelevant = true;
        }
    }

    public MDPValueFunction getMDPValueFunction() {
        return grid.getMDPValueFunction();
    }

    public Set<Integer> getPossibleStates() {
        Set<Integer> possibleStates = new HashSet<>();
        for (int iState = 0; iState < grid.getStateCount(); iState++) {
            for (BeliefState belief : expandedBeliefs) {
                if (belief.valueAt(iState) > 0 && iState != grid.DONE) {
                    possibleStates.add(iState);
                    break;
                }
            }
        }
        return possibleStates;
    }

    public void initAgentWise() throws InvalidModelFileFormatException{
        grid.setRewardType( POMDP.RewardType.StateAction );
        grid.addTerminalState(grid.DONE);

        if (currentState < 0 || currentState >= grid.getStateCount()) {
            throw new InvalidModelFileFormatException("Start: must be valid state - between 0 and " + grid.getStateCount());
        }

        if (END_STATE < 0 || END_STATE >= grid.getStateCount()) {
            throw new InvalidModelFileFormatException("Terminal: must be valid state - between 0 and " + grid.getStateCount());
        }

//        double reward = 0;
        for (int iStartState = 0; iStartState < grid.getStateCount(); iStartState++) {
            grid.setStartStateProb(iStartState, currentBelief.valueAt(iStartState));
//            if (!grid.isTerminalState(iStartState)) {
//                reward = iStartState == grid.getEndState() ? SUCCESS_REWARD : INTER_REWARD;
//            }
//            grid.setReward(iStartState, reward);

//            for (int iAction = 0; iAction < grid.getActionCount(); iAction++) {
//
//                if (iStartState == grid.getEndState()) {
//                    grid.setTransition(iStartState, iAction, grid.DONE, 1);
//                }
//
//                for (int iEndState = 0; iEndState < grid.getStateCount() - 1; iEndState++) {
//                    if (iStartState == grid.getEndState()) {
//                        grid.setTransition(iStartState, iAction, iEndState, 0);
//                    }
//                }
//            }
        }

//        for (int iAction = 0; iAction < grid.getActionCount(); iAction++) {
//            grid.setTransition(grid.DONE, iAction, grid.DONE, 1 );
//        }
//        grid.setReward(grid.DONE, 0);

//        grid.initStoredRewards();
    }

    public BeliefState getInitialBeliefState() {
        return grid.getBeliefStateFactory().getInitialBeliefState();
    }

    public boolean step() {
        int iAction = getRelevantPolicy().getAction(currentBelief);
        if( iAction == -1 )
            throw new Error( "Could not find optimal action for bs " + currentBelief );

        return step(iAction);
    }

    public boolean step(int iAction) {
        return step(iAction, isTimerSet());
    }

    public boolean step(int iAction, boolean reduceTimer) {
        int currentNextState, currentObservation;

        currentNextState = grid.execute( iAction, currentState );
        currentObservation = grid.observe( iAction, currentNextState );

        return step(iAction, currentNextState, currentObservation, reduceTimer);
    }

    public boolean step(int iAction, int currentNextState, int currentObservation) {
        return step(iAction, currentNextState, currentObservation, isTimerSet());
    }

    public boolean step(int iAction, int currentNextState, int currentObservation, boolean reduceTimer) {
        BeliefState bsNext;
        boolean done = grid.endADR(currentNextState, 0);
        bsNext = currentBelief.nextBeliefState(iAction, currentObservation);
        double reward = grid.R(currentState, iAction);
        sumOfDiscountedRewards += reward * discountFactor;
        discountFactor *= gamma;

        if( currentState != currentNextState )
            cSameStates = 0;
        else
            cSameStates++;

        done = done || (bsNext == null || ( bsNext.valueAt( currentNextState ) == 0 || ( cSameStates > 10 ) ));

        log("GridAgent", 0, "step", true, "Agent " + id + ": " + grid.getActionName(iAction) + " -> " + grid.parseState(currentNextState) + " O: " + grid.parseObservation(currentObservation) + " R: " + reward);

        currentState = currentNextState;
        currentBelief.release();
        currentBelief = bsNext;

        if (!done) {
            log("GridAgent", 0, "step", true, "Agent " + id + " current belief: " + currentBelief.toString());
        }
        else {
            log("GridAgent", 0, "step", true, "Agent " + id + " done! discounted reward: " + sumOfDiscountedRewards);
        }

        irrelevantExpandedBeliefs();
        if (reduceTimer) {
            decreaseTimer();
        }

        return done;
    }

    public boolean canDone(JointBeaconDistanceGrid jointGrid) {
        for (int iState = 0; iState < jointGrid.getNumOfSingleStates() - 1; iState++) {
            if (getEndState() == grid.fromJointGrid(iState, jointGrid)) {
                return true;
            }
        }
        return false;
    }

    public BeaconDistanceGrid getGrid() {
        return grid;
    }

    // consider only feasible paths
    public boolean isClose(GridAgent other) {
        if (other.isDone()) {
            return false;
        }

        BeliefState otherBelief = other.getCurrentBelief();

        for (Map.Entry<Integer, Double> integerDoubleEntry : otherBelief.getNonZeroEntries()) {
            if (distanceTo(integerDoubleEntry.getKey()) <= 2 * distanceThreshold) {
                return true;
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

    public int getEndState() {
        return END_STATE;
    }

    public BeliefState getCurrentBelief() {
        return currentBelief;
    }

    public int getStateCount() {
        return grid.getStateCount();
    }

    public double solve(String methodName, double dTargetADR, int cMaxIterations, int maxSteps) {

        TrackLogger.getInstance().log("GridAgent", 0, "solve", true, "Agent " + id + " solves:");
//        if( methodName.equals( "QMDP" ) ){
//            MDPValueFunction vfQMDP = grid.getMDPValueFunction();
//            vfQMDP.persistQValues( true );
//            vfQMDP.valueIteration( 100, 0.001 );
//            System.out.println("Forbidden states: " + getForbiddenStates().stream().map(s -> grid.parseState(s)).toList().toString());
//            double dDiscountedReward = grid.computeAverageDiscountedReward( 1000, 100, vfQMDP );
//            Logger.getInstance().log( "POMDPSolver", 0, "main", "ADR = " + dDiscountedReward );
//            this.policy = vfQMDP;
//        }

        double dDiscountedReward = 0;
        ValueIteration viAlgorithm = AlgorithmsFactory.getAlgorithm( methodName, grid );
        try{
            assert viAlgorithm != null;
            viAlgorithm.valueIteration( cMaxIterations, ExecutionProperties.getEpsilon(), dTargetADR );
            dDiscountedReward = grid.computeAverageDiscountedReward( 200, maxSteps, viAlgorithm, true , ExecutionProperties.useHighLevelMultiThread() || ExecutionProperties.useMultiThread() );
            Logger.getInstance().log( "GridAgent", 0, "main", "ADR = " + dDiscountedReward );

            if (dDiscountedReward >= 0) {
                if (forbiddenStates.isEmpty()) {
                    isTimed = false;
                    this.mainPolicy = viAlgorithm;
                }
                else {
                    isTimed = true;
                    this.escapePolicy = viAlgorithm;
                }
                tempPolicy = null;
            }
            else {
                tempPolicy = viAlgorithm;
            }
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
        return dDiscountedReward;
    }

    public void replaceWithTempPolicy() {
        if (forbiddenStates.isEmpty()) {
            isTimed = false;
            this.mainPolicy = tempPolicy;
        }
        else {
            isTimed = true;
            this.escapePolicy = tempPolicy;
        }
        tempPolicy = null;
    }

    public void saveForbiddenStates() {
        tempForbiddenStates.clear();
        tempForbiddenStates.putAll(forbiddenStates);
    }

    public void restoreForbiddenStates() {
        forbiddenStates.clear();
        forbiddenStates.putAll(tempForbiddenStates);
        grid.setForbiddenStates(forbiddenStates);
    }

    public int localize() {
        int steps = 0;
        for (int senseAction : grid.getSensingActions()) {
            step(senseAction, false);
            steps++;
        }
        return steps;
    }

    public void setNullPolicy() {
        mainPolicy = new NullPolicy(grid);
        isTimed = false;
    }

    public int distanceTo(int state) {
        return grid.actualDistance(currentBelief, state);
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

    public int getCurrentState() {
        return currentState;
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
                else if (grid.locationToState(i, j) == grid.getEndState()) {
                    System.out.print(Character.toUpperCase(id));
                    clear = false;
                }
                else if (grid.locationToState(i, j) == currentState) {
                    System.out.print(Character.toLowerCase(id));
                    clear = false;
                }
                else {
                    for (Beacon b : grid.getBeacons()){
                        if (b.getLoc().equals(Point.getPoint(i, j))) {
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
            grid.setOrigin(Point.getPoint(0, 0));
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
        cSameStates = 0;
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

    public boolean isTimerSet() {
        return isTimed;
    }

    public boolean finishEscaping() {
        return isTimerSet() && getForbiddenTimer() <= 0;
    }

    public double stateProb(int iState) {
        return currentBelief.valueAt(iState);
    }

    public int getEND_STATE() {
        return END_STATE;
    }

    public void addPunishment() {
        sumOfDiscountedRewards += grid.FAILURE_REWARD * discountFactor - grid.INTER_REWARD * discountFactor;
    }

    @Override
    public int compareTo(GridAgent o) {
        return o.distanceToGoal() - this.distanceToGoal();
    }
}
