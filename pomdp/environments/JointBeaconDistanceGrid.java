package pomdp.environments;

import pomdp.GridAgent;
import pomdp.utilities.*;
import pomdp.utilities.datastructures.CartesianIterator;

import java.util.*;

public class JointBeaconDistanceGrid extends BeaconDistanceGrid{
    protected int numOfSingleStates;
    protected int numOfSingleActions;
    protected int numOfSingleObservations;
    public int SINGLE_DONE;
    public Map<Character, Integer> endStates;
    private List<GridAgent> agents;

    private List<Integer>[] jointStateToState;
    private List<Integer>[] jointActionToAction;
    private List<Integer>[] jointObservationToObservation;

    private CartesianIterator[][] cachedTransitions;
    private boolean isOffline;

    public JointBeaconDistanceGrid(List<GridAgent> agents, boolean isOffline) {
        super(agents.size());
        endStates = new HashMap<>();
        this.agents = agents;
        this.isOffline = isOffline;
    }

    public void initCaching() {
        jointStateToState = new List[m_cStates];
        jointActionToAction = new List[m_cActions];
        jointObservationToObservation = new List[m_cObservations];
        cachedTransitions = new CartesianIterator[m_cStates][m_cActions];
    }

    public void addEndState(char id, int endState) {
        endStates.put(id, endState);
    }

    public int getEndState(char id) {
        return endStates.get(id);
    }

    public List<GridAgent> getAgents() {
        return agents;
    }

    @Override
    public MDPValueFunction getMDPValueFunction() {
        return super.getMDPValueFunction();
    }

    @Override
    public int observe(int iAction, int iState) {
        List<Integer> stateValues = decodeState(iState);
        List<Integer> actionValues = decodeAction(iAction);
        List<Integer> observationValues = new ArrayList<>();
        for (int i = 0; i < numOfAgents; i++) {
            observationValues.add(agents.get(i).getGrid().observe(actionValues.get(i), agents.get(i).getGrid().fromJointGrid(stateValues.get(i), this)));
        }
        int encodedObservation = encodeObservations(observationValues);
//        System.out.print(parseObservation(encodedObservation) + " ");
        return encodedObservation;
    }

    @Override
    public double O(int iAction, int iState, int iObservation) {
        List<Integer> stateValues = decodeState(iState);
        List<Integer> actionValues = decodeAction(iAction);
        List<Integer> observationValues = decodeObservation(iObservation);
        double prob = 1;
        for (int i = 0; i < numOfAgents; i++) {
            prob *= agents.get(i).getGrid().O(actionValues.get(i), agents.get(i).getGrid().fromJointGrid(stateValues.get(i), this), observationValues.get(i));
        }
        return prob;
    }

    @Override
    public String parseState(int iState) {
        List<Integer> stateValues = decodeState(iState);
        StringBuilder sState = new StringBuilder("(");
        for (int i = 0; i < numOfAgents; i++) {
            if (stateValues.get(i) == SINGLE_DONE) {
                sState.append("DONE");
            }
            else {
                sState.append(stateToLocation(stateValues.get(i)).add(origin).toString());
            }
            sState.append(", ");
        }
        sState.delete(sState.length()-2, sState.length());
        sState.append(")");
        if (isTerminalState(iState)) {
            sState.append("*");
        }
        return sState.toString();
    }

    @Override
    public double tr(int iState1, int iAction, int iState2) {
        if (isForbidden(iState2)) {
            return 0;
        }

        if (endStates.containsValue(iState1)) {
            return endStates.containsValue(iState2) ? 1 : 0;
        }

        List<Integer> startStates = decodeState(iState1);
        List<Integer> actions = decodeAction(iAction);
        List<Integer> endStates = decodeState(iState2);

        double prob = 1;
        for (int iAgent = 0; iAgent < numOfAgents; iAgent++) {
            if (startStates.get(iAgent) == SINGLE_DONE) {
                prob *= endStates.get(iAgent) == SINGLE_DONE ? 1.0 : 0.0;
            }
            else if (!agents.get(iAgent).canDone(this) && isInBorder(startStates.get(iAgent))) {
                prob *= Objects.equals(startStates.get(iAgent), endStates.get(iAgent)) ? 1.0 : 0.0;
            }
            else {
                prob *= agents.get(iAgent).getGrid().tr(agents.get(iAgent).getGrid().fromJointGrid(startStates.get(iAgent), this), actions.get(iAgent), agents.get(iAgent).getGrid().fromJointGrid(endStates.get(iAgent), this));
            }
        }

        return prob;
    }

    @Override
    public Iterator<Map.Entry<Integer, Double>> getNonZeroTransitions(int iStartState, int iAction) {
        if (cachedTransitions[iStartState][iAction] == null) {
            Grid bigGrid;
            int bigGridState;
            List<Integer> startStates = decodeState(iStartState);
            List<Integer> actions = decodeAction(iAction);
            List<Iterable<Map.Entry<Integer, Double>>> res = new ArrayList<>();

            for (int iAgent = 0; iAgent < numOfAgents; iAgent++) {
                bigGrid = agents.get(iAgent).getGrid();
                bigGridState = bigGrid.fromJointGrid(startStates.get(iAgent), this);
//            if (startStates.get(iAgent) == SINGLE_DONE) {
//                elements.add(new Pair<>(SINGLE_DONE, 1.0));
//                res.add(elements);
//                continue;
//            }
                res.add(getBigGridTransitions(bigGrid, bigGridState, actions.get(iAgent), startStates.get(iAgent)));
            }
            cachedTransitions[iStartState][iAction] = new CartesianIterator(res, numOfSingleStates);
        }
        else {
            cachedTransitions[iStartState][iAction].initialize();
        }
        return cachedTransitions[iStartState][iAction];
    }

    private Iterable<Map.Entry<Integer, Double>> getBigGridTransitions(Grid bigGrid, int bigGridState, int action, int startState) {
        List<Map.Entry<Integer, Double>> elements = new ArrayList<>();
        Iterator<Map.Entry<Integer, Double>> it = bigGrid.getNonZeroTransitions(bigGridState, action);
        while (it.hasNext()) {
            Map.Entry<Integer, Double> e = it.next();
            int endState = e.getKey();
            if (endState == bigGrid.DONE) {
                elements.add(new Pair<>(SINGLE_DONE, 1.0));
            }
            else if (isInBorder(startState) && !bigGrid.stateToLocation(endState).inBound(this)) {
                elements.add(new Pair<>(startState, e.getValue()));
            }
            else {
                elements.add(new Pair<>(bigGrid.toJointGrid(endState, this), e.getValue()));
            }
        }
        int numOfSINGLE_DONE = 0;
        for (Map.Entry<Integer, Double> element : elements) {
            if (element.getKey() == SINGLE_DONE) {
                numOfSINGLE_DONE++;
            }
        }
        if (numOfSINGLE_DONE >= 2) {
            double doneProb = 0;
            List<Map.Entry<Integer, Double>> newElements = new ArrayList<>();
            for (Map.Entry<Integer, Double> element : elements){
                if (element.getKey() != SINGLE_DONE) {
                    newElements.add(element);
                }
                else {
                    doneProb += element.getValue();
                }
            }
            newElements.add(new Pair<>(SINGLE_DONE, doneProb));
            return newElements;
        }
        else {
            return elements;
        }
    }

    @Override
    public double R(int iStartState, int iAction) {

        if (isOffline) {
            List<Integer> iStartStateValues = decodeState(iStartState);
            List<Integer> iActionValues = decodeAction(iAction);

            double reward = 0;
            for (int iAgent = 0; iAgent < numOfAgents; iAgent++) {
                reward += agents.get(iAgent).getGrid().R(iStartStateValues.get(iAgent), iActionValues.get(iAgent));
            }
            return reward;
        }

        double dReward = m_adStoredRewards[iStartState][iAction];
        if( dReward == MIN_INF ) {
            double dSumRewards = 0;
            for (Iterator<Map.Entry<Integer, Double>> it = getNonZeroTransitions(iStartState, iAction); it.hasNext(); ) {
                Map.Entry<Integer, Double> e = it.next();
                dSumRewards += R(iStartState, iAction, e.getKey()) * e.getValue();
            }
            m_adStoredRewards[iStartState][iAction] = dSumRewards;
            dReward = dSumRewards;
        }
        return dReward;
    }

    @Override
    public double R(int iStartState, int iAction, int iEndState) {
        if (isOffline) {
            return R(iStartState, iAction);
        }

//        if (isTerminalState(iEndState)) {
//            return 0;
//        }

        List<Integer> iStartStateValues = decodeState(iStartState);
        List<Integer> iActionValues = decodeAction(iAction);
        List<Integer> iEndStateValues = decodeState(iEndState);

        double reward = 0;
        int numberOfTerminal = 0;
        for (int iAgent = 0; iAgent < numOfAgents; iAgent++) {
            if ((!agents.get(iAgent).canDone(this) && isInBorder(iEndStateValues.get(iAgent))) || (agents.get(iAgent).getEndState() == agents.get(iAgent).getGrid().fromJointGrid(iStartStateValues.get(iAgent), this) && iActionValues.get(iAgent) == agents.get(iAgent).getGrid().getActionIndex("DONE_ACT"))) {
                numberOfTerminal++;
            }
        }

        for (int iAgent = 0; iAgent < numOfAgents; iAgent++) {
            if (numberOfTerminal >= numOfAgents - 1) {
                reward += agents.get(iAgent).getMDPValueFunction().getValue(agents.get(iAgent).getGrid().fromJointGrid(iEndStateValues.get(iAgent), this));
            }
            else {
                reward += agents.get(iAgent).getGrid().R(agents.get(iAgent).getGrid().fromJointGrid(iStartStateValues.get(iAgent), this), iActionValues.get(iAgent), agents.get(iAgent).getGrid().fromJointGrid(iEndStateValues.get(iAgent), this));
            }
        }

        return reward;
    }

    @Override
    public boolean isInBorder(int iStateValue) {
        if (iStateValue == SINGLE_DONE) {
            return true;
        }

        Point loc = stateToLocation(iStateValue).add(origin);
        return loc.first().equals(origin.first()) || loc.second().equals(origin.second()) || loc.first() == origin.first() + rows-1 || loc.second() == origin.second() + cols-1;
    }

    @Override
    public void initGrid() throws InvalidModelFileFormatException {
        grid = new int[rows][cols];
        Point loc;
        int istate = 0;
        for (int i = 0; i < this.rows; i++) {
            for (int j = 0; j < this.cols; j++) {
                loc = Point.getPoint(i, j);
                if (!holes.contains(loc)) {
                    stateToLocation.add(loc);
                    grid[i][j] = istate++;
                }
                else {
                    grid[i][j] = HOLE;
                }
            }
        }
        if (stateToLocation.size() + 1 != numOfSingleStates) {
            throw new RuntimeException("Grid data does not match state number: " + stateToLocation.size() + 1 + " vs " + numOfSingleStates);
        }

        System.out.println("beacons: " + Arrays.toString(beacons.toArray()));

        System.out.println("Grid Load Complete!");
    }

    @Override
    public boolean isForbidden(int iState) {
        return isCollisionState(iState);
    }

    public boolean isCollisionState(int state) {
        List<Integer> stateValues = decodeState(state);
        for (int i = 0; i < numOfAgents; i++) {
            for (int j = i+1; j < numOfAgents; j++) {
                if (stateValues.get(i).equals(stateValues.get(j)) && stateValues.get(i) != SINGLE_DONE) {
                    return true;
                }
            }
        }
        return false;
    }

    // relative position
    @Override
    public Point stateToLocation(int iState) {
        return super.stateToLocation(iState);
    }

    @Override
    public int locationToState(Point loc) {
        loc = loc.subtract(origin);
        if (loc.first() < 0 || loc.second() < 0 || loc.first() >= rows || loc.second() >= cols) {
            return SINGLE_DONE;
        }
        return super.locationToState(loc);
    }

    public void setNumOfSingleStates(int numOfSingleStates) {
        this.numOfSingleStates = numOfSingleStates;
        SINGLE_DONE = numOfSingleStates - 1;
    }

    public int getNumOfSingleStates() {
        return numOfSingleStates;
    }

    public void setNumOfSingleActions(int numOfSingleActions) {
        this.numOfSingleActions = numOfSingleActions;
    }

    public int getNumOfSingleActions() {
        return numOfSingleActions;
    }

    public void setNumOfSingleObservations(int numOfSingleObservations) {
        this.numOfSingleObservations = numOfSingleObservations;
    }

    public int getNumOfSingleObservations() {
        return numOfSingleObservations;
    }

    private List<Integer> decode(int encoded, int numOfSingle) {
        List<Integer> decoded = new ArrayList<>();
        for (int i = 0; i < numOfAgents; i++) {
            decoded.add(encoded % numOfSingle);
            encoded /= numOfSingle;
        }
        return decoded;
    }

    public List<Integer> decodeState(int state) {
        if (jointStateToState[state] == null) {
            jointStateToState[state] = decode(state, numOfSingleStates);
        }
        return jointStateToState[state];
    }

    public List<Integer> decodeAction(int action) {
        if (jointActionToAction[action] == null) {
            jointActionToAction[action] = decode(action, numOfSingleActions);
        }
        return jointActionToAction[action];
    }

    public List<Integer> decodeObservation(int observation) {
        if (jointObservationToObservation[observation] == null) {
            jointObservationToObservation[observation] = decode(observation, numOfSingleObservations);
        }
        return jointObservationToObservation[observation];
    }

    private int encode(List<Integer> decoded, int numOfSingle) {
        int encoded = 0;
        for (int i = 0, power = 1; i < numOfAgents; i++, power *= numOfSingle) {
            encoded += power * decoded.get(i);
        }
        return encoded;
    }

    @Override
    public List<Integer> getMovableActions() {
        List<Integer> movableActions = new ArrayList<>();
        int maxActionValue = getNumOfSingleActions()-1;
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

            // add movable actions here
            boolean found = false;
            // check if one of the agent's action is movable
            for (int iAgent = 0; iAgent < numOfAgents; iAgent++) {
                if (agents.get(iAgent).getGrid().getMovableActions().contains(actionValues.get(iAgent))) {
                    found = true;
                    break;
                }
            }
            if (found) {
                movableActions.add(encodeActions(actionValues));
            }
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
        return movableActions;
    }

    @Override
    public List<Integer> getSensingActions() {
        List<Integer> sensingActions = new ArrayList<>();
        int maxActionValue = getNumOfSingleActions()-1;
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

            // add movable actions here
            boolean allPing = true;
            // check if one of the agent's action is movable
            for (int iAgent = 0; iAgent < numOfAgents; iAgent++) {
                if (!agents.get(iAgent).getGrid().getSensingActions().contains(actionValues.get(iAgent)) && actionValues.get(iAgent) != SINGLE_DONE) {
                    allPing = false;
                    break;
                }
            }
            if (allPing) {
                sensingActions.add(encodeActions(actionValues));
            }
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
        return sensingActions;
    }

    @Override
    public int getDoneAction() {
        return encodeActions(agents.stream().map(a -> a.getGrid().getDoneAction()).toList());
    }

    public int encodeStates(List<Integer> stateValues) {
        return encode(stateValues, numOfSingleStates);
    }

    public int encodeActions(List<Integer> actionValues) {
        return encode(actionValues, numOfSingleActions);
    }

    public int encodeObservations(List<Integer> observationValues) {
        return encode(observationValues, numOfSingleObservations);
    }

    public String parseObservation(int iObservation) {
        StringBuilder output = new StringBuilder("(");
        List<Integer> decodedObservation = decodeObservation(iObservation);
        for (int i = 0; i < numOfAgents; i++) {
            output.append(super.parseObservation(decodedObservation.get(i))).append(", ");
        }
        if (output.length() > 2) {
            output.delete(output.length()-2, output.length()).append(")");
        }
        return output.toString();
    }
}
