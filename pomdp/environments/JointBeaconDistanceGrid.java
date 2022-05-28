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

    public JointBeaconDistanceGrid(List<GridAgent> agents) {
        super(agents.size());
        endStates = new HashMap<>();
        this.agents = agents;
    }

    public void addEndState(char id, int endState) {
        endStates.put(id, endState);
    }

    public int getEndState(char id) {
        return endStates.get(id);
    }

    @Override
    public int observe(int iAction, int iState) {
        List<Integer> stateValues = decodeState(iState);
        List<Integer> actionValues = decodeAction(iAction);
        List<Integer> observationValues = new ArrayList<>();
        for (int i = 0; i < numOfAgents; i++) {
            if (stateValues.get(i) == SINGLE_DONE) {
                observationValues.add(0);
            }
            else {
                observationValues.add(super.observe(actionValues.get(i), stateValues.get(i)));
            }
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
            if (stateValues.get(i) != SINGLE_DONE) {
                prob *= super.O(actionValues.get(i), stateValues.get(i), observationValues.get(i));
            }
        }
        return prob;
    }

    @Override
    public String parseState(int iState) {
        if (iState == DONE) {
            return "DONE";
        }
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

        if (iState2 == DONE) {
            if (endStates.containsValue(iState1) || iState1 == DONE) {
                return 1;
            }
            else {
                return 0;
            }
        }
        if (iState1 == DONE) {
            return 0;
        }

        if (endStates.containsValue(iState1)) {
            return 0;
        }

        List<Integer> startStates = decodeState(iState1);
        List<Integer> actions = decodeAction(iAction);
        List<Integer> endStates = decodeState(iState2);

        double prob = 1;
        for (int iAgent = 0; iAgent < numOfAgents; iAgent++) {
            if (isInBorder(startStates.get(iAgent))) {
                prob *= endStates.get(iAgent) == SINGLE_DONE ? 1.0 : 0.0;
            }
            else {
                prob *= agents.get(iAgent).getGrid().tr(fromJointGrid(startStates.get(iAgent), this), actions.get(iAgent), fromJointGrid(endStates.get(iAgent), this));
            }
        }

        return prob;
    }

    @Override
    public Iterator<Map.Entry<Integer, Double>> getNonZeroTransitions(int iStartState, int iAction) {
        Grid bigGrid;
        int bigGridState;
        List<Integer> startStates = decodeState(iStartState);
        List<Integer> actions = decodeAction(iAction);
        List<Iterable<Map.Entry<Integer, Double>>> res = new ArrayList<>();

        int numOfTerminalAgents = 0;
        for (int iAgent = 0; iAgent < numOfAgents; iAgent++) {
            if (startStates.get(iAgent) == SINGLE_DONE) {
                numOfTerminalAgents++;
            }
        }
        if (iStartState == DONE || numOfTerminalAgents >= numOfAgents - 1) {
            List<Map.Entry<Integer, Double>> entries = new ArrayList<>();
            entries.add(new Pair<>(DONE, 1.0));
            return entries.iterator();
        }

        for (int iAgent = 0; iAgent < numOfAgents; iAgent++) {
            bigGrid = agents.get(iAgent).getGrid();
            bigGridState = bigGrid.fromJointGrid(startStates.get(iAgent), this);
            List<Map.Entry<Integer, Double>> elements = new ArrayList<>();
            if (startStates.get(iAgent) == SINGLE_DONE || isInBorder(startStates.get(iAgent))) {
                elements.add(new Pair<>(SINGLE_DONE, 1.0));
                res.add(elements);
                continue;
            }
            Iterator<Map.Entry<Integer, Double>> it = bigGrid.getNonZeroTransitions(bigGridState, actions.get(iAgent));
            while (it.hasNext()) {
                Map.Entry<Integer, Double> e = it.next();
                int endState = e.getKey();
                elements.add(new Pair<>(bigGrid.toJointGrid(endState, this), e.getValue()));
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
                res.add(newElements);
            }
            else {
                res.add(elements);
            }
        }

        return new CartesianIterator(res, numOfSingleStates);
    }

    @Override
    public double R(int iStartState, int iAction) {
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
        if (isTerminalState(iStartState)) {
            return 0;
        }

        List<Integer> iStartStateValues = decodeState(iStartState);
        List<Integer> iActionValues = decodeAction(iAction);
        List<Integer> iEndStateValues = decodeState(iEndState);

        double reward = 0;
        for (int iAgent = 0; iAgent < numOfAgents; iAgent++) {
            if (!(iEndStateValues.get(iAgent) == SINGLE_DONE || iStartStateValues.get(iAgent) == SINGLE_DONE || isInBorder(iStartStateValues.get(iAgent)))) {
                if (isInBorder(iEndStateValues.get(iAgent))) {
                    reward += agents.get(iAgent).getMDPValueFunction().getValue(agents.get(iAgent).getGrid().fromJointGrid(iEndStateValues.get(iAgent), this));
                }
                else {
                    reward += agents.get(iAgent).getGrid().R(iStartStateValues.get(iAgent), iActionValues.get(iAgent), iEndStateValues.get(iAgent));
                }
            }
        }

        return reward;
    }

    @Override
    public boolean isInBorder(int iStateValue) {
        if (iStateValue == SINGLE_DONE) {
            return false;
        }

        Point loc = stateToLocation(iStateValue).add(origin);
        return loc.first().equals(origin.first()) || loc.second().equals(origin.second()) || loc.first() == origin.first() + rows-1 || loc.second() == origin.second() + cols-1;
    }

    @Override
    public void initGrid() throws InvalidModelFileFormatException {
        DONE = getStateCount() - 1;
        grid = new int[rows][cols];
        Point loc;
        int istate = 0;
        for (int i = 0; i < this.rows; i++) {
            for (int j = 0; j < this.cols; j++) {
                loc = new Point(i, j);
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
            throw new RuntimeException("Grid data does not match state number: " + stateToLocation.size() + " vs " + numOfSingleStates);
        }

        System.out.println("beacons: " + Arrays.toString(beacons.toArray()));

        System.out.println("Grid Load Complete!");
    }

    @Override
    public boolean isForbidden(int iState) {
        return isCollisionState(iState);
    }

    public boolean isCollisionState(int state) {
        if (state == DONE) {
            return false;
        }

        List<Integer> stateValues = decodeState(state);
        while (stateValues.remove((Object)SINGLE_DONE));
        return (long) stateValues.size() != stateValues.stream().distinct().count();
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
        return decode(state, numOfSingleStates);
    }

    public List<Integer> decodeAction(int action) {
        return decode(action, numOfSingleActions);
    }

    public List<Integer> decodeObservation(int observation) {
        return decode(observation, numOfSingleObservations);
    }

    private int encode(List<Integer> decoded, int numOfSingle) {
        int encoded = 0;
        for (int i = 0, power = 1; i < numOfAgents; i++, power *= numOfSingle) {
            encoded += power * decoded.get(i);
        }
        return encoded;
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
