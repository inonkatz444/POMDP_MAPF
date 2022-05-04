package pomdp.environments;

import pomdp.utilities.*;

import java.util.*;

public class JointBeaconDistanceGrid extends BeaconDistanceGrid{
    protected int numOfSingleStates;
    protected int numOfSingleActions;
    protected int numOfSingleObservations;
    protected List<Character> listPosToAgentID;

    public JointBeaconDistanceGrid(int numOfAgents) {
        super(numOfAgents);
        listPosToAgentID = new ArrayList<>();
    }

    public void setAgentListPos(char id) {
        listPosToAgentID.add(id);
    }

    @Override
    public int observe(int iAction, int iState) {
        List<Integer> stateValues = decodeState(iState);
        List<Integer> actionValues = decodeAction(iAction);
        List<Integer> observationValues = new ArrayList<>();
        for (int i = 0; i < numOfAgents; i++) {
            observationValues.add(super.observe(actionValues.get(i), stateValues.get(i)));
        }
        int encodedObservation = encodeObservations(observationValues);
        System.out.print(parseObservation(encodedObservation) + " ");
        return encodedObservation;
    }

    @Override
    public double O(int iAction, int iState, int iObservation) {
        List<Integer> stateValues = decodeState(iState);
        List<Integer> actionValues = decodeAction(iAction);
        List<Integer> observationValues = decodeObservation(iObservation);
        double prob = 1;
        for (int i = 0; i < numOfAgents; i++) {
            prob *= super.O(actionValues.get(i), stateValues.get(i), observationValues.get(i));
        }
        return prob;
    }

    @Override
    public String parseState(int iState) {
        List<Integer> stateValues = decodeState(iState);
        StringBuilder sState = new StringBuilder("(");
        boolean isFinal = true;
        for (int i = 0; i < numOfAgents; i++) {
            sState.append(super.parseState(stateValues.get(i))).append(", ");
            if (stateValues.get(i) != getEndState(listPosToAgentID.get(i))) {
                isFinal = false;
            }
        }
        sState.delete(sState.length()-2, sState.length());
        sState.append(")");
        if (isFinal) {
            sState.append("*");
        }
        return sState.toString();
    }

    @Override
    public void initGrid() throws InvalidModelFileFormatException {

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
        if (stateToLocation.size() != numOfSingleStates) {
            throw new InvalidModelFileFormatException("Grid data does not match state number: " + stateToLocation.size() + " vs " + this.m_cStates);
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
        return (long) stateValues.size() != stateValues.stream().distinct().count();
    }

    public void setNumOfSingleStates(int numOfSingleStates) {
        this.numOfSingleStates = numOfSingleStates;
    }

    public void setNumOfSingleActions(int numOfSingleActions) {
        this.numOfSingleActions = numOfSingleActions;
    }

    public void setNumOfSingleObservations(int numOfSingleObservations) {
        this.numOfSingleObservations = numOfSingleObservations;
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
