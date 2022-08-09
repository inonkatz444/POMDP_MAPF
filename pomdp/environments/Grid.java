package pomdp.environments;

import pomdp.utilities.*;

import java.io.IOException;
import java.util.*;

public class Grid extends POMDP {
    protected int rows;
    protected int cols;
    protected List<Point> holes;
    protected List<Point> stateToLocation;   // mapping between state to place in grid
    protected int[][] grid;                                   // mapping between place in grid to state
    protected List<Beacon> beacons;
    private float o_radius;                                 // base reception radius (beacons will make it smaller)
    private final Random oGenerator;
    private Map<Integer, Float> sigmaPerState;
    protected int numOfAgents;
    protected Point origin;

    public final int HOLE = -1;
    public int DONE;
    public final double SUCCESS_REWARD = 50.0;
    public final double FAILURE_REWARD = -20;
    public final double INTER_REWARD = -0.04;

    List<Map.Entry<Integer, Double>>[][] cachedTransitions;

    public Grid(int numOfAgents) {
        super();
        holes = new ArrayList<>();
        stateToLocation = new ArrayList<>();
        beacons = new ArrayList<>();
        oGenerator = new Random();
        oGenerator.setSeed(42);
        sigmaPerState = new HashMap<>();
        this.numOfAgents = numOfAgents;
    }

    public void setOrigin(Point origin) {
        this.origin = origin;
    }

    public Point getOrigin() {
        return origin;
    }

    public List<Beacon> getBeacons() {
        return beacons;
    }

    public List<Point> getHoles() {
        return holes;
    }

    public Point stateToLocation(int iState) {
        return stateToLocation.get(iState);
    }

    public String parseState(int iState) {
        if (iState == DONE) {
            return "DONE";
        }
        String state = stateToLocation(iState).toString();
        if (isTerminalState(iState)) {
            state += "*";
        }
        return state;
    }

    public void setRadius(float radius) {
        this.o_radius = radius;
    }

    public int getRows() {
        return rows;
    }

    public void setRows(int rows) {
        this.rows = rows;
    }

    public int getCols() {
        return cols;
    }

    public void setCols(int cols) {
        this.cols = cols;
    }

    public void addHole(int hole_row, int hole_col) {
        holes.add(Point.getPoint(hole_row, hole_col));
    }

    public void setHoles(List<Point> holes) {
        this.holes = holes;
    }

    public void addBeacon(Beacon b) {
        beacons.add(b);
    }

    public void setBeacons(List<Beacon> beacons) {
        this.beacons = beacons;
    }

    @Override
    public int observe(int iAction, int iState) {
        Point statePos = stateToLocation(iState);
        int i = statePos.first(), j = statePos.second();
        float currSigma = this.sigmaPerState.get(iState);
        int dist;
        if (currSigma == -1) {
            currSigma = o_radius;
            for (Beacon b : beacons) {
                dist = b.distTo(i, j);
                if (dist < b.getRange()) {
                    currSigma = currSigma * ((dist * dist * 1.0f)/(b.getRange() * b.getRange()));
                }
            }
            this.sigmaPerState.put(iState, currSigma);
        }


        while(true) {
            int delta_i = (int)Math.round(oGenerator.nextGaussian()*Math.sqrt(currSigma));
            int delta_j = (int)Math.round(oGenerator.nextGaussian()*Math.sqrt(currSigma));

            if (validLocation(i+delta_i, j+delta_j)) {
                return grid[i+delta_i][j+delta_j];
            }
        }

    }

    public int manhattanDistance(int state1, int state2) {
        Point loc1 = stateToLocation(state1);
        Point loc2 = stateToLocation(state2);
        return loc1.distance(loc2);
    }

    public int actualDistance(BeliefState bs, int state) {
        Iterator<Map.Entry<Integer,Double>> thisStateid = bs.getNonZeroEntries().iterator();
        int min_distance = rows + cols;

        while (thisStateid.hasNext()) {
            min_distance = Math.min(min_distance, actualDistance(thisStateid.next().getKey(), state));
        }

        return min_distance;
    }

    public int actualDistance(int state1, int state2) {
        // A*
        List<Integer> closed = new ArrayList<>();
        Map<Integer, Integer> g = new HashMap<>();
        Map<Integer, Integer> f = new HashMap<>();
        g.put(state1, 0);
        f.put(state1, manhattanDistance(state1, state2));
        PriorityQueue<Integer> open = new PriorityQueue<>((o1, o2) -> f.get(o1) - f.get(o2));
        open.add(state1);

        while (!open.isEmpty()) {
            int current = open.peek();
            if (current == state2) {
                return g.get(current);
            }
            for (int n : getNeighbors(current)) {
                int totalWeight = g.get(current) + 1;
                if (!closed.contains(n) && !open.contains(n)) {
                    g.put(n, totalWeight);
                    f.put(n, g.get(n) + manhattanDistance(n, state2));
                    open.add(n);
                }
                else {
                    if (totalWeight < g.get(n)) {
                        g.put(n, totalWeight);
                        f.put(n, g.get(n) + manhattanDistance(n, state2));

                        if (closed.contains(n)) {
                            closed.remove((Object)n);
                            open.add(n);
                        }
                    }
                }
            }

            open.remove((Object)current);
            closed.add(current);
        }
        return Integer.MAX_VALUE;
    }

    private double calculateND(double x, double stateSigma) {
        return (1/Math.sqrt(2*Math.PI*stateSigma)) * Math.exp(-(x*x)/(2*stateSigma));
    }

    @Override
    public double O(int iAction, int iState, int iObservation) {


        Point statePos, obsPos;
        statePos = stateToLocation(iState);
        obsPos = stateToLocation(iObservation);

        float currSigma = this.sigmaPerState.get(iState);
        int dist;
        if (currSigma == -1) {
            currSigma = o_radius;
            for (Beacon b : beacons) {
                dist = b.distTo(statePos.first(), statePos.second());
                if (dist < b.getRange()) {
                    currSigma = currSigma * ((dist * dist * 1.0f)/(b.getRange() * b.getRange()));
                }
            }
            this.sigmaPerState.put(iState, currSigma);
        }

        if (currSigma < 0.5) {
            if (iObservation == iState)
                return 1;
            else
                return 0;
        }
        double p_i = calculateND(obsPos.first() - statePos.first(), currSigma);
        double p_j = calculateND(obsPos.second() - statePos.second(), currSigma);
        return p_i * p_j;
    }

    public void initGrid() throws InvalidModelFileFormatException {

        DONE = getStateCount() - 1;
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
        if (stateToLocation.size() + 1 != this.m_cStates) { // +1 for DONE
            throw new InvalidModelFileFormatException("Grid data does not match state number: " + stateToLocation.size() + " vs " + this.m_cStates);
        }

        for (int i = 0; i < this.m_cStates; i++) {
            sigmaPerState.put(i, -1.0f);
        }

        System.out.println("beacons: " + Arrays.toString(beacons.toArray()));

        System.out.println("Grid Load Complete!");
    }

    public List<Integer> getNeighbors(int state) {
        List<Integer> neighbors = new ArrayList<>();
        Point loc = stateToLocation(state);
        if (validLocation(loc.first()-1, loc.second())) {
            neighbors.add(locationToState(loc.first()-1, loc.second()));
        }
        if (validLocation(loc.first()+1, loc.second())) {
            neighbors.add(locationToState(loc.first()+1, loc.second()));
        }
        if (validLocation(loc.first(), loc.second()-1)) {
            neighbors.add(locationToState(loc.first(), loc.second()-1));
        }
        if (validLocation(loc.first(), loc.second()+1)) {
            neighbors.add(locationToState(loc.first(), loc.second()+1));
        }

        return neighbors;
    }

    public boolean validLocation(int first, int second) {
        return first >= 0 && first < rows && second >= 0 && second < cols && grid[first][second] != HOLE;
    }

    public boolean validLocation(Point p) {
        return validLocation(p.first(), p.second());
    }

    public int locationToState(int first, int second) {
        return validLocation(first, second) ? grid[first][second] : HOLE;
    }

    public int locationToState(Point loc) {
        return locationToState(loc.first(), loc.second());
    }

    @Override
    public double tr(int iState1, int iAction, int iState2) {
        if (iState2 == DONE) {
            if (getEndState() == iState1 || iState1 == DONE || iAction == getActionIndex("DONE_ACT")) {
                return 1;
            }
            else {
                return 0;
            }
        }
        if (iState1 == DONE) {
            return 0;
        }
        if (getEndState() == iState1) {
            return 0;
        }
        if (iAction == getActionIndex("DONE_ACT")) {
            return 0;
        }

        Point loc1 = stateToLocation(iState1);
        Point loc2 = stateToLocation(iState2);
        double dprob = 0;
        switch (iAction) {
            case 0:
                if ((loc2.first() - loc1.first() == -1 && Objects.equals(loc2.second(), loc1.second()))) {
                    return 0.8;
                }
                else if (Objects.equals(loc2.first(), loc1.first()) && Math.abs(loc2.second() - loc1.second()) == 1) {
                    return 0.1;
                }
                else if (loc1.equals(loc2)) {
                    if (!validLocation(loc1.first() - 1, loc1.second())) {
                        dprob += 0.8;
                    }
                    if (!validLocation(loc1.first(), loc1.second() - 1)) {
                        dprob += 0.1;
                    }
                    if (!validLocation(loc1.first(), loc1.second() + 1)) {
                        dprob += 0.1;
                    }
                    return dprob;
                }
                else {
                    return 0;
                }
            case 1:
                if ((loc2.first() - loc1.first() == 1 && Objects.equals(loc2.second(), loc1.second()))) {
                    return 0.8;
                }
                else if (Objects.equals(loc2.first(), loc1.first()) && Math.abs(loc2.second() - loc1.second()) == 1) {
                    return 0.1;
                }
                else if (loc1.equals(loc2)) {
                    if (!validLocation(loc1.first() + 1, loc1.second())) {
                        dprob += 0.8;
                    }
                    if (!validLocation(loc1.first(), loc1.second() - 1)) {
                        dprob += 0.1;
                    }
                    if (!validLocation(loc1.first(), loc1.second() + 1)) {
                        dprob += 0.1;
                    }
                    return dprob;
                }
                else {
                    return 0;
                }
            case 2:
                if ((loc2.second() - loc1.second() == 1 && Objects.equals(loc2.first(), loc1.first()))) {
                    return 0.8;
                }
                else if (Objects.equals(loc2.second(), loc1.second()) && Math.abs(loc2.first() - loc1.first()) == 1) {
                    return 0.1;
                }
                else if (loc1.equals(loc2)) {
                    if (!validLocation(loc1.first(), loc1.second() + 1)) {
                        dprob += 0.8;
                    }
                    if (!validLocation(loc1.first() - 1, loc1.second())) {
                        dprob += 0.1;
                    }
                    if (!validLocation(loc1.first() + 1, loc1.second())) {
                        dprob += 0.1;
                    }
                    return dprob;
                }
                else {
                    return 0;
                }
            case 3:
                if ((loc2.second() - loc1.second() == -1 && Objects.equals(loc2.first(), loc1.first()))) {
                    return 0.8;
                }
                else if (Objects.equals(loc2.second(), loc1.second()) && Math.abs(loc2.first() - loc1.first()) == 1) {
                    return 0.1;
                }
                else if (loc1.equals(loc2)) {
                    if (!validLocation(loc1.first(), loc1.second() - 1)) {
                        dprob += 0.8;
                    }
                    if (!validLocation(loc1.first() - 1, loc1.second())) {
                        dprob += 0.1;
                    }
                    if (!validLocation(loc1.first() + 1, loc1.second())) {
                        dprob += 0.1;
                    }
                    return dprob;
                }
                else {
                    return 0;
                }
            default:
                if (iState1 == iState2) {
                    return 1;
                }
                else {
                    return 0;
                }
        }
    }

    @Override
    public Iterator<Map.Entry<Integer, Double>> getNonZeroTransitions(int iStartState, int iAction) {
        if (cachedTransitions[iStartState][iAction] == null) {
            List<Map.Entry<Integer, Double>> entries = new ArrayList<>();
            if (iStartState == getEndState() || iStartState == DONE || iAction == getActionIndex("DONE_ACT")) {
                entries.add(new Pair<>(DONE, 1.0));
                return entries.iterator();
            }

            Point startLoc = stateToLocation(iStartState);
            double selfProb = 0;
            switch (iAction) {
                case 0:
                    if (validLocation(startLoc.first() - 1, startLoc.second())) {
                        entries.add(new Pair<>(locationToState(startLoc.first() - 1, startLoc.second()), 0.8));
                    }
                    else {
                        selfProb += 0.8;
                    }
                    if (validLocation(startLoc.first(), startLoc.second() - 1)) {
                        entries.add(new Pair<>(locationToState(startLoc.first(), startLoc.second() - 1), 0.1));
                    }
                    else {
                        selfProb += 0.1;
                    }
                    if (validLocation(startLoc.first(), startLoc.second() + 1)) {
                        entries.add(new Pair<>(locationToState(startLoc.first(), startLoc.second() + 1), 0.1));
                    }
                    else {
                        selfProb += 0.1;
                    }

                    if (selfProb > 0) {
                        entries.add(new Pair<>(iStartState, selfProb));
                    }
                    break;
                case 1:
                    if (validLocation(startLoc.first() + 1, startLoc.second())) {
                        entries.add(new Pair<>(locationToState(startLoc.first() + 1, startLoc.second()), 0.8));
                    }
                    else {
                        selfProb += 0.8;
                    }
                    if (validLocation(startLoc.first(), startLoc.second() - 1)) {
                        entries.add(new Pair<>(locationToState(startLoc.first(), startLoc.second() - 1), 0.1));
                    }
                    else {
                        selfProb += 0.1;
                    }
                    if (validLocation(startLoc.first(), startLoc.second() + 1)) {
                        entries.add(new Pair<>(locationToState(startLoc.first(), startLoc.second() + 1), 0.1));
                    }
                    else {
                        selfProb += 0.1;
                    }

                    if (selfProb > 0) {
                        entries.add(new Pair<>(iStartState, selfProb));
                    }
                    break;
                case 2:
                    if (validLocation(startLoc.first(), startLoc.second() + 1)) {
                        entries.add(new Pair<>(locationToState(startLoc.first(), startLoc.second() + 1), 0.8));
                    }
                    else {
                        selfProb += 0.8;
                    }
                    if (validLocation(startLoc.first() - 1, startLoc.second())) {
                        entries.add(new Pair<>(locationToState(startLoc.first() - 1, startLoc.second()), 0.1));
                    }
                    else {
                        selfProb += 0.1;
                    }
                    if (validLocation(startLoc.first() + 1, startLoc.second())) {
                        entries.add(new Pair<>(locationToState(startLoc.first() + 1, startLoc.second()), 0.1));
                    }
                    else {
                        selfProb += 0.1;
                    }

                    if (selfProb > 0) {
                        entries.add(new Pair<>(iStartState, selfProb));
                    }
                    break;
                case 3:
                    if (validLocation(startLoc.first(), startLoc.second() - 1)) {
                        entries.add(new Pair<>(locationToState(startLoc.first(), startLoc.second() - 1), 0.8));
                    }
                    else {
                        selfProb += 0.8;
                    }
                    if (validLocation(startLoc.first() - 1, startLoc.second())) {
                        entries.add(new Pair<>(locationToState(startLoc.first() - 1, startLoc.second()), 0.1));
                    }
                    else {
                        selfProb += 0.1;
                    }
                    if (validLocation(startLoc.first() + 1, startLoc.second())) {
                        entries.add(new Pair<>(locationToState(startLoc.first() + 1, startLoc.second()), 0.1));
                    }
                    else {
                        selfProb += 0.1;
                    }

                    if (selfProb > 0) {
                        entries.add(new Pair<>(iStartState, selfProb));
                    }
                    break;
                default:
                    entries.add(new Pair<>(iStartState, 1.0));
                    break;
            }
            cachedTransitions[iStartState][iAction] = entries;
        }

        return cachedTransitions[iStartState][iAction].iterator();
    }

    @Override
    public void load(String sFileName, char id) throws IOException, InvalidModelFileFormatException{
        m_sName = sFileName.substring( sFileName.lastIndexOf( "/" ) + 1, sFileName.lastIndexOf( "." ) );
        if (numOfAgents > 1) {
            MultiAgentGridLoader p = new MultiAgentGridLoader(this);
            p.load( sFileName );
        }
        else {
            GridLoader p = new GridLoader( this );
            p.load( sFileName );

            if( m_rtReward == RewardType.StateActionState )
                initStoredRewards();
        }

        m_vfMDP = new MDPValueFunction( this, 0.0 );
        initBeliefStateFactory();
        System.out.println();
    }

    public List<Point> getStateToLocation() {
        return stateToLocation;
    }

    public int getNumOfAgents() {
        return numOfAgents;
    }

    // return: [upperLeft, lowerRight]
    public Point[] getBoundaryPoints(List<BeliefState> beliefs, Set<Integer> collisionStates) {
        int left = cols-1, up = rows-1, right = 0, down = 0;
        for (BeliefState belief : beliefs) {
            for (Map.Entry<Integer, Double> stateD : belief.getNonZeroEntries()) {
                Point p = stateToLocation(stateD.getKey());

                up = Math.min(up, p.first());
                down = Math.max(down, p.first());
                left = Math.min(left, p.second());
                right = Math.max(right, p.second());
            }

            for (Integer collisionState : collisionStates) {
                Point p = stateToLocation(collisionState);

                up = Math.min(up, p.first());
                down = Math.max(down, p.first());
                left = Math.min(left, p.second());
                right = Math.max(right, p.second());
            }
        }

        return new Point[]{
                Point.getPoint(Math.max(up-1, 0), Math.max(left-1, 0)),
                Point.getPoint(Math.min(down+1, rows-1), Math.min(right+1, cols-1))
        };
    }

    public boolean isInBorder(int iState) {
        if (iState == DONE) {
            return false;
        }

        Point loc = stateToLocation(iState).add(origin);
        return loc.first().equals(origin.first()) || loc.second().equals(origin.second()) || loc.first() == origin.first() + rows-1 || loc.second() == origin.second() + cols-1;
    }

    public int fromJointGrid(int iState, JointBeaconDistanceGrid g) {
        if (iState == g.SINGLE_DONE) {
            return DONE;
        }
        return locationToState(g.stateToLocation(iState).add(g.getOrigin()));
    }

    public int toJointGrid(int iState, JointBeaconDistanceGrid g) {
        if (iState == DONE) {
            return g.SINGLE_DONE;
        }
        return g.locationToState(stateToLocation(iState).add(getOrigin()));
    }

    @Override
    public double R(int iStartState) {
        double reward = 0;
        for (int iAction = 0; iAction < m_cActions; iAction++) {
            reward += R(iStartState, iAction);
        }
        return reward / m_cActions;
    }

    @Override
    public double R(int iStartState, int iAction) {
        if (iStartState == DONE)
            return 0;
        if (iAction == getActionIndex("DONE_ACT")) {
            if (iStartState == getEndState())
                return SUCCESS_REWARD;
            else {
                return FAILURE_REWARD;
            }
        }
        return INTER_REWARD;
    }

    @Override
    public double R(int iStartState, int iAction, int iEndState) {
        return R(iStartState, iAction);
    }

    public void initTransitionCaching() {
        cachedTransitions = new List[m_cStates][m_cActions];
    }
}
