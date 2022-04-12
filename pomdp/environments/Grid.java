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
    protected boolean multiAgent;

    protected final int HOLE = -1;

    public Grid(boolean multiAgent) {
        super();
        holes = new ArrayList<>();
        stateToLocation = new ArrayList<>();
        beacons = new ArrayList<>();
        oGenerator = new Random();
        oGenerator.setSeed(42);
        sigmaPerState = new HashMap<>();
        this.multiAgent = multiAgent;
    }

    public List<Beacon> getBeacons() {
        return beacons;
    }

    public Point stateToLocation(int iState) {
        return stateToLocation.get(iState);
    }

    public void printGrid() {
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (grid[i][j] == HOLE)
                    System.out.print("   ");
                else
                    System.out.print(grid[i][j] + " ");
            }
            System.out.println();
        }
        System.out.println();
    }

    public String parseState(int iState) {
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
        holes.add(new Point(hole_row, hole_col));
    }

    public void addBeacon(int id, int beacon_row, int beacon_col, int range) {
        beacons.add(new Beacon(id, beacon_row, beacon_col, range));
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
        if (stateToLocation.size() != this.m_cStates) {
            throw new InvalidModelFileFormatException("Grid data does not match state number");
        }

        for (int i = 0; i < this.m_cStates; i++) {
            sigmaPerState.put(i, -1.0f);
        }

        System.out.println("beacons: " + Arrays.toString(beacons.toArray()));

        System.out.println("Grid Load Complete!");
    }

    public boolean validLocation(int first, int second) {
        return first >= 0 && first < rows && second >= 0 && second < cols && grid[first][second] != HOLE;
    }

    public int locationToState(int first, int second) {
        return validLocation(first, second) ? grid[first][second] : -1;
    }

    @Override
    public void load( String sFileName) throws IOException, InvalidModelFileFormatException{
        m_sName = sFileName.substring( sFileName.lastIndexOf( "/" ) + 1, sFileName.lastIndexOf( "." ) );
        if (multiAgent) {
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
}
