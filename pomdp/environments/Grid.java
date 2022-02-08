package pomdp.environments;

import pomdp.utilities.*;

import java.io.IOException;
import java.util.*;

public class Grid extends POMDP {
    private int rows;
    private int cols;
    private List<Pair<Integer, Integer>> holes;
    private List<Pair<Integer, Integer>> stateToLocation;   // mapping between state to place in grid
    private int[][] grid;                                   // mapping between place in grid to state
    private List<Beacon> beacons;
    private float o_radius;                                 // base reception radius (beacons will make it smaller)
    private final Random oGenerator;
    private Map<Integer, Float> sigmaPerState;

    public Grid() {
        super();
        holes = new ArrayList<>();
        stateToLocation = new ArrayList<>();
        beacons = new ArrayList<>();
        oGenerator = new Random();
        sigmaPerState = new HashMap<>();
    }

    public void printGrid() {
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (grid[i][j] == -1)
                    System.out.print("   ");
                else
                    System.out.print(grid[i][j] + " ");
            }
            System.out.println();
        }
        System.out.println();
    }

    public void setRadius(float radius) {
        this.o_radius = radius;
    }

    public List<Pair<Integer, Integer>> getStateToLocation() {
        return this.stateToLocation;
    }

    public void setRows(int rows) {
        this.rows = rows;
    }

    public void setCols(int cols) {
        this.cols = cols;
    }

    public void addHole(int hole_row, int hole_col) {
        holes.add(new Pair<>(hole_row, hole_col));
    }

    public void addBeacon(int beacon_row, int beacon_col, int range) {
        beacons.add(new Beacon(beacon_row, beacon_col, range));
    }

    @Override
    public int observe(int iAction, int iState) {


        Pair<Integer, Integer> statePos = stateToLocation.get(iState);
        int i = statePos.first(), j = statePos.second();
        float currSigma = this.sigmaPerState.get(iState);
        int dist;
        if (currSigma == -1) {
            currSigma = o_radius;
            for (Beacon b : beacons) {
                dist = b.distTo(i, j);
                if (dist < b.getRange()) {
                    currSigma = (currSigma / b.getRange()) * dist;
                }
            }
            this.sigmaPerState.put(iState, currSigma);
        }


        while(true) {
            int delta_i = (int)Math.round(oGenerator.nextGaussian()*Math.sqrt(currSigma));
            int delta_j = (int)Math.round(oGenerator.nextGaussian()*Math.sqrt(currSigma));

            if (i+delta_i >= 0 && i+delta_i < rows && j+delta_j >= 0 && j+delta_j < cols && grid[i+delta_i][j+delta_j] != -1) {
                return grid[i+delta_i][j+delta_j];
            }
        }

    }

    private double calculateND(double x, double stateSigma) {
        return (1/Math.sqrt(2*Math.PI*stateSigma)) * Math.exp(-(x*x)/(2*stateSigma));
    }

    @Override
    public double O(int iAction, int iState, int iObservation) {


        Pair<Integer, Integer> statePos, obsPos;
        statePos = stateToLocation.get(iState);
        obsPos = stateToLocation.get(iObservation);

        float currSigma = this.sigmaPerState.get(iState);
        int dist;
        if (currSigma == -1) {
            currSigma = o_radius;
            for (Beacon b : beacons) {
                dist = b.distTo(statePos.first(), statePos.second());
                if (dist < b.getRange()) {
                    currSigma = (currSigma / b.getRange()) * dist;
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
        Pair<Integer, Integer> loc;
        int istate = 0;
        for (int i = 0; i < this.rows; i++) {
            for (int j = 0; j < this.cols; j++) {
                loc = new Pair<>(i, j);
                if (!holes.contains(loc)) {
                    stateToLocation.add(loc);
                    grid[i][j] = istate++;
                }
                else {
                    grid[i][j] = -1;
                }
            }
        }
        if (stateToLocation.size() != this.m_cStates) {
            throw new InvalidModelFileFormatException("Grid data does not match state number");
        }

        for (int i = 0; i < this.m_cStates; i++) {
            sigmaPerState.put(i, -1.0f);
        }

        System.out.println("Grid Load Complete!");
    }

    @Override
    public void load( String sFileName ) throws IOException, InvalidModelFileFormatException{
        m_sName = sFileName.substring( sFileName.lastIndexOf( "/" ) + 1, sFileName.lastIndexOf( "." ) );
        GridLoader p = new GridLoader( this );
        p.load( sFileName );
        if( m_rtReward == RewardType.StateActionState )
            initStoredRewards();

        m_vfMDP = new MDPValueFunction( this, 0.0 );
        initBeliefStateFactory();
        System.out.println();
    }
}
