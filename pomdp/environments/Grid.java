package pomdp.environments;

import pomdp.utilities.*;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

public class Grid extends POMDP {
    private int rows;
    private int cols;
    private List<Pair<Integer, Integer>> holes;
    private List<Pair<Integer, Integer>> stateToLocation;   // mapping between state to place in grid
    private int[][] grid;                                   // mapping between place in grid to state
    private List<Beacon> beacons;
    private float o_radius;                                 // base reception radius (beacons will make it smaller)

    // 0 < BASE < 1, determines the observation probabilities
    // BASE -> 1: very accurate sensors (high probability of the actual state)
    // BASE -> 0: inaccurate sensors (roughly equal probabilities)
    private final double BASE;

    public Grid() {
        super();
        holes = new ArrayList<>();
        stateToLocation = new ArrayList<>();
        beacons = new ArrayList<>();
        BASE = 0.2;
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
        int num_of_observations = m_fObservation.countNonZeroEntries(iAction, iState);
        if (num_of_observations == 0) {     // The observations for the state is yet to be initialized
            initObservation(iState);
        }

        return super.observe(iAction, iState);
    }

    @Override
    public double O(int iAction, int iState, int iObservation) {
        int num_of_observations = m_fObservation.countNonZeroEntries(iAction, iState);
        if (num_of_observations == 0) {     // The observations for the state is yet to be initialized
            initObservation(iState);
        }
        return super.O(iAction, iState, iObservation);
    }

    private int getNumValidObservations(int r, int i, int j) {
        int numOfValidObservations = 0;
        if (i-r >= 0 && grid[i-r][j] != -1)
            numOfValidObservations++;
        if (i+r < rows && grid[i+r][j] != -1)
            numOfValidObservations++;
        if (j-r >= 0 && grid[i][j-r] != -1)
            numOfValidObservations++;
        if (j+r < cols && grid[i][j+r] != -1)
            numOfValidObservations++;

        return numOfValidObservations;
    }

    public void initObservation(int iState) {
        Pair<Integer, Integer> pos = this.stateToLocation.get(iState);
        float radius = o_radius;

        int dist, i = pos.first(), j = pos.second();
        for (Beacon b : beacons) {
            dist = b.distTo(i, j);
            if (dist < b.getRange()) {
                radius = (radius / b.getRange()) * dist;
            }
        }

        int round_radius = Math.round(radius);
        int iAction = -1;   // it doesn't matter from where we got to the state

        // calculating the base observations (without beacons)
        if (grid[i][j] != -1 && !isTerminalState(grid[i][j])) {
            double sum = 0;
            double[] observations_d = new double[round_radius+1];
            for (int r = 0; r <= round_radius; r++) {
                double pow = Math.pow(BASE, r+1);
                if (r == 0)
                    sum += pow;
                else {
                    int numOfValidObservations = getNumValidObservations(r, i, j);
                    sum += numOfValidObservations * pow;     // the sensor can be wrong only to 4 sides (not "squared 8")
                }
                observations_d[r] = pow;
            }

            // normalizing to sum 1
            for (int r = 0; r <= round_radius; r++) {
                observations_d[r] /= sum;
            }

            // sets the observation values according to the calculation
            for (int r = 0; r <= round_radius; r++) {
                double o = observations_d[r];
                if (i-r >= 0 && grid[i-r][j] != -1) {
                    setObservation(iAction, grid[i][j], grid[i-r][j], o);
                }
                if (i+r < rows && grid[i+r][j] != -1) {
                    setObservation(iAction, grid[i][j], grid[i+r][j], o);
                }
                if (j-r >= 0 && grid[i][j-r] != -1) {
                    setObservation(iAction, grid[i][j], grid[i][j-r], o);
                }
                if (j+r < cols && grid[i][j+r] != -1) {
                    setObservation(iAction, grid[i][j], grid[i][j+r], o);
                }
            }
        }
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
