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
    private List<Float> sigmaPerState;

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
        oGenerator = new Random();
        sigmaPerState = new ArrayList<>();
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
//        int num_of_observations = m_fObservation.countNonZeroEntries(iAction, iState);
//        if (num_of_observations == 0) {     // The observations for the state is yet to be initialized
//            initObservation(iState);
//        }
//
//        return super.observe(iAction, iState);


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
            this.sigmaPerState.set(iState, currSigma);
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
//        int num_of_observations = m_fObservation.countNonZeroEntries(iAction, iState);
//        if (num_of_observations == 0) {     // The observations for the state is yet to be initialized
//            initObservation(iState);
//            double dSumO = 0.0;
//            Iterator<Map.Entry<Integer,Double>> itNonZero = null;
//            Map.Entry<Integer,Double> e = null;
//            itNonZero = this.getNonZeroObservations( iAction, iState );
//            while( itNonZero.hasNext() ){
//                e = itNonZero.next();
//                double o = e.getKey();
//                double dO = e.getValue();
//                dSumO += dO;
//            }
//            if( Math.abs( dSumO - 1.0 ) > 0.0001 )
//                System.out.println( "sum O( " + iAction + ", " + iState + ", * ) = " + dSumO );
//        }
//        return super.O(iAction, iState, iObservation);
//        System.out.println("Calculate obs prob");


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
            this.sigmaPerState.set(iState, currSigma);
        }

        if (currSigma == 0) {
            if (iObservation == iState)
                return 1;
            else
                return 0;
        }
        double p_i = calculateND(obsPos.first() - statePos.first(), Math.sqrt(currSigma));
        double p_j = calculateND(obsPos.second() - statePos.second(), Math.sqrt(currSigma));
//        System.out.println("p_i = " + p_i);
//        System.out.println("p_j = " + p_j);
        return p_i * p_j;
    }

    private int getNumValidObservations(int r, int i, int j) {
        int numOfValidObservations = 0;
        if (i-r >= 0 && grid[i-r][j] != -1)
            numOfValidObservations++;
        if (i-r >= 0 && j-r >= 0 && grid[i-r][j-r] != -1)
            numOfValidObservations++;
        if (j-r >= 0 && grid[i][j-r] != -1)
            numOfValidObservations++;
        if (i+r < rows && j-r >= 0 && grid[i+r][j-r] != -1)
            numOfValidObservations++;
        if (i+r < rows && grid[i+r][j] != -1)
            numOfValidObservations++;
        if (i+r < rows && j+r < cols && grid[i+r][j+r] != -1)
            numOfValidObservations++;
        if (j+r < cols && grid[i][j+r] != -1)
            numOfValidObservations++;
        if (i-r >= 0 && j+r < cols && grid[i-r][j+r] != -1)
            numOfValidObservations++;

        return numOfValidObservations;
    }

    public void initObservation(int iState) {
        if (isTerminalState(iState)) {
            setObservation(-1, iState, iState, 1);
            return;
        }
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
                    sum += numOfValidObservations * pow;     // the sensor can be wrong only to square 8
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
                if (i-r >= 0 && j-r >= 0 && grid[i-r][j-r] != -1) {
                    setObservation(iAction, grid[i][j], grid[i-r][j-r], o);
                }
                if (j-r >= 0 && grid[i][j-r] != -1) {
                    setObservation(iAction, grid[i][j], grid[i][j-r], o);
                }
                if (j-r >= 0 && i+r < rows && grid[i+r][j-r] != -1) {
                    setObservation(iAction, grid[i][j], grid[i+r][j-r], o);
                }
                if (i+r < rows && grid[i+r][j] != -1) {
                    setObservation(iAction, grid[i][j], grid[i+r][j], o);
                }
                if (i+r < rows && j+r < cols && grid[i+r][j+r] != -1) {
                    setObservation(iAction, grid[i][j], grid[i+r][j+r], o);
                }
                if (j+r < cols && grid[i][j+r] != -1) {
                    setObservation(iAction, grid[i][j], grid[i][j+r], o);
                }
                if (j+r < cols && i-r >= 0 && grid[i-r][j+r] != -1) {
                    setObservation(iAction, grid[i][j], grid[i-r][j+r], o);
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

        for (int i = 0; i < this.m_cStates; i++) {
            sigmaPerState.add(-1.0f);
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
