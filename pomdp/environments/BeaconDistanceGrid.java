package pomdp.environments;

import pomdp.utilities.*;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class BeaconDistanceGrid extends Grid{

    protected RandomGenerator noiseGenerator;
    protected int maxNoise;
    protected int maxDist;
    private int INF;

    public BeaconDistanceGrid() {
        super();
        noiseGenerator = RandomGenerator.getInstance();
        maxNoise = 3;
    }

    public void setMaxDist(int maxDist) {
        this.maxDist = maxDist;
        INF = maxDist + 1;
    }

    @Override
    public int observe(int iAction, int iState) {
        int dist = rows + cols;
        int dist_from_b = 0;
        Pair<Integer, Integer> iLoc = stateToLocation.get(iState);
        for (Beacon b : this.beacons){
            dist_from_b = b.distTo(iLoc.first(), iLoc.second());
            if (dist_from_b <= b.getRange()) {
                dist = Math.min(dist, dist_from_b);
            }
        }

        if (dist > maxDist)
            return INF;
        dist = (dist - noiseGenerator.nextInt(maxNoise));
        dist = Math.max(dist, 0);
        return dist;
    }

    @Override
    public double O(int iAction, int iState, int iObservation) {
        int dist = rows + cols;
        int dist_from_b = 0;
        Pair<Integer, Integer> iLoc = stateToLocation.get(iState);
        for (Beacon b : this.beacons){
            dist_from_b = b.distTo(iLoc.first(), iLoc.second());
            if (dist_from_b <= b.getRange()) {
                dist = Math.min(dist, dist_from_b);
            }
        }

        if (iObservation == INF) {
            if (dist >= maxDist + 1 + maxNoise)
                return 1;
            for (int i = maxNoise - 1; i >= 0; i--) {
                if (dist == maxDist + 1 + i)
                    return (i+1.0)/(maxNoise+1.0);
            }
        }

        Map<Integer, Double> probMap = new HashMap<>();
        for (int i = maxNoise; i >= 0; i--) {
            probMap.put(Math.max(dist - i, 0), probMap.getOrDefault(Math.max(dist - i, 0), 0.0) + 1.0/(maxNoise+1));
        }

        return probMap.getOrDefault(iObservation, 0.0);

    }

    @Override
    public void load( String sFileName ) throws IOException, InvalidModelFileFormatException {
        m_sName = sFileName.substring( sFileName.lastIndexOf( "/" ) + 1, sFileName.lastIndexOf( "." ) );
        BeaconDistanceGridLoader p = new BeaconDistanceGridLoader( this );
        p.load( sFileName );
        if( m_rtReward == RewardType.StateActionState )
            initStoredRewards();

        m_vfMDP = new MDPValueFunction( this, 0.0 );
        initBeliefStateFactory();
        System.out.println();
    }
}
