package pomdp.environments;

import pomdp.utilities.*;

import java.io.IOException;
import java.util.*;

public class BeaconDistanceGrid extends Grid{

    protected RandomGenerator noiseGenerator;
    protected int maxNoise;
    protected int maxDist;
    protected int INF;
    protected int entry_options;

    public BeaconDistanceGrid(int numOfAgents) {
        super(numOfAgents);
        noiseGenerator = RandomGenerator.getInstance();
        maxNoise = 3;
    }

    public void setMaxDist(int maxDist) {
        this.maxDist = maxDist;
        INF = maxDist + 1;
        entry_options = maxDist + 2;
    }

    public int getINF() {
        return INF;
    }

    public int getMaxNoise() {
        return maxNoise;
    }

    @Override
    public int observe(int iAction, int iState) {
        Point iLoc = stateToLocation.get(iState);
        List<Integer> dists = beacons.stream().map(b -> {
            // Formula: 2^d / (2^(range+2) - 1)
          int dist = b.distTo(iLoc.first(), iLoc.second());
          if (dist > b.getRange())
              return INF;
          int obsDist = dist;
          double dProb = noiseGenerator.nextDouble();
          double normalizer = (1 << (dist+1)) - 1.0;
            dProb -= ((1 << (obsDist)) / normalizer);
          while (dProb > 0) {
              obsDist--;
              dProb -= ((1 << (obsDist)) / normalizer);
          }
          if (obsDist < 0) {
              throw new RuntimeException("ObsDist is " + obsDist);
          }
          return obsDist;
        }).toList();
//        System.out.println("observed dists: " + dists);
//        System.out.println();
        return distsToObs(dists);
    }

    public int distsToObs(List<Integer> dists) {
        int iObservation = 0, power = 1;
        for (int dist : dists) {
            iObservation += dist * power;
            power *= entry_options;
        }
        //        System.out.println("observe iObservation: " + iObservation);
        //        System.out.println();
        return iObservation;
    }

    public List<Integer> obsToDists(int iObservation) {
        List<Integer> dists = new ArrayList<>();
        for (Beacon b : beacons) {
            dists.add(iObservation % entry_options);
            iObservation /= entry_options;
        }

        return dists;
    }

    @Override
    public double O(int iAction, int iState, int iObservation) {
        Point iLoc = stateToLocation.get(iState);
//        System.out.println("O iObservation: " + iObservation);

        double prob = 1.0;
        int observed_dist, actual_dist;
        for (Beacon b : beacons) {
            observed_dist = iObservation % entry_options;
            iObservation /= entry_options;
            actual_dist = b.distTo(iLoc.first(), iLoc.second());

//            System.out.println("beacon's range: " + b.getRange());
//            System.out.println("O observed_dist: " + observed_dist);
//            System.out.println("O Actual dist: " + actual_dist);
//            System.out.println();

            if (observed_dist == INF) {
                prob *= actual_dist > b.getRange() ? 1.0 : 0.0;
            }
            else if (actual_dist > b.getRange() || actual_dist - observed_dist < 0) {
                prob = 0;
                return prob;
            }
            else {
                // Formula: 2^d / (2^(range+2) - 1)
                prob *= (((1 << observed_dist)) * 1.0) / ((1 << (actual_dist+1)) - 1);
            }
        }

        return prob;
    }

    @Override
    public void load(String sFileName) throws IOException, InvalidModelFileFormatException {
        m_sName = sFileName.substring( sFileName.lastIndexOf( "/" ) + 1, sFileName.lastIndexOf( "." ) );
        if (numOfAgents > 1) {
            MultiAgentBeaconDistanceGridLoader p = new MultiAgentBeaconDistanceGridLoader(this);
            p.load( sFileName );
        }
        else {
            BeaconDistanceGridLoader p = new BeaconDistanceGridLoader( this );
            p.load( sFileName );

            if( m_rtReward == RewardType.StateActionState )
                initStoredRewards();
        }

        m_vfMDP = new MDPValueFunction( this, 0.0 );
        initBeliefStateFactory();
        System.out.println();
    }
}
