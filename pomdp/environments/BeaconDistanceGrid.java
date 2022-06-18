package pomdp.environments;

import pomdp.utilities.*;

import java.io.IOException;
import java.util.*;

public class BeaconDistanceGrid extends Grid{

    protected RandomGenerator noiseGenerator;
    protected int maxDist;
    protected int INF;
    protected int entry_options;

    protected Map<Integer, Beacon> actionToBeacon;

    public BeaconDistanceGrid(int numOfAgents) {
        super(numOfAgents);
        noiseGenerator = RandomGenerator.getInstance();
        actionToBeacon = new HashMap<>();
    }

    public void mapPingAction(int iAction, Beacon b) {
        actionToBeacon.put(iAction, b);
    }

    public void setMaxDist(int maxDist) {
        this.maxDist = maxDist;
        INF = maxDist + 1;
        entry_options = maxDist + 2;
    }

    public int getMaxDist() {
        return this.maxDist;
    }

    public int getINF() {
        return INF;
    }

//    @Override
//    public int observe(int iAction, int iState) {
//        if (iState == DONE) {
//            return getObservationCount()-1;
//        }
//        Point iLoc = stateToLocation.get(iState);
//        List<Integer> dists = beacons.stream().map(b -> {
//            // Formula: 2^o / (2^(range+2) - 2^d)
//          int dist = b.distTo(iLoc.first(), iLoc.second());
//          if (dist > b.getRange())
//              return INF;
//          int obsDist = dist;
//          double dProb = noiseGenerator.nextDouble();
//          double normalizer = (1 << (b.getRange()+1)) - (1 << dist);
//            dProb -= ((1 << (-obsDist+dist+b.getRange())) / normalizer);
//          while (dProb > 0) {
//              obsDist++;
//              dProb -= ((1 << (-obsDist+dist+b.getRange())) / normalizer);
//          }
//          if (obsDist > b.getRange()) {
//              throw new RuntimeException("ObsDist is " + obsDist);
//          }
//          return obsDist;
//        }).toList();
////        System.out.println("observed dists: " + dists);
////        System.out.println();
//        return distsToObs(dists);
//    }

    @Override
    public int observe(int iAction, int iState) {
        if (iState == DONE) {
            return getObservationCount()-1;
        }

        if (iAction <= 5) {
            return INF;
        }

        Point iLoc = stateToLocation.get(iState);

        Beacon current_b = pingActionToBeacon(iAction);
        assert current_b != null;
        int dist = current_b.distTo(iLoc.first(), iLoc.second());
        if (dist > current_b.getRange())
            return INF;

        int obsDist = dist;
        double dProb = noiseGenerator.nextDouble();
        double normalizer = (1 << (current_b.getRange()+1)) - (1 << dist);
        dProb -= ((1 << (-obsDist+dist+current_b.getRange())) / normalizer);
        while (dProb > 0) {
            obsDist++;
            dProb -= ((1 << (-obsDist+dist+current_b.getRange())) / normalizer);
        }
        if (obsDist > current_b.getRange()) {
            throw new RuntimeException("ObsDist is " + obsDist);
        }
        return obsDist;
    }

    private Beacon pingActionToBeacon(int iAction) {
        // map between action indices to beacons (instead of parsing the action name)
//        String[] coordinates = getActionName(iAction).substring(getActionName(iAction).indexOf('<')+1, getActionName(iAction).indexOf('>')).split(",");
//        int current_b_row = Integer.parseInt(coordinates[0]);
//        int current_b_col = Integer.parseInt(coordinates[1]);
//        Beacon current_b = null;
//        for (Beacon b : beacons) {
//            if (b.getLoc() == Point.getPoint(current_b_row, current_b_col)) {
//                current_b = b;
//                break;
//            }
//        }
        return actionToBeacon.get(iAction);
    }

//    public int distsToObs(List<Integer> dists) {
//        int iObservation = 0, power = 1;
//        for (int dist : dists) {
//            iObservation += dist * power;
//            power *= entry_options;
//        }
//        //        System.out.println("observe iObservation: " + iObservation);
//        //        System.out.println();
//        return iObservation;
//    }

//    public List<Integer> obsToDists(int iObservation) {
//        List<Integer> dists = new ArrayList<>();
//        for (Beacon b : beacons) {
//            dists.add(iObservation % entry_options);
//            iObservation /= entry_options;
//        }
//
//        return dists;
//    }

//    @Override
//    public double O(int iAction, int iState, int iObservation) {
//        if (iState == DONE) {
//            return iObservation == getObservationCount()-1 ? 1.0 : 0.0;
//        }
//        if (iObservation == getObservationCount()-1) {
//            return 0;
//        }
//        Point iLoc = stateToLocation(iState);
////        System.out.println("O iObservation: " + iObservation);
//
//        double prob = 1.0;
//        int observed_dist, actual_dist;
//        for (Beacon b : beacons) {
//            observed_dist = iObservation % entry_options;
//            iObservation /= entry_options;
//            actual_dist = b.distTo(iLoc.first(), iLoc.second());
//
////            System.out.println("beacon's range: " + b.getRange());
////            System.out.println("O observed_dist: " + observed_dist);
////            System.out.println("O Actual dist: " + actual_dist);
////            System.out.println();
//
//            if (observed_dist == INF) {
//                prob *= actual_dist > b.getRange() ? 1.0 : 0.0;
//            }
//            else if (actual_dist > b.getRange() || observed_dist - actual_dist < 0 || observed_dist > b.getRange()) {
//                prob = 0;
//                return prob;
//            }
//            else {
//                // Formula: 2^o / (2^(range+2) - 2^d)
//                prob *= (((1 << (-observed_dist+actual_dist+b.getRange()))) * 1.0) / ((1 << (b.getRange()+1)) - (1 << actual_dist));
//            }
//        }
//
//        if (prob < 0) {
//            throw new RuntimeException("prob is " + prob);
//        }
//        return prob;
//    }

    @Override
    public double O(int iAction, int iState, int iObservation) {
        if (iState == DONE) {
            return iObservation == getObservationCount()-1 ? 1.0 : 0.0;
        }
        if (iObservation == getObservationCount()-1) {
            return 0;
        }

        if (iAction <= 5) {
            return iObservation == INF ? 1 : 0;
        }

        Beacon current_b = pingActionToBeacon(iAction);
        Point iLoc = stateToLocation(iState);
//        System.out.println("O iObservation: " + iObservation);

        int observed_dist, actual_dist;

        observed_dist = iObservation;
        actual_dist = current_b.distTo(iLoc.first(), iLoc.second());

        if (observed_dist == INF) {
            return actual_dist > current_b.getRange() ? 1.0 : 0.0;
        }
        else if (actual_dist > current_b.getRange() || observed_dist - actual_dist < 0 || observed_dist > current_b.getRange()) {
            return 0;
        }
        else {
            // Formula: 2^o / (2^(range+2) - 2^d)
            double prob = (((1 << (-observed_dist+actual_dist+current_b.getRange()))) * 1.0) / ((1 << (current_b.getRange()+1)) - (1 << actual_dist));
            if (prob < 0) {
                throw new RuntimeException("prob is " + prob);
            }
            return prob;
        }
    }

    @Override
    public void load(String sFileName, char id) throws IOException, InvalidModelFileFormatException {
        m_sName = sFileName.substring( sFileName.lastIndexOf( "/" ) + 1, sFileName.lastIndexOf( "." ) );
        if (numOfAgents > 1) {
            MultiAgentBeaconDistanceGridLoader p = new MultiAgentBeaconDistanceGridLoader(this);
            p.load( sFileName, id );
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

    public String parseObservation(int iObservation) {
        return iObservation == getObservationCount()-1 ? "DONE_OBS" : String.valueOf(iObservation);
    }
}
