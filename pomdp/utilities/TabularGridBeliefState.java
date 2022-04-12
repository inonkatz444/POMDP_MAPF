package pomdp.utilities;

import pomdp.environments.BeaconDistanceGrid;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

public class TabularGridBeliefState extends TabularBeliefState{
    private List<Integer> minDistToBeacons;
    private List<Integer> maxDistToBeacons;
    private final List<Beacon> gridBeacons;

    public TabularGridBeliefState(int cStates, int cActions, int cObservations, int id, boolean bSparse, boolean bCacheBelifStates, BeliefStateFactory bsFactory) {
        super(cStates, cActions, cObservations, id, bSparse, bCacheBelifStates, bsFactory);
        gridBeacons = ((BeaconDistanceGrid) bsFactory.getPOMDP()).getBeacons();
        minDistToBeacons = new ArrayList<>();
        maxDistToBeacons = new ArrayList<>();
    }

    public void getMinMaxDistToBeacons() {
        minDistToBeacons.clear();
        maxDistToBeacons.clear();
        BeaconDistanceGrid grid = (BeaconDistanceGrid) m_bsFactory.getPOMDP();
        int minDist, maxDist;
        for (Beacon b : grid.getBeacons()) {
            minDist = grid.getINF();
            maxDist = -1;
            for (int iState = 0; iState < grid.getStateCount(); iState++) {
                if (valueAt(iState) > 0) {
                    minDist = Math.min(minDist, b.distTo(grid.stateToLocation(iState)));
                    maxDist = Math.max(maxDist, b.distTo(grid.stateToLocation(iState)));
                }
            }
            minDistToBeacons.add(minDist);
            maxDistToBeacons.add(maxDist);
        }
    }

    @Override
    public Vector<BeliefState> computeSuccessors() {
        int iObservation = 0;
        BeliefState bsSuccessor = null;
        double dProb = 0.0;
        getMinMaxDistToBeacons();

        m_vAllSuccessors = new Vector();
        for( iObservation = 0 ; iObservation < m_cObservations ; iObservation++ ){
//            if (isNonZero(iObservation)) {
//                for( int iAction : m_bsFactory.getPOMDP().getRelevantActions(this)){
//                    dProb = probabilityOGivenA( iAction, iObservation );
//                    if( dProb > 0 ){
//                        bsSuccessor = nextBeliefState( iAction, iObservation );
//                        addSuccessor( iAction, iObservation, bsSuccessor );
//                        if( !m_vAllSuccessors.contains( bsSuccessor ) )
//                            m_vAllSuccessors.add( bsSuccessor );
//                    }
//                }
//            }
            for( int iAction : m_bsFactory.getPOMDP().getRelevantActions(this)){
                dProb = probabilityOGivenA( iAction, iObservation );
                if( dProb > 0 ){
                    bsSuccessor = nextBeliefState( iAction, iObservation );
                    addSuccessor( iAction, iObservation, bsSuccessor );
                    if( !m_vAllSuccessors.contains( bsSuccessor ) )
                        m_vAllSuccessors.add( bsSuccessor );
                }
            }
        }
        return m_vAllSuccessors;
    }

    private boolean isNonZero(int iObservation) {
        BeaconDistanceGrid grid = (BeaconDistanceGrid) m_bsFactory.getPOMDP();
        final int INF = grid.getINF();
        List<Integer> obsDists = grid.obsToDists(iObservation);
        int minDist, maxDist, obsDist;
        for (int iBeacon = 0; iBeacon < gridBeacons.size(); iBeacon++) {
            minDist = minDistToBeacons.get(iBeacon);
            maxDist = maxDistToBeacons.get(iBeacon);
            obsDist = obsDists.get(iBeacon);
            if (minDist == INF && obsDist < INF)
                return false;
            if ((minDist - obsDist > grid.getMaxNoise()) || obsDist > maxDist)
                return false;
        }
        return true;
    }

    public String toString( double dMin ){
        String sVector = "bs" + m_iID + "[", sValue = "";
        int iState = 0, iEndState = 0, iStartState = 0;
        double dValue = -1.0, dNextValue = -1.0, dSum = 0.0;
        double dPrevValue = -1000000.0;
        int cEntries = 0;

        for( iState = 0 ; iState < m_cStates ; iState++ ){
            dValue = valueAt( iState );
            dSum += dValue;
            if( dValue >= dMin ){
                cEntries++;

                sValue = dValue + "";
                if( sValue.length() > 8 )
                    sValue = sValue.substring( 0, 8 );

                if( iState < m_cStates - 1 ){
                    iEndState = iState;
                    do{
                        iEndState++;
                        dNextValue = valueAt( iEndState );
                    }while( ( iEndState < m_cStates - 1 ) && ( dValue == dNextValue ) );
                    if( dValue != dNextValue )
                        iEndState--;
                    if( iEndState == iState )
                        sVector += m_bsFactory.m_pPOMDP.parseState(iState) + "=" + sValue + ",";
                    else{
                        sVector += "(" + m_bsFactory.m_pPOMDP.parseState(iState) + "-" + m_bsFactory.m_pPOMDP.parseState(iEndState) + ")=" + sValue + ",";
                        dSum += dValue * ( iEndState - iState );
                        iState = iEndState;
                    }
                }
                else{
                    sVector += m_bsFactory.m_pPOMDP.parseState(iState) + "=" + sValue + ",";
                }
            }
        }
        if( cEntries > 0 )
            sVector = sVector.substring( 0, sVector.length() - 1 ) + "]";
        else
            sVector += "]";

        //if( diff( dSum, 1.0 ) > 0.001 )
        //	Logger.getInstance().logError( "BS", "toString", "Sum of probabilities is " + dSum + ", bs = " + sVector );

        return sVector;
    }
}
