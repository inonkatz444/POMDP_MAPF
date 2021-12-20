package pomdp.utilities;

import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.SortedSet;
import java.util.TreeMap;
import java.util.TreeSet;
import java.util.Vector;
import java.util.Map.Entry;

import pomdp.environments.POMDP;

public class LimitedBeliefStateFactory extends BeliefStateFactory {

	private int m_iMaxBeliefValues;
	
	public LimitedBeliefStateFactory( POMDP pomdp, int iMaxBeliefValues ){
		super( pomdp );
		m_iMaxBeliefValues = iMaxBeliefValues;
		//m_hmCachedBeliefStates = new TreeMap<BeliefState, BeliefState>( new BeliefStateComparator( 0.1 ) );
	}

	// all non zero entries
	public BeliefState nextBeliefStateAll( BeliefState bs, int iAction, int iObservation ){
		try{
			BeliefState bsNext = newBeliefState();
			SortedSet<Integer> vSuccessorStates = new TreeSet<Integer>();
			
			double dNormalizingFactor = 0.0;
			int iEndState = 0, iStartState = 0;
			Iterator<Entry<Integer, Double>> itNonZeroTransitions = null;
			Entry<Integer, Double> eTransitons = null;
			Iterator<Entry<Integer, Double>> itNonZeroBeliefs = null;
			Entry<Integer, Double> eBelief = null;
			double dTr = 0.0, dO = 0.0, dBelief = 0.0;
			
			long lTimeBefore = 0, lTimeAfter = 0;
			
			if( ExecutionProperties.getReportOperationTime() )
				lTimeBefore = JProf.getCurrentThreadCpuTimeSafe();
			if( m_bCountBeliefUpdates )
				m_cBeliefUpdates++;
			
			dNormalizingFactor = 0.0;
			
			itNonZeroBeliefs = bs.getNonZeroEntries().iterator();
			while( itNonZeroBeliefs.hasNext() ){
				eBelief = itNonZeroBeliefs.next();
				iStartState = eBelief.getKey();
				dBelief = eBelief.getValue(); 
				itNonZeroTransitions = m_pPOMDP.getNonZeroTransitions( iStartState, iAction );
				while( itNonZeroTransitions.hasNext() ){
					eTransitons = itNonZeroTransitions.next();
					iEndState = eTransitons.getKey();
					dTr = eTransitons.getValue();
					dO = m_pPOMDP.O( iAction, iEndState, iObservation );
					if( dO > 0.0 ){
						if( !vSuccessorStates.contains( iEndState ) )
							vSuccessorStates.add( iEndState );
					}
				}
			}
			
			if( vSuccessorStates.size() == 0 )
				return null;
			
			dBelief = 1.0 / vSuccessorStates.size();
			
			for( int iState : vSuccessorStates ){
				bsNext.setValueAt( iState, dBelief );
			}
			
			if( m_bCacheBelifStates ){
				BeliefState bsExisting = (BeliefState) m_hmCachedBeliefStates.get( bsNext );
				if( bsExisting == null ){
					//Logger.getInstance().log( "BeliefStateFactory", 0, "nextBeliefState", 
					//		"Tau( " + bs.getId() + ", " + iAction + ", " + iObservation + " ) = " + bsNext.toString() );
					cacheBeliefState( bsNext );
					m_cBeliefPoints++;
				}
				else{
					bsNext = bsExisting;
				}
						
				if( bsNext != bs )
					bsNext.addPredecessor( bs, dNormalizingFactor, iAction );
			}
			if( ExecutionProperties.getReportOperationTime() && m_bCountBeliefUpdates ){
				lTimeAfter = JProf.getCurrentThreadCpuTimeSafe();
				m_cTimeInTau += ( lTimeAfter - lTimeBefore ) / 1000;
			}

			return bsNext;
		}
		catch( Error err ){
			Runtime rtRuntime = Runtime.getRuntime();
			System.out.println( "|BeliefSpace| " + m_cBeliefPoints + ", " + err +
					" allocated " + ( rtRuntime.totalMemory() - rtRuntime.freeMemory() ) / 1000000 +
					" free " + rtRuntime.freeMemory() / 1000000 +
					" max " + rtRuntime.maxMemory() / 1000000 );

			err.printStackTrace();
			System.exit( 0 );
		}
		return null;
	}

	// only max value entries
	public BeliefState nextBeliefState( BeliefState bs, int iAction, int iObservation ){
		try{
			BeliefState bsNext = newBeliefState();
			
			double dNormalizingFactor = 0.0, dNextValue = 0.0, dSum = 0.0;
			int iEndState = 0, iStartState = 0;
			Iterator<Entry<Integer, Double>> itNonZeroTransitions = null;
			Entry<Integer, Double> eTransitons = null;
			Iterator<Entry<Integer, Double>> itNonZeroBeliefs = null;
			Entry<Integer, Double> eBelief = null;
			double dTr = 0.0, dO = 0.0, dBelief = 0.0;
			double dSumProbs = 0.0;
			double dMaxEntry = 0.0, dLastMax = Double.POSITIVE_INFINITY;
			Vector<Integer> vMaxEntries = null, vNonZeroEntries = new Vector<Integer>();
			
			long lTimeBefore = 0, lTimeAfter = 0;
			
			if( ExecutionProperties.getReportOperationTime() )
				lTimeBefore = JProf.getCurrentThreadCpuTimeSafe();
			if( m_bCountBeliefUpdates )
				m_cBeliefUpdates++;
			
			dNormalizingFactor = 0.0;
			
			itNonZeroBeliefs = bs.getNonZeroEntries().iterator();
			while( itNonZeroBeliefs.hasNext() ){
				eBelief = itNonZeroBeliefs.next();
				iStartState = eBelief.getKey();
				dBelief = eBelief.getValue(); 
				itNonZeroTransitions = m_pPOMDP.getNonZeroTransitions( iStartState, iAction );
				while( itNonZeroTransitions.hasNext() ){
					eTransitons = itNonZeroTransitions.next();
					iEndState = eTransitons.getKey();
					dTr = eTransitons.getValue();
					dO = m_pPOMDP.O( iAction, iEndState, iObservation );
					if( dO > 0.0 ){
						dSumProbs += dTr * dO * dBelief;
						dNextValue = bsNext.valueAt( iEndState ) + dTr * dO * dBelief;
						bsNext.setValueAt( iEndState, dNextValue );
						if( !vNonZeroEntries.contains( iEndState ) )
							vNonZeroEntries.add( iEndState );
					}
				}
			}
			
			bs.setProbabilityOGivenA( iAction, iObservation, dSumProbs );
			
			if( dSumProbs == 0.0 )
				return null;
			
			vMaxEntries = new Vector<Integer>();
			if( bsNext.getNonZeroEntriesCount() > m_iMaxBeliefValues ){
				dNormalizingFactor = 0.0;
				while( vMaxEntries.size() < m_iMaxBeliefValues ){
					dMaxEntry = 0.0;
					itNonZeroBeliefs = bsNext.getNonZeroEntries().iterator();
					while( itNonZeroBeliefs.hasNext() ){
						eBelief = itNonZeroBeliefs.next();
						dBelief = eBelief.getValue();
						if( ( dBelief > dMaxEntry ) && ( dBelief < dLastMax ) ){
							dMaxEntry = dBelief;
						}
					}
					itNonZeroBeliefs = bsNext.getNonZeroEntries().iterator();
					while( itNonZeroBeliefs.hasNext() ){
						eBelief = itNonZeroBeliefs.next();
						dBelief = eBelief.getValue();
						if( dBelief == dMaxEntry ){
							vMaxEntries.add( eBelief.getKey() );
							dNormalizingFactor += dBelief;
						}
					}
					dLastMax = dMaxEntry;
				}
			}
			else{
				dNormalizingFactor = dSumProbs;
				itNonZeroBeliefs = bsNext.getNonZeroEntries().iterator();
				while( itNonZeroBeliefs.hasNext() ){
					eBelief = itNonZeroBeliefs.next();
					vMaxEntries.add( eBelief.getKey() );
				}
			}
			if( dNormalizingFactor == 0.0 )
				return null;
			for( int iState : vNonZeroEntries ){
				iEndState = iState;
				dNextValue = bsNext.valueAt( iEndState );
				if( vMaxEntries.contains( iEndState ) ){
					bsNext.setValueAt( iEndState, dNextValue / dNormalizingFactor );
					dSum += dNextValue / dNormalizingFactor;
				}
				else{
					bsNext.setValueAt( iEndState, 0.0 );
				}
			}
			
			bsNext.clearZeroEntries();
			
			if( m_bCacheBelifStates ){
				BeliefState bsExisting = (BeliefState) m_hmCachedBeliefStates.get( bsNext );
				if( bsExisting == null ){
					//Logger.getInstance().log( "BeliefStateFactory", 0, "nextBeliefState", 
					//		"Tau( " + bs.getId() + ", " + iAction + ", " + iObservation + " ) = " + bsNext.toString() );
					cacheBeliefState( bsNext );
					m_cBeliefPoints++;
				}
				else{
					bsNext = bsExisting;
				}
						
				if( bsNext != bs )
					bsNext.addPredecessor( bs, dNormalizingFactor, iAction );
			}
			if( ExecutionProperties.getReportOperationTime() && m_bCountBeliefUpdates ){
				lTimeAfter = JProf.getCurrentThreadCpuTimeSafe();
				m_cTimeInTau += ( lTimeAfter - lTimeBefore ) / 1000;
			}

			return bsNext;
		}
		catch( Error err ){
			Runtime rtRuntime = Runtime.getRuntime();
			System.out.println( "|BeliefSpace| " + m_cBeliefPoints + ", " + err +
					" allocated " + ( rtRuntime.totalMemory() - rtRuntime.freeMemory() ) / 1000000 +
					" free " + rtRuntime.freeMemory() / 1000000 +
					" max " + rtRuntime.maxMemory() / 1000000 );

			err.printStackTrace();
			System.exit( 0 );
		}
		return null;
	}

}
