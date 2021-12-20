package pomdp.utilities;

import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;

import pomdp.environments.POMDP;
import pomdp.environments.WordComplete;

public class WordCompleteBeliefStateFactory extends BeliefStateFactory {

	private WordComplete m_wcPOMDP;
	
	public WordCompleteBeliefStateFactory( WordComplete pomdp ) {
		super(pomdp);
		m_wcPOMDP = pomdp;
	}
	
	//pr(o|a,b) = \sum_s b(s) \sum_s' tr(s,a,s')O(a,s',o)
	public double calcNormalizingFactor( BeliefState bs, int iAction, int iObservation ){
		
		double dNormalizingFactor = 0.0, dO = 0.0;
		int iEndState = 0, iStartState = 0, iTrueObservation = 0;
		
		Iterator<Entry<Integer, Double>> itNonZeroEntries = bs.getNonZeroEntries().iterator();
		Entry<Integer, Double> e = null;
		while( itNonZeroEntries.hasNext() ){
			e = itNonZeroEntries.next();
			iStartState = e.getKey();
			iTrueObservation = m_wcPOMDP.observeForward( iStartState, iAction );
			if( iTrueObservation == iObservation ){
				iEndState = m_wcPOMDP.execute( iAction, iStartState );
				dNormalizingFactor += e.getValue();
			}
		}
		
		return dNormalizingFactor;
	}
	

	public BeliefState nextBeliefState( BeliefState bs, int iAction, int iObservation ){
		try{
			BeliefState bsNext = newBeliefState();
			
			double dNormalizingFactor = 0.0, dPreviousValue = 0.0, dNextValue = 0.0, dSum = 0.0;
			int iEndState = 0, iStartState = 0, iTrueObservation = 0;
			int cStates = m_pPOMDP.getStateCount();
			
			long lTimeBefore = 0, lTimeAfter = 0;
			
			if( ExecutionProperties.getReportOperationTime() )
				lTimeBefore = JProf.getCurrentThreadCpuTimeSafe();
			if( m_bCountBeliefUpdates )
				m_cBeliefUpdates++;
			
			dNormalizingFactor = 0.0;
			
			Iterator<Entry<Integer, Double>> itNonZeroEntries = bs.getNonZeroEntries().iterator();
			Entry<Integer, Double> e = null;
			while( itNonZeroEntries.hasNext() ){
				e = itNonZeroEntries.next();
				iStartState = e.getKey();
				iTrueObservation = m_wcPOMDP.observeForward( iStartState, iAction );
				if( iTrueObservation == iObservation ){
					iEndState = m_wcPOMDP.execute( iAction, iStartState );
					dPreviousValue = bsNext.valueAt( iEndState );
	 				bsNext.setValueAt( iEndState, dPreviousValue + e.getValue() );
					dNormalizingFactor += e.getValue();
				}
			}
			if( dNormalizingFactor == 0.0 )
				return null;
			itNonZeroEntries = bsNext.getNonZeroEntries().iterator();
			while( itNonZeroEntries.hasNext() ){
				e = itNonZeroEntries.next();
				iEndState = e.getKey();
				dNextValue =  e.getValue();
				bsNext.setValueAt( iEndState, dNextValue / dNormalizingFactor );
				dSum += dNextValue / dNormalizingFactor;
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

}
