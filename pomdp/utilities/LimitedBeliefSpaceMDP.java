package pomdp.utilities;

import java.util.Collection;
import java.util.Map;
import java.util.TreeMap;
import java.util.Iterator;
import java.util.Vector;
import java.util.Map.Entry;

import pomdp.environments.POMDP;
import pomdp.environments.POMDP.IntegerCollection;
import pomdp.utilities.datastructures.DoubleVector;

public class LimitedBeliefSpaceMDP extends MDPValueFunction {
	private int m_cOriginalStates;
	private Vector<Map<Integer, Double>>[] m_aTransitions;
	private Vector<Map<Integer, Double>>[] m_aObservationProbabilities;
	private Vector<BeliefState> m_vStates;
	private LimitedBeliefStateFactory m_lbsFactory;
	
	private int m_cMaxEntries = Integer.MAX_VALUE;
	//private Map<LimitedBelief, Integer> m_mStateIndexes;
	
	public LimitedBeliefSpaceMDP( POMDP pomdp ){
		super( pomdp, 0.0 );
		m_lbsFactory = new LimitedBeliefStateFactory( m_pPOMDP, 1 );
		m_lbsFactory.cacheBeliefStates( true );
		m_cOriginalStates = m_pPOMDP.getStateCount();
		computeValidStates();
		verifyModelParameters();
		m_adValues = new DoubleVector( m_cStates );
	}
	
	private void verifyModelParameters() {
		int iState = 0, iAction = 0;
		double dSum = 0.0;
		Map<Integer, Double> mTr = null, mOb = null;
		for( iState = 0 ; iState < m_vStates.size() ; iState++ ){
			for( iAction = 0 ; iAction < m_cActions ; iAction++ ){
				dSum = 0.0;
				mTr = m_aTransitions[iAction].elementAt( iState );
				for( Entry<Integer, Double> eTr : mTr.entrySet() ){
					dSum += eTr.getValue();
				}
				if( dSum < 0.99 || dSum > 1.01 )
					Logger.getInstance().logError( "LimitedBeliefMDP", "verifyModelParameters", "Uncalibrated transitions" );
				dSum = 0.0;
				mOb = m_aObservationProbabilities[iAction].elementAt( iState );
				for( Entry<Integer, Double> eTr : mTr.entrySet() ){
					dSum += eTr.getValue();
				}
				if( dSum < 0.99 || dSum > 1.01 )
					Logger.getInstance().logError( "LimitedBeliefMDP", "verifyModelParameters", "Uncalibrated observations" );
			}
		}
		
	}

	protected double tr( int iS1, int iAction, int iS2 ){
		
		Double dValue = m_aTransitions[iAction].elementAt( iS1 ).get( iS2 );
		if( dValue == null )
			return 0.0;
		return dValue;
	}
	protected double O( int iAction, int iState, int iObservation ){
		Double dValue = m_aObservationProbabilities[iAction].elementAt( iState ).get( iObservation );
		if( dValue == null )
			return 0.0;
		return dValue;		
	}
	protected double R( int iState, int iAction ){
		BeliefState lbState = getState( iState );
		double dR = m_pPOMDP.immediateReward( lbState, iAction );
		return dR;
	}
	protected Iterator<Entry<Integer,Double>> getNonZeroTransitions( int iStartState, int iAction ){
		return m_aTransitions[iAction].elementAt( iStartState ).entrySet().iterator();
	}
	private int getStateIndex( BeliefState lbState ) {
		return m_vStates.indexOf( lbState );
	}
	private BeliefState getState( int iState ){
		return m_vStates.elementAt( iState );
	}
	
	public Collection<Integer> getValidStates(){
		return new IntegerCollection( 0, m_vStates.size() );
	}
	
	protected void computeValidStates(){
		m_cStates = 0;
		m_aTransitions = (Vector<Map<Integer, Double>>[])new Vector[m_pPOMDP.getActionCount()];
		m_aObservationProbabilities = (Vector<Map<Integer, Double>>[])new Vector[m_pPOMDP.getActionCount()];
		//m_mStateIndexes = new TreeMap<LimitedBelief, Integer>();
		m_vStates = new Vector<BeliefState>();
		int iCurrent = 0, iAction = 0;
		
		for( iAction = 0 ; iAction < m_pPOMDP.getActionCount() ; iAction++ ){
			m_aTransitions[iAction] = new Vector<Map<Integer,Double>>();
			m_aObservationProbabilities[iAction] = new Vector<Map<Integer,Double>>();
		}
		
				
		m_vStates.add( m_lbsFactory.getInitialBeliefState() );
		iCurrent = 0;
		while( iCurrent < m_vStates.size() ){
			addState( iCurrent );
			iCurrent++;
			if( iCurrent % 1000 == 0 )
				Logger.getInstance().logFull( "LimitedBeliefSpaceMDP", 0, "computeValidStates", "Done " + iCurrent + " states. |Q| = " + ( m_vStates.size() - iCurrent ) );
		}
		m_cStates = m_vStates.size();
		System.out.println( "Done computing model. |S| = " + m_vStates.size() );
	}
	
	private void addState( int iCurrent ) {
		Map<Integer, Double> mTransitions = null;
		Map<Integer, Double> mObservations = null;
		int iObservation = 0, iAction = 0, iNextState = 0;
		double dPr = 0.0;
		BeliefState bsCurrent = m_vStates.elementAt( iCurrent ), bsSuccessor = null;
		int cCurrentNonZero = bsCurrent.getNonZeroEntriesCount(), cSuccessorNonZero = 0;
		//if( bsCurrent.getId() == 68 )
		//	System.out.println( bsCurrent );

		//System.out.println( bsCurrent );
		
		for( iAction = 0 ; iAction < m_cActions ; iAction++ ){
			mTransitions = new TreeMap<Integer, Double>();
			mObservations = new TreeMap<Integer, Double>();
			for( iObservation = 0 ; iObservation < m_cObservations ; iObservation++ ){
				bsSuccessor = bsCurrent.nextBeliefState( iAction, iObservation );
				if( bsSuccessor != null ){				
					//dPr = observationProbability( bsSuccessor, iAction, iObservation );
					dPr = bsCurrent.probabilityOGivenA( iAction, iObservation );
					cSuccessorNonZero = bsSuccessor.getNonZeroEntriesCount();
					if( ( cSuccessorNonZero < cCurrentNonZero ) ||
							( cSuccessorNonZero <= m_cMaxEntries ) ){
						if( bsSuccessor.getId() >= m_vStates.size() ){// new state
							m_vStates.add( bsSuccessor );
						}
						iNextState = bsSuccessor.getId();
					}
					else{
						iNextState = bsCurrent.getId();
					}
					if( !mTransitions.containsKey( iNextState ) )
						mTransitions.put( iNextState, 0.0 );
					mTransitions.put( iNextState, mTransitions.get( iNextState ) + dPr );
					mObservations.put( iObservation, dPr );
				}
				/*
				dPr = bsCurrent.probabilityOGivenA( iAction, iObservation );
				if( dPr > 0.0 ){
					bsSuccessor = bsCurrent.nextBeliefState( iAction, iObservation );
					cSuccessorNonZero = bsSuccessor.getNonZeroEntriesCount();
					if( ( cSuccessorNonZero < cCurrentNonZero ) ||
							( cSuccessorNonZero <= m_cMaxEntries ) ){
						if( bsSuccessor.getId() >= m_vStates.size() ){// new state
							m_vStates.add( bsSuccessor );
						}
						iNextState = bsSuccessor.getId();
					}
					else{
						iNextState = bsCurrent.getId();
					}
					if( !mTransitions.containsKey( iNextState ) )
						mTransitions.put( iNextState, 0.0 );
					mTransitions.put( iNextState, mTransitions.get( iNextState ) + dPr );
					mObservations.put( iObservation, dPr );
				}
				*/
			}
			
			m_aObservationProbabilities[iAction].add( mObservations );
			m_aTransitions[iAction].add( mTransitions );
		}
	}

	private double observationProbability(BeliefState bsSuccessor, int iAction,
			int iObservation) {
		Collection<Entry<Integer, Double>> cNonZeroBeliefs = bsSuccessor.getNonZeroEntries();
		double dPr = 0.0, dOb = 0.0;
		for( Entry<Integer, Double> eBelief : cNonZeroBeliefs ){
			dOb = m_pPOMDP.O( iAction, eBelief.getKey(), iObservation );
			dPr += dOb;
		}
		return dPr / cNonZeroBeliefs.size();
	}

	public AlphaVector newAlphaVector(){
		AlphaVector av = new TabularAlphaVector( null, -1.0, m_pPOMDP );
		av.setSize( m_cStates );
		return av;
	}
	/**
	 * Attempts to load the value function from a file. If the value function file is not found it computes the value function and saves it to a file.
	 */
	public void valueIteration( int cMaxIterations, double dEpsilon ){
		int cMDPBackups = 0, iStartState = 0;
		double dMaxDelta = Double.MAX_VALUE;
		
		Logger.getInstance().logFull( "LimitedBeliefMDPVF", 0, "VI", "Starting MDP value iteration" );
		try{
			dMaxDelta = computeValueFunction( cMaxIterations, dEpsilon, false );
		}
		catch( Error e ){
			Logger.getInstance().logError( "LimitedBeliefMDPVF", "VI", "Error in computeVN: " + e );
			throw e;
		}
		
		makeVectors();
		Logger.getInstance().logFull( "LimitedBeliefMDPVF", 0, "VI", "MDP value iteration done - iterations " + cMDPBackups + " delta " + dMaxDelta );
	}
	
	
	public int execute( int iAction, int iState ){
		Map<Integer, Double> mTransitions = m_aTransitions[iAction].elementAt( iState );
		double dProb = m_rndGenerator.nextDouble();
		for( Entry<Integer, Double> e : mTransitions.entrySet() ){
			dProb -= e.getValue();
			if( dProb < 0.0 )
				return e.getKey();
		}
		return -1;
	}

	public int getSuccessor( int iState, int iAction, int iObservation ){
		BeliefState bs = m_vStates.elementAt( iState );
		BeliefState bsSuccessor = bs.nextBeliefState( iAction, iObservation );
		if( bsSuccessor == null )
			bsSuccessor = bs.nextBeliefState( iAction, iObservation );
		return bsSuccessor.getId();
	}
	
	public int observe( int iState, int iAction ){
		Map<Integer, Double> mObservations = m_aObservationProbabilities[iAction].elementAt( iState );
		double dProb = m_rndGenerator.nextDouble();
		for( Entry<Integer, Double> e : mObservations.entrySet() ){
			dProb -= e.getValue();
			if( dProb < 0.0 )
				return e.getKey();
		}
		return -1;
	}
	
	public String getStateName( int iState ){
		BeliefState bs = m_vStates.elementAt( iState );
		return bs.toString();
	}

	public int sampleRealState( int iState ) {
		BeliefState bs = getState( iState );
		Iterator<Entry<Integer, Double>> itNonZero = bs.getNonZeroEntries().iterator();
		Entry<Integer, Double> e = null;
		double dProb = m_rndGenerator.nextDouble();
		while( itNonZero.hasNext() ){
			e = itNonZero.next();
			dProb -= e.getValue();
			if( dProb <= 0.0 )
				return e.getKey();
		}
		return -1;
	}
}
