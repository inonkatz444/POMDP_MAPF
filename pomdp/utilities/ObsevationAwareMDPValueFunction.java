package pomdp.utilities;

import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Vector;
import java.util.Map.Entry;

import pomdp.environments.POMDP;
import pomdp.utilities.datastructures.DoubleVector;

public class ObsevationAwareMDPValueFunction extends MDPValueFunction {

	private Vector<Integer> m_vObservationStates;
	private int m_cObservationStates;
	private double m_dObservationReward = 100.0;
	private Collection<Integer> m_colValidStates;
	
	public ObsevationAwareMDPValueFunction( POMDP pomdp, double dExplorationRate ){
		super( pomdp, dExplorationRate );
		m_vObservationStates = pomdp.getObservationRelevantStates();
		m_cObservationStates = m_vObservationStates.size();
		m_cStates = pomdp.getStateCount() * (int)Math.pow( 2, m_cObservationStates );
		m_adValues = new DoubleVector( m_cStates );
		computeValidStates();
	}

	private int[] getObservationAwareState( int iState ){
		int[] aiState = new int[m_cObservationStates + 1];
		int i = 0;
		for( i = 0 ; i < m_cObservationStates ; i++ ){
			aiState[i] = iState % 2;
			iState = iState / 2;
		}
		aiState[m_cObservationStates] = iState;
		return aiState;
	}
	private int getFullState( int[] aiState ){
		int i = 0, iState = 0;
		for( i = m_cObservationStates ; i >= 0 ; i-- ){
			iState *= 2;
			iState += aiState[i];
		}
		return iState;
		
	}
	private int countObservationVariableChanges( int[] aiS1, int[] aiS2 ){
		int i = 0, cChanges = 0;
		for( i = 0 ; i < m_cObservationStates ; i++ ){
			if( aiS1[i] != aiS2[i] )
				cChanges++;
		}
		return cChanges;
	}
	private int firstObservationVariableChange( int[] aiS1, int[] aiS2 ){
		int i = 0, cChanges = 0;
		for( i = 0 ; i < m_cObservationStates ; i++ ){
			if( aiS1[i] != aiS2[i] )
				return i;
		}
		return -1;
	}
	private int getObservationStateIndex( int iState ){
		return m_vObservationStates.indexOf( iState );
	}
	
	protected double tr( int iS1, int iAction, int iS2 ){
		int[] aiS1 = getObservationAwareState( iS1 );
		int[] aiS2 = getObservationAwareState( iS2 );
		int iUnderlyingS1 = aiS1[m_cObservationStates];
		int iUnderlyingS2 = aiS2[m_cObservationStates];
		int iObservationIndex = getObservationStateIndex( iUnderlyingS1 );
		int cObservationVariablesChanges = countObservationVariableChanges( aiS1, aiS2 );
		int iObservationVariableChange = firstObservationVariableChange( aiS1, aiS2 );
		
		if( cObservationVariablesChanges == 0 ){
			if( iObservationIndex == -1 ){
				return m_pPOMDP.tr( iUnderlyingS1, iAction, iUnderlyingS2 );
			}
			else if( aiS2[iObservationIndex] == 0 ){
				return m_pPOMDP.tr( iUnderlyingS1, iAction, iUnderlyingS2 );				
			}
			else if( aiS2[iObservationIndex] == 1 ){ //will never happen because of the compare above
				return m_pPOMDP.tr( iUnderlyingS1, iAction, iUnderlyingS2 );				
			}
			else{
				return 0.0; //visiting an observation state and observation variable not turned to 0
			}
		}
		else if( cObservationVariablesChanges > 1 ){
			return 0.0; //can only visit a single observation variable each time
		}
		else{
			if( aiS2[iObservationVariableChange] == 1 ){
				return 0.0; //can't "univisit" a state
			}
			else{
				return m_pPOMDP.tr( iUnderlyingS1, iAction, iUnderlyingS2 );				
			}
		}
	}
	protected double O( int iAction, int iState, int iObservation ){
		int[] aiState = getObservationAwareState( iState );
		int iUnderlyingState = aiState[m_cObservationStates];
		return m_pPOMDP.O( iAction, iUnderlyingState, iObservation );
	}
	protected double R( int iState, int iAction ){
		int[] aiState = getObservationAwareState( iState );
		int iUnderlyingState = aiState[m_cObservationStates];
		int iObservationIndex = getObservationStateIndex( iUnderlyingState );
		double dR = m_pPOMDP.R( iUnderlyingState, iAction );
		if( ( iObservationIndex > -1 ) && ( aiState[iObservationIndex] == 1 ) ){
			dR += m_dObservationReward;
		}
		return dR;
	}
	protected Iterator<Entry<Integer,Double>> getNonZeroTransitions( int iState, int iAction ){
		HashMap<Integer,Double> mTransitions = new HashMap<Integer,Double>();
		int[] aiState = getObservationAwareState( iState );
		int iUnderlyingState = aiState[m_cObservationStates], iFullState = 0;
		int iObservationIndex = getObservationStateIndex( iState );
		Iterator<Entry<Integer,Double>> itUnderlying = m_pPOMDP.getNonZeroTransitions( iUnderlyingState, iAction );
		Entry<Integer,Double> e = null;
		if( iObservationIndex != -1 ){
			aiState[iObservationIndex] = 0;
		}
		while( itUnderlying.hasNext() ){
			e = itUnderlying.next();
			aiState[m_cObservationStates] = e.getKey();
			iFullState = getFullState( aiState );
			mTransitions.put( iFullState, e.getValue() );
		}
		return mTransitions.entrySet().iterator();
	}
	public Collection<Integer> getValidStates(){
		return m_colValidStates;
	}
	protected void computeValidStates(){
		Collection<Integer> colValidStates = m_pPOMDP.getValidStates();
		Collection<Integer> colAddObservation = new Vector<Integer>();
		int i = 0;
		for( i = 0 ; i < m_cObservationStates ; i++ ){
			for( int iState : colValidStates ){
				colAddObservation.add( iState * 2 + 0 );
				colAddObservation.add( iState * 2 + 1 );				
			}
			colValidStates = colAddObservation;
			colAddObservation = new Vector<Integer>();
		}
		m_colValidStates = colValidStates;
	}
	public int getIntialState( int iState ){
		int[] aiState = new int[m_cObservationStates + 1];
		int i = 0;
		for( i = 0 ; i < m_cObservationStates ; i++ ){
			aiState[i] = 1;
		}
		aiState[m_cObservationStates] = iState;
		return getFullState( aiState );
	}
	public AlphaVector newAlphaVector(){
		AlphaVector av = m_pPOMDP.newAlphaVector();
		av.setSize( m_cStates );
		return av;
	}
	/**
	 * Attempts to load the value function from a file. If the value function file is not found it computes the value function and saves it to a file.
	 */
	public void valueIteration( int cMaxIterations, double dEpsilon ){
		int cMDPBackups = 0, iStartState = 0;
		double dMaxDelta = Double.MAX_VALUE;
		
		Logger.getInstance().logFull( "OAMDPVF", 0, "VI", "Starting MDP value iteration" );
		try{
			dMaxDelta = computeValueFunction( cMaxIterations, dEpsilon, false );
		}
		catch( Error e ){
			Logger.getInstance().logError( "OAMDPVF", "VI", "Error in computeVN: " + e );
			throw e;
		}
		
		makeVectors();
		Logger.getInstance().logFull( "OAMDPVF", 0, "VI", "MDP value iteration done - iterations " + cMDPBackups + " delta " + dMaxDelta );
	}
	
	public int execute( int iAction, int iState ){
		int[] aiState = getObservationAwareState( iState );
		int[] aiNewState = aiState.clone();
		int iUnderlyingState = aiState[m_cObservationStates];
		int iNewState = -1, iNewUnderlyingState = -1;
		int iObservationIndex = getObservationStateIndex( iUnderlyingState );
		
		if( iObservationIndex != -1 ){
			aiNewState[iObservationIndex] = 0;
		}
		iNewUnderlyingState = m_pPOMDP.execute( iAction, iUnderlyingState );
		aiNewState[m_cObservationStates] = iNewUnderlyingState;
		iNewState = getFullState( aiNewState );
		return iNewState;
	}

	public int observe( int iAction, int iState ){
		int[] aiState = getObservationAwareState( iState );
		int iUnderlyingState = aiState[m_cObservationStates];
		return m_pPOMDP.observe( iAction, iUnderlyingState );
	}
	
	public int getUnderlyingState( int iState ){
		int[] aiState = getObservationAwareState( iState );
		int iUnderlyingState = aiState[m_cObservationStates];
		return iUnderlyingState;
	}
}
