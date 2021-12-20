package pomdp.environments;

import java.util.Collection;
import java.util.Iterator;
import java.util.Map;
import java.util.TreeMap;
import java.util.Vector;
import java.util.Map.Entry;

import pomdp.algorithms.PolicyStrategy;
import pomdp.algorithms.pointbased.HeuristicSearchValueIteration;
import pomdp.algorithms.pointbased.PrioritizedValueIteration;
import pomdp.utilities.AlphaVector;
import pomdp.utilities.BeliefState;
import pomdp.utilities.BeliefStateFactory;
import pomdp.utilities.ExecutionProperties;
import pomdp.utilities.Logger;
import pomdp.utilities.MDPValueFunction;
import pomdp.utilities.RandomGenerator;
import pomdp.utilities.TabularAlphaVector;
import pomdp.utilities.factored.FactoredBeliefStateFactory;

public class DeterministicPOMDP extends POMDPAdapter {
	private static final long serialVersionUID = 8406127746313003632L;
	private boolean m_bDeterministicTransitions, m_bDeterministicObservations;
	private int[][] m_aMaxTransitionStates;
	private int[][] m_aMaxObservations;
	
	public DeterministicPOMDP( POMDP pomdp, boolean bDeterministicTransitions, boolean bDeterministicObservations ){
		super( pomdp );
		m_bDeterministicTransitions = bDeterministicTransitions;
		m_bDeterministicObservations = bDeterministicObservations;
		initMax();
		copyAndInitialize();
	}
	
	private void copyAndInitialize() {
		m_cStates = m_pPOMDP.m_cStates;
		m_cActions = m_pPOMDP.m_cActions;
		m_cObservations = m_pPOMDP.m_cObservations;
		//m_fStartState = m_pPOMDP.m_fStartState;
		//m_fReward = m_pPOMDP.m_fReward;
		//m_adMinActionRewards = m_pPOMDP.m_adMinActionRewards;
		//m_rtReward = m_pPOMDP.m_rtReward;
		//m_adStoredRewards = m_pPOMDP.m_adStoredRewards;
		
		m_bsFactory = new BeliefStateFactory( this );
		m_vfMDP = new MDPValueFunction( this, 0.0 );
		m_vfMDP.persistQValues( true );
	}

	public AlphaVector newAlphaVector(){
		//If I use the factored alpha vector of the contained pomdp the ADDs will be those of the non-deterministic POMDP
		return new TabularAlphaVector( null, 0.0, this );
	}
	
	public MDPValueFunction getMDPValueFunction(){
		return m_vfMDP;
	}
	
	private void initMax() {
		int iAction = 0, iObservation = 0;
		int iMaxState = 0, iMaxObservation = 0;
		double dMaxObservationProbability = 0.0, dMaxTransition = 0.0, dTr = 0.0, dO = 0.0;
		
		Logger.getInstance().log( "DPOMDP", 0, "init", "Started initializing model" );
		
		m_aMaxTransitionStates = new int[m_pPOMDP.getStateCount()][m_pPOMDP.getActionCount()];
		m_aMaxObservations = new int[m_pPOMDP.getActionCount()][m_pPOMDP.getStateCount()];

		if( m_bDeterministicObservations ){
			for( int iEndState : m_pPOMDP.getValidStates() ){
				for( iAction = 0; iAction < m_pPOMDP.getActionCount() ; iAction++ ){
					iMaxObservation = -1;
					dMaxObservationProbability = 0.0;
					for( iObservation = 0 ; iObservation < m_pPOMDP.getObservationCount() ; iObservation++ ){
						dO = m_pPOMDP.O( iAction, iEndState, iObservation );
						if( dO > dMaxObservationProbability ){
							dMaxObservationProbability = dO;
							iMaxObservation = iObservation;
						}
					}
					m_aMaxObservations[iAction][iEndState] = iMaxObservation;
				}
			}
			Logger.getInstance().log( "DPOMDP", 0, "init", "Done observations" );
		}
		
		if( m_bDeterministicTransitions ){
			for( int iStartState : m_pPOMDP.getValidStates() ){
				for( iAction = 0; iAction < m_pPOMDP.getActionCount() ; iAction++ ){
					iMaxState = -1;
					dMaxTransition = 0.0;
					for( int iEndState : m_pPOMDP.getValidStates() ){
						dTr = m_pPOMDP.tr( iStartState, iAction, iEndState );
						if( dTr > dMaxTransition ){
							dMaxTransition = dTr;
							iMaxState = iEndState;
						}
					}
					m_aMaxTransitionStates[iStartState][iAction] = iMaxState;
					//System.out.println( iStartState + ", " + iAction + ", " + iMaxState + " => " + getStateName( iStartState ) + ", " + getActionName( iAction ) + ", " + getStateName( iMaxState ) );
				}
			}
			Logger.getInstance().log( "DPOMDP", 0, "init", "Done tranisitons" );
		}
		Logger.getInstance().log( "DPOMDP", 0, "init", "Done" );
	}

	public double tr( int iStartState, int iAction, int iEndState ){
		if( !m_bDeterministicTransitions )
			return m_pPOMDP.tr( iStartState, iAction, iEndState );
		else{
			if( iEndState == m_aMaxTransitionStates[iStartState][iAction] )
				return 1.0;
			else
				return 0.0;
		}		
	}
	public double O( int iAction, int iEndState, int iObservation ){
		if( !m_bDeterministicObservations )
			return m_pPOMDP.O( iAction, iEndState, iObservation );
		else{
			try{
			if( iObservation == m_aMaxObservations[iAction][iEndState] )
				return 1.0;
			else
				return 0.0;
			}
			catch( Exception e ){
				return 0.0;
			}
		}		
	}
	public double O( int iStartState, int iAction, int iEndState, int iObservation ){
		return tr( iStartState, iAction, iEndState ) * O( iAction, iEndState, iObservation );
	}
	
	public Iterator<Entry<Integer, Double>> getNonZeroTransitions( int iStartState, int iAction ){
		if( !m_bDeterministicTransitions )
			return m_pPOMDP.getNonZeroTransitions( iStartState, iAction );
		else{
			TreeMap<Integer,Double> mTr = new TreeMap<Integer, Double>();
			try{
				mTr.put( m_aMaxTransitionStates[iStartState][iAction], 1.0 );
			}
			catch( Exception e ){
				System.out.println( e );
			}
			return mTr.entrySet().iterator();
		}
	}
	public Collection<Entry<Integer,Double>> getNonZeroBackwardTransitions( int iAction, int iEndState ) {
		if( !m_bDeterministicTransitions )
			return m_pPOMDP.getNonZeroBackwardTransitions( iAction, iEndState );
		else{
			if( m_amBackwardTransitions == null ){
				m_amBackwardTransitions = new TreeMap[m_cActions][m_cStates];
			}
			if( m_amBackwardTransitions[iAction][iEndState] == null ){
				Map<Integer,Double> mTr = new TreeMap<Integer, Double>();
				for( int iStartState = 0 ; iStartState < m_cStates ; iStartState++ ){
					if( m_aMaxTransitionStates[iStartState][iAction] == iEndState )
						mTr.put( iStartState, 1.0 );
				}
				m_amBackwardTransitions[iAction][iEndState] = mTr;
			}
			return m_amBackwardTransitions[iAction][iEndState].entrySet();
		}
	}

	public int observe( int iAction, int iState ){
		if( m_bDeterministicObservations )
			return m_aMaxObservations[iAction][iState];
		else
			return m_pPOMDP.observe( iAction, iState );
		
	}
	public int execute( int iAction, int iState ){
		if( m_bDeterministicTransitions ){
			return m_aMaxTransitionStates[iState][iAction];
		}
		else
			return m_pPOMDP.execute( iAction, iState );
	}

	public int getEndState( int iStartState, int iAction ){
		return m_aMaxTransitionStates[iStartState][iAction];
	}	
	public int getObservation( int iAction, int iEndState ){
		return m_aMaxObservations[iAction][iEndState];
	}
	public String getName(){
		return "Det" + m_pPOMDP.getName();
	}
	
	public double computeAverageDiscountedReward( int cTests, int cMaxStepsToGoal, PolicyStrategy policy ){
		return 0.0;
	}
	
	public static void main( String[] args ) throws Exception{
		RandomGenerator rnd = new RandomGenerator( "DeterministicPOMDP", 0 );
		POMDP pomdp = new POMDP();
		pomdp.load( "Models/Hallway.pomdp" );
		DeterministicPOMDP dp = new DeterministicPOMDP( pomdp, true, true );
		BeliefStateFactory bsf = pomdp.getBeliefStateFactory();
		//PrioritizedValueIteration pvi = new PrioritizedValueIteration( dp );
		HeuristicSearchValueIteration hsvi = new HeuristicSearchValueIteration( dp );
		/*
		Vector<BeliefState> vBeliefPoints = new Vector<BeliefState>();
		BeliefState bs = bsf.getInitialBeliefState();
		int iState = 0, iAction = 0, iObservation = 0;
		for( int i = 0 ; i < 10 ; i++ ){
			bs = bsf.getInitialBeliefState();
			iState = dp.chooseStartState();
			for( int j = 0 ; j < 100 ; j++ ){
				if( !vBeliefPoints.contains( bs ) )
					vBeliefPoints.add( bs );
				iAction = rnd.nextInt( dp.getActionCount() );
				iState = dp.getEndState( iState, iAction );
				iObservation = dp.getObservation( iAction, iState );
				bs = bs.nextBeliefState( iAction, iObservation );
				//System.out.println( bs );
			}
		}
		*/
		hsvi.valueIteration( 100 );
	}
}
