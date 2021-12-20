package pomdp.environments;

import pomdp.algorithms.pointbased.HeuristicSearchValueIteration;
import pomdp.utilities.LimitedBeliefStateFactory;
import pomdp.utilities.MDPValueFunction;

public class LimitedPOMDP extends POMDPAdapter {

	private static final long serialVersionUID = -3499300998882417909L;
	private int m_cBeliefValues;
	
	public LimitedPOMDP( POMDP pomdp ){
		super( pomdp );
		m_cBeliefValues = 1;
		m_bsFactory = new LimitedBeliefStateFactory( m_pPOMDP, m_cBeliefValues );
	}
	
	public String getName(){
		return "LimitedBelief" + m_pPOMDP.getName();
	}
	

	public void initBeliefStateFactory() {
		m_bsFactory = new LimitedBeliefStateFactory( m_pPOMDP, m_cBeliefValues );		
	}


	public static void main( String[] args ) throws Exception{
		POMDP pomdp = new POMDP();
		LimitedPOMDP lp = new LimitedPOMDP( pomdp );
		pomdp.load( "Models/Hallway.pomdp" );
		HeuristicSearchValueIteration hsvi = new HeuristicSearchValueIteration( lp );
		hsvi.valueIteration( 100 );
	}
}
