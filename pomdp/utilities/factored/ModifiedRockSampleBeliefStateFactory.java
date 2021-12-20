package pomdp.utilities.factored;

import pomdp.environments.ModifiedRockSample;
import pomdp.utilities.BeliefState;
import pomdp.utilities.BeliefStateFactory;


public class ModifiedRockSampleBeliefStateFactory extends BeliefStateFactory {

	private ModifiedRockSample m_mrsPOMDP;
	private ModifiedRockSampleBeliefState m_bsInitial;
	
	public ModifiedRockSampleBeliefStateFactory( ModifiedRockSample mrsPOMDP ){
		super( mrsPOMDP );
		m_mrsPOMDP = mrsPOMDP;
		m_bsInitial = new ModifiedRockSampleBeliefState( m_mrsPOMDP );
	}
	protected BeliefState newBeliefState(){
		return new ModifiedRockSampleBeliefState( m_mrsPOMDP );
	}
	public BeliefState getInitialBeliefState(){
		return m_bsInitial;
	}
	
	
	public BeliefState getRandomBeliefState() {
		int iState = 0;
		ModifiedRockSampleBeliefState bs = new ModifiedRockSampleBeliefState( m_mrsPOMDP );
		bs.initRandomValues();
		return bs;
	}

}
