package pomdp.algorithms.pointbased;

import java.util.Map;
import java.util.TreeMap;
import java.util.Map.Entry;

import pomdp.algorithms.ValueIteration;
import pomdp.environments.POMDP;
import pomdp.utilities.AlphaVector;
import pomdp.utilities.BeliefState;
import pomdp.utilities.Logger;

public class StateBasedValueIteration extends ValueIteration{

	public StateBasedValueIteration( POMDP pomdp ) {
		super(pomdp);
		m_vfMDP.valueIteration( 100, 0.01 );
	}

	@Override
	public void valueIteration( int cIterations, double dEpsilon, double dTargetValue ) {
		int iIteration = 0, iState = 0;
		Map<Double,Integer> mStates = new TreeMap<Double, Integer>();
		BeliefState bsCurrent = null, bsInitial = m_pPOMDP.getBeliefStateFactory().getInitialBeliefState();
		AlphaVector av = null;
		double dNewValue = 0.0, dPreviousValue = 0.0, dMaxDelta = Double.POSITIVE_INFINITY;
		int iAllRocks = ( m_cStates - 1 ) >> 8;
		for( iState = 0 ; iState < m_cStates ; iState++ ){
			int iRockState = iState >> 8;
			if( iRockState == iAllRocks )
				mStates.put( 1000.0 - m_vfMDP.getValue( iState ) + m_rndGenerator.nextDouble() * 0.001, iState );
		}
		
		for( iIteration = 0 ; iIteration < cIterations && dMaxDelta > dEpsilon ; iIteration++ ){
			dMaxDelta = 0.0;
			for( Entry<Double, Integer> e : mStates.entrySet() ){
				bsCurrent = m_pPOMDP.getBeliefStateFactory().getRandomBeliefState( e.getValue() );
				//bsCurrent = m_pPOMDP.getBeliefStateFactory().getDeterministicBeliefState( e.getValue() );
				av = backup( bsCurrent );
				dPreviousValue = m_vValueFunction.valueAt( bsCurrent );
				dNewValue = av.dotProduct( bsCurrent );
				if( dNewValue - dPreviousValue > 0.01 ){
					if( dNewValue - dPreviousValue > dMaxDelta )
						dMaxDelta = dNewValue - dPreviousValue;
					m_vValueFunction.add( av );
				}
			}
			Logger.getInstance().log( "StateBasedValueIteration", 0, "valueIteration", "Iteration " + iIteration + 
					" |V|=" + m_vValueFunction.size() + " V(b0)=" + m_vValueFunction.valueAt( bsInitial ) + " max delta=" + dMaxDelta );
		}
	}

}
