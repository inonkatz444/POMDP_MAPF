package pomdp.valuefunction;

import java.util.Collection;
import java.util.Iterator;
import java.util.Map;
import java.util.TreeMap;
import java.util.Vector;
import java.util.Map.Entry;

import pomdp.algorithms.pointbased.HeuristicSearchValueIteration.ValueFunctionEntry;
import pomdp.environments.POMDP;
import pomdp.utilities.BeliefState;
import pomdp.utilities.BeliefStateComparator;
import pomdp.utilities.BeliefStateFactory;
import pomdp.utilities.Logger;
import pomdp.utilities.MDPValueFunction;

public class UpperBoundValueFunctionApproximation {
	protected POMDP m_pPOMDP;
	protected Map<BeliefState, Double> m_mBeliefStateValues;
	protected double[] m_adStateValues;
	
	public UpperBoundValueFunctionApproximation( POMDP pomdp, double[] adStateValues ){
		m_pPOMDP = pomdp;
		m_mBeliefStateValues = new TreeMap<BeliefState, Double>( BeliefStateComparator.getInstance() );
		m_adStateValues = adStateValues.clone();
	}
	public UpperBoundValueFunctionApproximation( POMDP pomdp,
			MDPValueFunction vfMDP ) {
		m_pPOMDP = pomdp;
		m_mBeliefStateValues = new TreeMap<BeliefState, Double>( BeliefStateComparator.getInstance() );
		m_adStateValues = new double[m_pPOMDP.getStateCount()];
		int iState = 0;
		for( iState = 0 ; iState < m_pPOMDP.getStateCount() ; iState++ )
			m_adStateValues[iState] = vfMDP.getValue( iState );		
	}
	
	public double interpolate( BeliefState bs ){
		double dValue = 0.0;
		Entry<Integer,Double> e = null;
		Iterator<Entry<Integer,Double>> itNonZero = bs.getNonZeroEntries().iterator();
		while( itNonZero.hasNext() ){
			e = itNonZero.next();
			dValue += e.getValue() * m_adStateValues[e.getKey()];
		}
		return dValue;
	}
	
	public double valueAt( BeliefState bs ){
		if( !m_mBeliefStateValues.containsKey( bs ) ){
			return interpolate( bs );
		}
		return m_mBeliefStateValues.get( bs );
	}
	
	public int getAction( BeliefState bs ){
		int iAction = 0, iObservation = 0, iMaxAction = -1;
		double dMaxActionValue = Double.NEGATIVE_INFINITY, dPr = 0.0, dValueSum = 0.0;
		BeliefState bsSuccessor = null;
		
		for( iAction = 0 ; iAction < m_pPOMDP.getActionCount() ; iAction++ ){
			dValueSum = 0.0;
			for( iObservation = 0 ;iObservation < m_pPOMDP.getObservationCount() ; iObservation++ ){
				bsSuccessor = bs.nextBeliefState( iAction, iObservation );
				dPr = bs.probabilityOGivenA( iAction, iObservation );
				if( dPr > 0.0 ){
					dValueSum += dPr * valueAt( bsSuccessor );
				}
			}
			if( dValueSum > dMaxActionValue ){
				iMaxAction = iAction;
				dMaxActionValue = dValueSum;
			}
		}
		return iMaxAction;
	}
	
	public void updateValue(BeliefState bs) {
		int iAction = 0, iObservation = 0;
		BeliefState bsSuccessor = null;
		double dActionValue = 0.0, dMaxValue = 0.0, dPr = 0.0, dValue = 0.0;
		for( iAction = 0 ; iAction < m_pPOMDP.getActionCount() ; iAction++ ){
			dActionValue = m_pPOMDP.immediateReward( bs, iAction );
			for( iObservation = 0 ; iObservation < m_pPOMDP.getObservationCount() ; iObservation++ ){
				dPr = bs.probabilityOGivenA( iAction, iObservation );
				if( dPr > 0.0 ){
					bsSuccessor = bs.nextBeliefState( iAction, iObservation );
					dValue = valueAt( bsSuccessor );
					dActionValue += m_pPOMDP.getDiscountFactor() * dValue * dPr;
				}
			}
			if( dActionValue > dMaxValue )
				dMaxValue = dActionValue;
		}
		m_mBeliefStateValues.put( bs, dMaxValue );
	}
	public int getUpperBoundPointCount(){
		return m_mBeliefStateValues.size();
	}
	protected Collection<Entry<BeliefState,Double>> getUpperBoundPoints(){
		return m_mBeliefStateValues.entrySet();
	}
	public void pruneUpperBound(){
		BeliefState bs = null;
		Collection<Entry<BeliefState,Double>> colUpperBound = getUpperBoundPoints();
		double dUpperBound = 0.0, dHValue = 0.0, dNewValue = 0.0;
		int cBeliefPoints = getUpperBoundPointCount();
		double dEpsilon = (1e-10);
		Vector<BeliefState> vToRemove = new Vector<BeliefState>();
		
		for( Entry<BeliefState,Double> e : colUpperBound ){
			dUpperBound = e.getValue();
			
			bs = e.getKey();
			if( !bs.isDeterministic() ){				
				dHValue = interpolate( bs );
				if( dHValue <  dUpperBound - dEpsilon ){
					vToRemove.add( bs );
				}
				
			}
		}

		for( BeliefState bsRemove : vToRemove ){
			m_mBeliefStateValues.remove( bsRemove );
		}
		
		Logger.getInstance().log( "UpperBoundValueIteration", 0, "pruneUpperBound", "Pruned from " + cBeliefPoints + " to " + getUpperBoundPointCount() );
	}
}
