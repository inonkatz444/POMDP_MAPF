package pomdp.utilities;

import java.util.Iterator;
import java.util.Vector;

public class QuadraticValueFunctionApproximation {
	Vector m_vQuadraticFunctions;
	
	public QuadraticValueFunctionApproximation(){
		m_vQuadraticFunctions = new Vector();
	}
	
	public double valueAt( BeliefState bs ){
		QuadraticFunction qfMin = getMinQF( bs );
		if( qfMin == null )
			return Integer.MAX_VALUE;
		return qfMin.valueAt( bs );
	}
	
	public QuadraticFunction getMinQF( BeliefState bs ){
		Iterator itFunctions = m_vQuadraticFunctions.iterator();
		QuadraticFunction qfCurrent = null, qfMin = null;
		double dValue = 0.0, dMinValue = Double.MAX_VALUE;
		while( itFunctions.hasNext() ){
			qfCurrent = (QuadraticFunction)itFunctions.next();
			dValue = qfCurrent.valueAt( bs );
			if( dValue < dMinValue ){
				dMinValue = dValue;
				qfMin = qfCurrent;
			}
		}
		return qfMin;
	}
	
	public void add( QuadraticFunction qfNew ){
		m_vQuadraticFunctions.add( qfNew );
	}

	public int size(){
		return m_vQuadraticFunctions.size();
	}

	public int getBestAction( BeliefState bs ){
		QuadraticFunction qfMin = getMinQF( bs );
		if( qfMin == null )
			return -1;
		return qfMin.getAction();
	}

}
