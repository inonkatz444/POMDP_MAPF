package pomdp;

import pomdp.algorithms.PolicyStrategy;
import pomdp.environments.POMDP;
import pomdp.utilities.BeliefState;
import pomdp.utilities.RandomGenerator;
import pomdp.valuefunction.LinearValueFunctionApproximation;

import java.util.List;

/**
 * @author shanigu
 *
 * To change the template for this generated type comment go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */

public class RandomWalkPolicy extends PolicyStrategy {
	private POMDP pomdp;
	protected RandomGenerator m_rndGenerator;
	
	public RandomWalkPolicy( POMDP pomdp ){
		this.pomdp = pomdp;
		m_rndGenerator = RandomGenerator.getInstance();
	}
	 
	/* (non-Javadoc)
	 * @see PolicyStrategy#getAction(BeliefState)
	 */
	public int getAction( BeliefState bsCurrent ){
		List<Integer> iActions = pomdp.getRelevantActions(bsCurrent);
		return iActions.get(m_rndGenerator.nextInt(iActions.size()));
	}

	public double getValue(BeliefState bsCurrent) {
		// TODO Auto-generated method stub
		return 0;
	}

	public boolean hasConverged() {
		// TODO Auto-generated method stub
		return false;
	}

	public String getStatus() {
		return "N/A";
	}

	public LinearValueFunctionApproximation getValueFunction() {
		// TODO Auto-generated method stub
		return null;
	}

}
