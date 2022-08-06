package pomdp;

import pomdp.algorithms.PolicyStrategy;
import pomdp.environments.POMDP;
import pomdp.utilities.BeliefState;
import pomdp.valuefunction.LinearValueFunctionApproximation;

public class NullPolicy extends PolicyStrategy {
    private POMDP pomdp;

    public NullPolicy(POMDP pomdp) {
        this.pomdp = pomdp;
    }

    @Override
    public int getAction(BeliefState bsCurrent) {
        return pomdp.getDoneAction();
    }

    @Override
    public double getValue(BeliefState bsCurrent) {
        return 0;
    }

    @Override
    public boolean hasConverged() {
        return false;
    }

    @Override
    public String getStatus() {
        return null;
    }

    @Override
    public LinearValueFunctionApproximation getValueFunction() {
        return null;
    }
}
