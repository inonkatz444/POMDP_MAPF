package pomdp.utilities.skyline;

import java.util.Comparator;


public class EquationMatrixCreatorComparator implements Comparator<EquationMatrixCreator>{

	private static EquationMatrixCreatorComparator m_mComparator = new EquationMatrixCreatorComparator();
	public static EquationMatrixCreatorComparator getInstance(){
		return m_mComparator;
	}

	@Override
	public int compare( EquationMatrixCreator o1, EquationMatrixCreator o2 ) {
		//o1 is better than o2 if it has less zero belief states
		return o2.getZeroBeliefStates() - o1.getZeroBeliefStates();
	}
	
}