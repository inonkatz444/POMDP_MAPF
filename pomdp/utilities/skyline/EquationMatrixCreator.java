package pomdp.utilities.skyline;

import java.util.Comparator;

public class EquationMatrixCreator {
	private EquationMatrix m_emMatrix;
	private int m_iOutVariableIdx;
	private int m_cZeroBeliefStates;
	
	public EquationMatrixCreator( EquationMatrix em, int iOutVariable, int cZeroBeliefStates ){
		m_iOutVariableIdx = iOutVariable;
		m_emMatrix = em;
		m_cZeroBeliefStates = cZeroBeliefStates;
	}
	
	public EquationMatrix getMatrix(){
		return m_emMatrix.getNeighbor( m_iOutVariableIdx );
	}
	
	public int getZeroBeliefStates(){
		return m_cZeroBeliefStates;
	}
}
