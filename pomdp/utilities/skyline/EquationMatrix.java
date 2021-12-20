package pomdp.utilities.skyline;

import java.util.Collection;
import java.util.Map;
import java.util.TreeMap;
import java.util.Vector;
import java.util.Map.Entry;

import pomdp.utilities.AlphaVector;
import pomdp.valuefunction.LinearValueFunctionApproximation;

public class EquationMatrix {
	private Equation[] m_aEquations;
	private Map<Integer, AlphaVector> m_mSlackToAlphaVector;
	private int[] m_aRHSVariables, m_aLHSVariables;
	private int m_cStates;
	private int m_iDepth;
	//private EquationMatrix m_emParent;
	private int m_iParentInVariable, m_iParentOutVariable;
	public static int m_cMatrixes = 0, m_cFinalized = 0;
	private int m_iId;
	private boolean m_bInteriorPoint;
	private boolean m_bSkyline;
	private Vector<double[]> m_vObservedBeliefs;
	
	public void finalize(){
		m_cFinalized++;
	}
	
	private EquationMatrix( Equation[] aEquations, Map<Integer, AlphaVector> mSlackToAlphaVector, int cStates, int[] aRHSVariables, int[] aLHSVariables, int iDepth, boolean bSkyline ){
		m_aEquations = aEquations;
		m_mSlackToAlphaVector = mSlackToAlphaVector;
		m_cStates = cStates;
		m_aLHSVariables = aLHSVariables;
		m_aRHSVariables = aRHSVariables;
		m_iId = m_cMatrixes++;
		m_bInteriorPoint = true;
		m_bSkyline = bSkyline;
		
		//if( m_iId == 10802 )
		//	System.out.println( "*" );
		
		for( int iVariable : aRHSVariables ){
			if( iVariable < m_cStates ){
				m_bInteriorPoint = false;
				break;
			}	
		}
		m_iDepth = iDepth;
	}
	
	public EquationMatrix( Collection<AlphaVector> vValueFunction, int cStates, int iDeterministicState ){
		m_bSkyline = true;
		int cVectors = vValueFunction.size();
		Equation[] aInitial = new Equation[cVectors];
		m_cStates = cStates;
		m_aRHSVariables = new int[cStates];
		m_aLHSVariables = new int[cVectors];
		m_mSlackToAlphaVector = new TreeMap<Integer, AlphaVector>();
		int iVector = 0, iState = 0;
		double dMaxValue = Double.NEGATIVE_INFINITY;
		int iMaxValueEquation = 0;
		Equation eJoined = null;
		for( AlphaVector av : vValueFunction ){
			//x_0 = a_0 + a_1*x_1 + ... + a_n-1*x_n-1
			aInitial[iVector] = new Equation( av, cStates, cStates + iVector, iDeterministicState );
			
			//looking for the maximal alpha at <1,0,...,0>
			if( aInitial[iVector].valueAtZero() > dMaxValue ){
				dMaxValue = aInitial[iVector].valueAtZero();
				iMaxValueEquation = iVector;
			}
			m_mSlackToAlphaVector.put( cStates + iVector, av );
			iVector++;
		}
		//maximal alpha at <1,0,...,0> - cannot be dominated
		m_mSlackToAlphaVector.get( aInitial[iMaxValueEquation].getSlackVariable() ).setDominated( false );	
		m_aEquations = new Equation[cVectors];
		for( iVector = 0 ; iVector < cVectors ; iVector++ ){
			if( iVector != iMaxValueEquation ){
				//All equations are equal to x_0 at this point
				//we can therefore combine them by doing rhs_i = rhs_min then we leave only the slack variable at the LHS
				eJoined = aInitial[iVector].join( aInitial[iMaxValueEquation] );
				if( iVector < iMaxValueEquation ){
					m_aEquations[iVector] = eJoined;
					m_aLHSVariables[iVector] = cStates + iVector;
				}
				else{
					m_aEquations[iVector - 1] = eJoined;
					m_aLHSVariables[iVector - 1] = cStates + iVector;
				}
			}
		}
		//now we add the equation for the constraint x_0 + x_1 + x_2 + ... + x_n-1 <= 1
		m_aEquations[cVectors - 1] = Equation.getBeliefContraintEquation( cStates, iDeterministicState );
		m_aLHSVariables[cVectors - 1] = iDeterministicState;
		
		for( iState = 0 ; iState < cStates ; iState++ ){
			if( iState < iDeterministicState )
				m_aRHSVariables[iState] = iState;
			if( iState > iDeterministicState )
				m_aRHSVariables[iState - 1] = iState;
		}
		m_aRHSVariables[cStates - 1] = cStates + iMaxValueEquation;
		m_iId = m_cMatrixes++;
		m_bInteriorPoint = false;
		m_iDepth = 0;
	}
	
	public EquationMatrix( Collection<AlphaVector> vValueFunction, int cStates, int iDeterministicState, AlphaVector avSearchVector, double dDeltaShift ){
		m_bSkyline = false;
		int cVectors = vValueFunction.size(); //all except the search vector
		Equation[] aInitial = new Equation[cVectors];
		m_cStates = cStates;
		m_aRHSVariables = new int[cStates];
		m_aLHSVariables = new int[cVectors];
		m_mSlackToAlphaVector = new TreeMap<Integer, AlphaVector>();
		int iVector = 0, iState = 0, iMaxVectorVariable = -1;
		double dMaxValue = Double.NEGATIVE_INFINITY;
		int iMaxValueEquation = 0;
		Equation eJoined = null;
		for( AlphaVector av : vValueFunction ){
			//x_0 = a_0 + a_1*x_1 + ... + a_n-1*x_n-1
			if( av != avSearchVector ){
				aInitial[iVector] = new Equation( av, avSearchVector, cStates, cStates + iVector, iDeterministicState, cStates + cVectors, dDeltaShift );		
				//looking for the maximal alpha at <1,0,...,0>
				if( aInitial[iVector].getShift() > dMaxValue ){
					dMaxValue = aInitial[iVector].getShift();
					iMaxValueEquation = iVector;
					iMaxVectorVariable = cStates + iVector;
				}
				m_mSlackToAlphaVector.put( cStates + iVector, av );
				
				iVector++;
			}
		}
		m_mSlackToAlphaVector.put( cStates + cVectors, avSearchVector );
		
		//maximal alpha at <1,0,...,0> - cannot be dominated
		m_aEquations = new Equation[cVectors];
		for( iVector = 0 ; iVector < cVectors - 1 ; iVector++ ){
			if( iVector != iMaxValueEquation ){
				//All equations are equal to x_0 at this point
				//we can therefore combine them by doing rhs_i = rhs_min then we leave only the slack variable at the LHS
				eJoined = aInitial[iVector].join( aInitial[iMaxValueEquation] );
				m_aEquations[iVector] = eJoined;
				m_aLHSVariables[iVector] = cStates + iVector;
			}
			else{
				m_aEquations[iVector] = aInitial[iMaxValueEquation];
				m_aLHSVariables[iVector] = cStates + cVectors;
			}
		}
		//now we add the equation for the constraint x_0 + x_1 + x_2 + ... + x_n-1 <= 1
		m_aEquations[cVectors - 1] = Equation.getBeliefContraintEquation( cStates, iDeterministicState );
		m_aLHSVariables[cVectors - 1] = iDeterministicState;
		
		for( iState = 0 ; iState < cStates ; iState++ ){
			if( iState < iDeterministicState )
				m_aRHSVariables[iState] = iState;
			if( iState > iDeterministicState )
				m_aRHSVariables[iState - 1] = iState;
		}
		m_aRHSVariables[cStates - 1] = cStates + iMaxValueEquation;
		m_iId = m_cMatrixes++;
		m_bInteriorPoint = false;
		m_iDepth = 0;
	}
	
	private boolean exists( int[] aiRHSVariables, Vector<int[]> vObservedNodes ){
		for( int[] aiObserved : vObservedNodes ){
			if( equals( aiObserved, aiRHSVariables ) )
				return true;
		}
		return false;
	}
	
	private boolean equals( int[] aiObserved, int[] aiRHSVariables ){
		int idx = 0;
		if( aiObserved.length != aiRHSVariables.length )
			return false;
		for( idx = 0 ; idx < aiObserved.length ; idx++ ){
			if( aiObserved[idx] != aiRHSVariables[idx] )
				return false;
		}
		return true;
	}

	private boolean exists( double[] aiRHSVariables, Vector<double[]> vObservedNodes ){
		for( double[] aiObserved : vObservedNodes ){
			if( equals( aiObserved, aiRHSVariables ) )
				return true;
		}
		return false;
	}
	
	private boolean equals( double[] aiObserved, double[] aiRHSVariables ){
		int idx = 0;
		for( idx = 0 ; idx < aiObserved.length ; idx++ ){
			if( Math.abs( aiObserved[idx] - aiRHSVariables[idx] ) > 0.0001 )
				return false;
		}
		return true;
	}

	public int isValidEdge( int iVariableIdx, Vector<int[]> vObservedNodes, boolean bForceTowardsInterior ){
		int iOutVariable = m_aRHSVariables[iVariableIdx], iInVariable = -1;
		int[] aiRHSVariables = new int[m_cStates];
		int iEquation = 0, iMinEquation = -1, iVariable = 0, iIdx = 0;
		double dMinDelta = Double.POSITIVE_INFINITY, dDelta = 0.0;
		
		if( !m_bInteriorPoint && bForceTowardsInterior ){
			if( iOutVariable >= m_cStates )
				return -1;
		}
		
		for( iEquation = 0 ; iEquation < m_aEquations.length ; iEquation++ ){
			dDelta = m_aEquations[iEquation].computeDelta( iOutVariable );
			if( dDelta < dMinDelta ){
				dMinDelta = dDelta;
				iMinEquation = iEquation;
			}
		}
		if( iMinEquation == -1 )
			return -1;
		iInVariable = m_aEquations[iMinEquation].getLHSVariable();
		/*
		if( iInVariable >= m_cStates ){
			AlphaVector av = m_mSlackToAlphaVector.get( iInVariable );
			if( av.isDominated() ){
				av.setDominated( false );
			}
			else{
				return -1;
			}
		}
			*/
		Vector<Integer> vRHSVariables = new Vector<Integer>();
		int iRHSVariable = -1;
		for( iVariable = 0 ; iVariable < m_cStates ; iVariable++ ){
			if( m_aRHSVariables[iVariable] == iOutVariable ){
				iRHSVariable = iInVariable;
				//iIdx = iVariable;
				iIdx = vRHSVariables.size();
			}
			else{
				iRHSVariable = m_aRHSVariables[iVariable];
			}
			aiRHSVariables[iVariable] = iRHSVariable;
			if( iRHSVariable >= m_cStates ){
				vRHSVariables.add( iRHSVariable );
			}
		}
		aiRHSVariables = new int[vRHSVariables.size()];
		for( iVariable = 0 ; iVariable < aiRHSVariables.length ; iVariable++ ){
			aiRHSVariables[iVariable] = vRHSVariables.elementAt( iVariable );
		}
		resort( aiRHSVariables, iIdx );
		if( exists( aiRHSVariables, vObservedNodes ) )
			return -1;
		int cZeroBeliefVariables = 0;
		for( int iVar : aiRHSVariables ){
			if( iVar > 0 && iVar < m_cStates )
				cZeroBeliefVariables++;
		}
		//if( cZeroBeliefVariables == 0 )
		//	System.out.println( "*" );
		vObservedNodes.add( aiRHSVariables );
		return cZeroBeliefVariables;
	}
	
	public EquationMatrix getNeighbor( int iVariableIdx ){
		int iOutVariable = m_aRHSVariables[iVariableIdx], iInVariable = -1;
		int[] aiRHSVariables = new int[m_cStates], aiLHSVariable = new int[m_aEquations.length];
		int iEquation = 0, iMinEquation = -1, iVariable = 0, iIdx = 0;
		double dMinDelta = 1000.0, dDelta = 0.0;

		//if( !m_bInteriorPoint && ( iOutVariable >= m_cStates ) )
		//	return null;
		
		for( iEquation = 0 ; iEquation < m_aEquations.length ; iEquation++ ){
			dDelta = m_aEquations[iEquation].computeDelta( iOutVariable );
			if( dDelta < dMinDelta ){
				dMinDelta = dDelta;
				iMinEquation = iEquation;
			}
		}
		if( iMinEquation == -1 )
			return null;
		iInVariable = m_aEquations[iMinEquation].getLHSVariable();
		
		//if( m_bInteriorPoint && ( iInVariable < m_cStates ) )
		//	return null;
		
		//if( ( iInVariable >= m_cStates ) && ( iOutVariable >= m_cStates ) )
		//	return null;
		
		for( iVariable = 0 ; iVariable < m_cStates ; iVariable++ ){
			if( m_aRHSVariables[iVariable] == iOutVariable ){
				aiRHSVariables[iVariable] = iInVariable;
				iIdx = iVariable;
			}
			else{
				aiRHSVariables[iVariable] = m_aRHSVariables[iVariable];
			}
		}
		resort( aiRHSVariables, iIdx );

		Equation eqSubstitue = m_aEquations[iMinEquation].extractVariable( iOutVariable );
		Equation[] aSuccessorEquations = new Equation[m_aEquations.length];
		for( iEquation = 0 ; iEquation < m_aEquations.length ; iEquation++ ){
			if( iEquation == iMinEquation ){
				iIdx = iEquation;
				aSuccessorEquations[iEquation] = eqSubstitue;
			}
			else{				
				aSuccessorEquations[iEquation] = m_aEquations[iEquation].substitue( eqSubstitue );
			}
			if( m_aLHSVariables[iEquation] == iInVariable ){
				aiLHSVariable[iEquation] = iOutVariable;				
			}
			else{
				aiLHSVariable[iEquation] = m_aLHSVariables[iEquation];
			}
		}
		resort( aiLHSVariable, iIdx );
		EquationMatrix eqNew = new EquationMatrix( aSuccessorEquations, m_mSlackToAlphaVector, m_cStates, aiRHSVariables, aiLHSVariable, m_iDepth + 1, m_bSkyline );
		eqNew.m_iParentInVariable = iInVariable;
		eqNew.m_iParentOutVariable = iOutVariable;
				
		return eqNew;
	}
	
	private EquationMatrix getNeighbor( int iVariableIdx, Vector<int[]> vObservedNodes, Vector<double[]> vObservedBeliefs ){
		int iOutVariable = m_aRHSVariables[iVariableIdx], iInVariable = -1;
		int[] aiRHSVariables = new int[m_cStates], aiLHSVariable = new int[m_aEquations.length];
		int iEquation = 0, iMinEquation = -1, iVariable = 0, iIdx = 0;
		double dMinDelta = Double.POSITIVE_INFINITY, dDelta = 0.0;

		//if( !m_bInteriorPoint && ( iOutVariable >= m_cStates ) )
		//	return null;
		
		for( iEquation = 0 ; iEquation < m_aEquations.length ; iEquation++ ){
			dDelta = m_aEquations[iEquation].computeDelta( iOutVariable );
			if( dDelta < dMinDelta ){
				dMinDelta = dDelta;
				iMinEquation = iEquation;
			}
		}
		if( iMinEquation == -1 )
			return null;
		iInVariable = m_aEquations[iMinEquation].getLHSVariable();
		
		if( m_bInteriorPoint && ( iInVariable < m_cStates ) )
			return null;
		
		//if( iInVariable < m_cStates )// && iOutVariable < m_cStates )
		//	return null;
		
		for( iVariable = 0 ; iVariable < m_cStates ; iVariable++ ){
			if( m_aRHSVariables[iVariable] == iOutVariable ){
				aiRHSVariables[iVariable] = iInVariable;
				iIdx = iVariable;
			}
			else{
				aiRHSVariables[iVariable] = m_aRHSVariables[iVariable];
			}
		}
		resort( aiRHSVariables, iIdx );
		if( ( vObservedNodes != null ) && exists( aiRHSVariables, vObservedNodes ) )
			return null;
		if( m_bSkyline ){
			AlphaVector av = m_mSlackToAlphaVector.get( iInVariable );
			if( av != null ){
				av.setDominated( false );
			}
		}
		if( vObservedNodes != null )
			vObservedNodes.add( aiRHSVariables );
		//if( m_cMatrixes == 167 )
		//	System.out.println("*" );
		Equation eqSubstitue = m_aEquations[iMinEquation].extractVariable( iOutVariable );
		Equation[] aSuccessorEquations = new Equation[m_aEquations.length];
		for( iEquation = 0 ; iEquation < m_aEquations.length ; iEquation++ ){
			if( iEquation == iMinEquation ){
				iIdx = iEquation;
				aSuccessorEquations[iEquation] = eqSubstitue;
			}
			else{				
				aSuccessorEquations[iEquation] = m_aEquations[iEquation].substitue( eqSubstitue );
			}
			if( m_aLHSVariables[iEquation] == iInVariable ){
				aiLHSVariable[iEquation] = iOutVariable;				
			}
			else{
				aiLHSVariable[iEquation] = m_aLHSVariables[iEquation];
			}
		}
		resort( aiLHSVariable, iIdx );
		EquationMatrix eqNew = new EquationMatrix( aSuccessorEquations, m_mSlackToAlphaVector, m_cStates, aiRHSVariables, aiLHSVariable, m_iDepth + 1, m_bSkyline );
		eqNew.m_iParentInVariable = iInVariable;
		eqNew.m_iParentOutVariable = iOutVariable;
		if( vObservedBeliefs != null ){
			if( !exists( eqNew.getBeliefState(), vObservedBeliefs ) )
				vObservedBeliefs.add( eqNew.getBeliefState() );
		}
		return eqNew;
	}

	private double getNeighborValueOf( int iOutVariableIdx, int iAlphaVectorVariable ){
		int iOutVariable = m_aRHSVariables[iOutVariableIdx], iInVariable = -1;
		int iEquation = 0, iMinEquation = -1;
		double dMinDelta = Double.POSITIVE_INFINITY, dDelta = 0.0;
		
		for( iEquation = 0 ; iEquation < m_aEquations.length ; iEquation++ ){
			dDelta = m_aEquations[iEquation].computeDelta( iOutVariable );
			if( dDelta < dMinDelta ){
				dMinDelta = dDelta;
				iMinEquation = iEquation;
			}
		}
		if( iMinEquation == -1 )
			return Double.POSITIVE_INFINITY;
		
		iInVariable = m_aEquations[iMinEquation].getLHSVariable();
		if( iInVariable == iAlphaVectorVariable )
			return 0.0;
				
		if( m_bSkyline ){
			AlphaVector av = m_mSlackToAlphaVector.get( iInVariable );
			if( av != null ){
				av.setDominated( false );
			}
		}
		Equation eqSubstitue = m_aEquations[iMinEquation].extractVariable( iOutVariable );
		Equation eqSuccessor = null;
		for( iEquation = 0 ; iEquation < m_aEquations.length ; iEquation++ ){
			if( iEquation != iMinEquation ){
				if( m_aEquations[iEquation].getLHSVariable() == iAlphaVectorVariable ){
					eqSuccessor = m_aEquations[iEquation].substitue( eqSubstitue );
				}
			}
		}
		if( eqSuccessor == null ){
			//this means that we can now make the value of the target variable zero, and therefore it is miminized
			return 0.0;
		}
		
		return eqSuccessor.getShift();
	}

	
	private void resort( int[] a, int idx ){
		if( idx >= a.length )
			return;
		while( ( idx < a.length - 1 ) && ( a[idx + 1] < a[idx] ) ){
			swap( a, idx, idx + 1 );
			idx++;
		}
		while( ( idx > 0 ) && ( a[idx - 1] > a[idx] ) ){
			swap( a, idx, idx - 1 );
			idx--;
		}	
	}

	private void swap( int[] a, int i, int j) {
		int aux = a[i];
		a[i] = a[j];
		a[j] = aux;
		
	}

	public Collection<EquationMatrix> neighbors( Vector<int[]> vObservedNodes, Vector<double[]> vObservedBeliefs ){
		int iVariable = 0;
		EquationMatrix emNeighbor = null;
		Vector<EquationMatrix> vNeighbors = new Vector<EquationMatrix>();
		for( iVariable = 0 ; iVariable < m_cStates ; iVariable++ ){
			emNeighbor = getNeighbor( iVariable, vObservedNodes, vObservedBeliefs );
			if( emNeighbor != null ){
				vNeighbors.add( emNeighbor );
			}
		}
		return vNeighbors;
	}
	
	public Collection<EquationMatrixCreator> lazyNeighbors( Vector<int[]> vObservedNodes, Vector<double[]> vObservedBeliefs ){
		int iVariable = 0, iZeroBeliefVariables = 0;
		Vector<EquationMatrixCreator> vNeighbors = new Vector<EquationMatrixCreator>();
		
		m_vObservedBeliefs = vObservedBeliefs;
		
		for( iVariable = 0 ; iVariable < m_cStates ; iVariable++ ){
			iZeroBeliefVariables = isValidEdge( iVariable, vObservedNodes, false );
			if( iZeroBeliefVariables > -1 )
				vNeighbors.add( new EquationMatrixCreator( this, iVariable, iZeroBeliefVariables ) );
		}
		/*
		if( ( vNeighbors.size() == 0 ) && !m_bInteriorPoint ){
			for( iVariable = 0 ; iVariable < m_cStates && vNeighbors.size() == 0 ; iVariable++ ){
				if( isValidEdge( iVariable, vObservedNodes, false ) ){
					vNeighbors.add( new EquationMatrixCreator( this, iVariable ) );
				}
			}
		}
		if( vNeighbors.size() == 0 ){
			for( iVariable = 0 ; iVariable < m_cStates && vNeighbors.size() == 0 ; iVariable++ ){
				if( isValidEdge( iVariable, vObservedNodes, false ) ){
					vNeighbors.add( new EquationMatrixCreator( this, iVariable ) );
				}
			}
		}
		*/
		return vNeighbors;
	}
	
	public AlphaVector getAlphaVector( int iVariable ){
		return m_mSlackToAlphaVector.get( iVariable );
	}

	public int[] getRHSVariables() {
		return m_aRHSVariables;
	}
	
	public String toString(){
		String s = "Id = " + m_iId + "\nRHS: ";
		for( int iVar : m_aRHSVariables )
			s += iVar + ",";
		s += "\n";
		s += "LHS: ";
		for( int iVar : m_aLHSVariables )
			s += iVar + ",";
		s += "\n";
		for( Equation eq : m_aEquations )
			s += eq + "\n";
		return s;
	}
	
	public boolean isDeterministicBelief(){
		double dSum = 0.0;
		for( Equation eq : m_aEquations ){
			if( eq.getLHSVariable() < m_cStates ){
				if( eq.getShift() == 1.0 )
					return true;
				dSum += eq.getShift();
			}
		}
		return dSum == 0.0;
	}
	
	public double[] getBeliefState(){
		double[] adBelief = new double[m_cStates];
		double dSum = 0.0;
		for( Equation eq : m_aEquations ){
			if( eq.getLHSVariable() < m_cStates ){
				adBelief[eq.getLHSVariable()] += eq.getShift();
				dSum += eq.getShift();
			}
		}
		//adBelief[0] = 1.0 - dSum;
		return adBelief;
	}
	public String getBeliefStateString(){
		double[] adBelief = new double[m_cStates];
		double dSum = 0.0;
		for( Equation eq : m_aEquations ){
			if( eq.getLHSVariable() < m_cStates ){
				adBelief[eq.getLHSVariable()] += eq.getShift();
				dSum += eq.getShift();
			}
		}
		adBelief[0] = 1.0 - dSum;
		String sBelief = "[";
		for( int iState = 0 ; iState < m_cStates ; iState++ ){
			if( adBelief[iState] > 0.0 )
				sBelief += iState + "=" + adBelief[iState] + ", ";
		}
		sBelief += "]";
		return sBelief;
	}
	public boolean isInteriorPoint(){
		//return m_bInteriorPoint;
		/*
		for( int iVariable : m_aRHSVariables ){
			if( iVariable < m_cStates )
				return false;
		}
		return true;
		*/
		double[] adBelief = getBeliefState();
		for( double d : adBelief )
			if( d < 0.0000001 )
				return false;
		return true;
		
	}

	public int[] getIntersectingFunctions() {
		Vector<Integer> vRHSFunctions = new Vector<Integer>();
		for( int iVar : m_aRHSVariables ){
			if( iVar >= m_cStates ){
				vRHSFunctions.add( iVar );
			}
		}
		int[] aiIntersecting = new int[vRHSFunctions.size()];
		for( int iVar = 0 ; iVar < aiIntersecting.length ; iVar++ ){
			aiIntersecting[iVar] = vRHSFunctions.elementAt( iVar );
		}
		return aiIntersecting;
	}

	public int getVariable (AlphaVector av ) {
		for( Entry<Integer, AlphaVector> e : m_mSlackToAlphaVector.entrySet() ){
			if( e.getValue() == av ){
				return e.getKey();
			}
		}
		return -1;
	}

	//Returns the next node, trying to minimize the distance of the vector associated with iVectorVariable from the skyline at the related belief point
	public EquationMatrix nextVertex( int iVectorVariable ) {
		int iVariable = 0, iMinVariable = -1;
		double dCurrentValue = valueOf( iVectorVariable );
		double dValue = 0.0, dMinValue = dCurrentValue;
		for( iVariable = 0 ; iVariable < m_cStates ; iVariable++ ){
			dValue = getNeighborValueOf( iVariable, iVectorVariable );
			if( dValue < dMinValue ){
				dMinValue = dValue;
				iMinVariable = iVariable;
			}
		}
		if( dMinValue >= dCurrentValue ){
			return null;
		}
		return getNeighbor( iMinVariable );
	}

	public double valueOf( int iVariable ){
		for( Equation eq : m_aEquations ){
			if( eq.getLHSVariable() == iVariable )
				return eq.getShift();
		}
		return 0.0;
	}
}
