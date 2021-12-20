package pomdp.utilities.skyline;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.TreeMap;
import java.util.Map.Entry;

import javax.crypto.spec.IvParameterSpec;

import pomdp.utilities.AlphaVector;
import pomdp.utilities.RandomGenerator;
import pomdp.utilities.datastructures.StaticMap;

public class Equation {
	private int m_iLHSVariable;
	private Map<Integer, Double> m_mRHSVariables;
	private double m_dShift;	
	private int m_iSlackVariable;
	private static double NOISE = 0.0000;
	public static int m_cEquations = 0;
	
	private static RandomGenerator m_rndGenerator = new RandomGenerator( "Equation", 0 );
	
	//Notation - 
	//x_0 is the value of the vector at the current point.
	//shift is the coef of s_0 which does not have a belief associated since b_0 = 1 - \sum_i>1 b_i
	//x_1..x_n-1 are the coefs for s_1..s_n-1
	//the slack variable is the identifier of the current alpha vector
	public Equation( AlphaVector avEquationVector, int cStates, int iSlackVariable, int iDeterministicState ){
		m_cEquations++;
		m_iSlackVariable = iSlackVariable;
		m_iLHSVariable = 0;
		m_mRHSVariables = new HashMap<Integer, Double>();
		/*
		Iterator<Entry<Integer, Double>> it = av.getNonZeroEntries();
		Entry<Integer, Double> e = null;
		m_dShift = 0.0;//-m_rndGenerator.nextDouble( 0.00001 ); 
		while( it.hasNext() ){
			e = it.next();
			if( e.getKey() == 0 ){
				m_dShift = e.getValue();
			}
			else{			
				m_mRHSVariables.put( e.getKey(), e.getValue() - m_dShift );
			}
		}
		*/
		m_dShift = avEquationVector.valueAt( iDeterministicState ) - m_rndGenerator.nextDouble( NOISE );
		for( int iState = 0 ; iState < cStates ; iState ++ ){
			if( iState != iDeterministicState ){
				double dValue = avEquationVector.valueAt( iState );
				m_mRHSVariables.put( iState, dValue - m_dShift - m_rndGenerator.nextDouble( NOISE ) );
			}
		}
		m_mRHSVariables.put( iSlackVariable, 1.0 );
	}
	
	public Equation( AlphaVector avEquationVector, AlphaVector avSearchVector, 
			int cStates, int iSlackVariable, int iDeterministicState, int iDeltaVariable, double dDeltaShift ){
		m_cEquations++;
		m_iSlackVariable = iSlackVariable;
		m_iLHSVariable = iDeltaVariable;
		m_mRHSVariables = new HashMap<Integer, Double>();
		double dCoefChange = avEquationVector.valueAt( iDeterministicState ) - avSearchVector.valueAt( iDeterministicState );
		m_dShift = dCoefChange + dDeltaShift + m_rndGenerator.nextDouble( NOISE );
		for( int iState = 0 ; iState < cStates ; iState ++ ){
			if( iState != iDeterministicState ){
				double dValue = avEquationVector.valueAt( iState ) - avSearchVector.valueAt( iState );
				m_mRHSVariables.put( iState, dValue - dCoefChange - m_rndGenerator.nextDouble( NOISE ) );
			}
		}
		m_mRHSVariables.put( iSlackVariable, 1.0 );
	}
	
	private Equation(){
		m_cEquations++;
		m_iLHSVariable = -1;
		m_mRHSVariables = new TreeMap<Integer, Double>();
		m_dShift = Double.NaN; 
		m_iSlackVariable = -1;
	}
	
	public static Equation getBeliefContraintEquation( int cStates, int iSlackVariable ){
		Equation eq = new Equation();
		int iState = 0;
		//Skipping x_0 which does not exist since it is a fake variable - see above
		for( iState = 0 ; iState < cStates ; iState++ ){
			if( iState != iSlackVariable )
				eq.put( iState, -1.0 );
		}
		eq.m_dShift = 1.0;
		eq.m_iSlackVariable = iSlackVariable;
		eq.m_iLHSVariable = iSlackVariable;
		
		return eq;
	}
	
	public Equation join( Equation eq ){
		if( ( eq == null ) || ( m_iLHSVariable != eq.m_iLHSVariable ) )
			return null;
		Equation eJoin = new Equation();		
		eJoin.m_iLHSVariable = m_iSlackVariable;
		eJoin.m_dShift = eq.m_dShift - m_dShift;
		for( Entry<Integer, Double> e : m_mRHSVariables.entrySet() ){
			if( e.getKey() != m_iSlackVariable ){
				Double dOther = eq.get( e.getKey() );
				if( dOther == null ){
					eJoin.put( e.getKey(), 0.0 - e.getValue() );
				}
				else{
					eJoin.put( e.getKey(), dOther - e.getValue() );
				}
			}
		}
		for( Entry<Integer, Double> e : eq.m_mRHSVariables.entrySet() ){
			Double dOther = m_mRHSVariables.get( e.getKey() );
			if( dOther == null ){
				eJoin.put( e.getKey(), e.getValue() );
			}
		}
		
		return eJoin;
	}
	
	public Equation substitue( Equation eq ){
		int iSubstituteVar = eq.m_iLHSVariable;
		if( !m_mRHSVariables.containsKey( iSubstituteVar ) )
			return this;
		double dCoef = m_mRHSVariables.get( iSubstituteVar );
		Equation eSubstitue = new Equation();
		eSubstitue.m_iLHSVariable = m_iLHSVariable;
		eSubstitue.m_dShift = m_dShift + dCoef * eq.m_dShift;
		if( eSubstitue.m_dShift < 0.0 )
			eSubstitue.m_dShift = 0.0;
		for( Entry<Integer, Double> e : eq.m_mRHSVariables.entrySet() ){
			Double dOther = m_mRHSVariables.get( e.getKey() );
			double dNewValue = dCoef * e.getValue();
			if( dOther != null ){
				dNewValue += dOther;
			}
			eSubstitue.put( e.getKey(), dNewValue );
		}
		for( Entry<Integer, Double> e : m_mRHSVariables.entrySet() ){
			if( e.getKey() != iSubstituteVar ){
				Double dOther = eSubstitue.get( e.getKey() );
				if( dOther == null ){
					eSubstitue.put( e.getKey(), e.getValue() );
				}
			}
		}
		
		return eSubstitue;
	}
	
	public double valueAtZero(){
		return m_dShift;
	}
	
	public Equation extractVariable( int iVariable ){
		double dCoef = -1.0 * m_mRHSVariables.get( iVariable );
		Equation eqExtracted = new Equation();
		eqExtracted.m_dShift = m_dShift / dCoef;
		for( Entry<Integer, Double> e : m_mRHSVariables.entrySet() ){
			if( e.getKey() != iVariable ){
				eqExtracted.put( e.getKey(), e.getValue() / dCoef );
			}
		}
		eqExtracted.put( m_iLHSVariable, -1.0 / dCoef );
		eqExtracted.m_iLHSVariable = iVariable;
		return eqExtracted;
	}
	
	private void put( int iVariable, double dValue ){
		m_mRHSVariables.put( iVariable, dValue );
	}
	private Double get( int iVariable ){
		return m_mRHSVariables.get( iVariable );
	}
	
	public int getSlackVariable(){
		return m_iSlackVariable;
	}
	
	public double computeDelta( int iVariable ){
		Double dCoef = m_mRHSVariables.get( iVariable );
		if( ( dCoef == null ) || ( dCoef >= 0 ) )
			return Double.POSITIVE_INFINITY;
		return -1.0 * m_dShift / dCoef;
	}

	public int getLHSVariable() {
		return m_iLHSVariable;
	}
	
	private double round( double x ){
		if( x == 0.0 )
			return 0.0;
		double dSign = 1.0;
		if( x < 0 ){
			dSign = -1.0;
			x *= -1.0;
		}
			
		int c = 0;
		while( x < 100 ){
			x *= 10;
			c++;
		}
		x = Math.round( x );
		while( c > 0 ){
			x /= 10;
			c--;
		}
		return x * dSign;
	}
	
	public String toString(){
		String sValue = String.format( "%.2f", m_dShift );
		String s = "X" + m_iLHSVariable + " = " + sValue;
		for( Entry<Integer, Double> e : m_mRHSVariables.entrySet() ){
			sValue = String.format( "%.2f", e.getValue() );
			s += " + " + sValue + "*X" + e.getKey();
		}
		return s;
	}

	public double getShift() {
		return m_dShift;
	}
}
