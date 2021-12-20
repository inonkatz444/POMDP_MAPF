package pomdp.utilities;

public class QuadraticFunction {
	protected double[] m_adAs;
	protected double[] m_adBs;
	protected double m_dC;
	protected int m_cStates;
	protected int m_iAction;
	protected int m_iID;
	public static int m_cQuadraticFunctions = 0;
	protected BeliefState m_bsWitness;
	
	protected QuadraticFunction( int cStates, double[] adAs, double dA, BeliefState bs, double dValueAtBs, int iAction ){
		m_iID = m_cQuadraticFunctions++;
		m_cStates = cStates;
		m_adAs = new double[m_cStates];
		m_adBs = new double[m_cStates];
		int iState = 0;
		for( iState = 0 ; iState < m_cStates ; iState++ ){
			if( adAs != null )
				m_adAs[iState] = adAs[iState];
			else
				m_adAs[iState] = dA;
			m_adBs[iState] = bs.valueAt( iState );
			m_dC = dValueAtBs;
		}
		m_iAction = iAction;
		m_bsWitness = bs;
	}
	
	public QuadraticFunction( int cStates, double[] adAs, BeliefState bs, double dValueAtBs, int iAction ){
		this( cStates, adAs, 0, bs, dValueAtBs, iAction );
	}
	
	public QuadraticFunction( int cStates, double dA, BeliefState bs, double dValueAtBs, int iAction ){
		this( cStates, null, dA, bs, dValueAtBs, iAction );
	}

	protected double square( double dX ){
		return dX * dX;
	}
	
	public double valueAt( BeliefState bs ){
		int iState = 0;
		double dSum = 0.0;
		for( iState = 0 ; iState < m_cStates ; iState++ ){
			dSum += m_adAs[iState] * square( bs.valueAt( iState ) - m_adBs[iState] );
		}
		dSum += m_dC;
		
		//if( dSum < 0.0 )
		//	System.out.println( getAllData() );
		return dSum;
	}
	
	public int getId(){
		return m_iID;
	}

	public int getAction() {
		return m_iAction;
	}
	
	public String toString(){
		return "QF" + m_iID;
	}
	
	private double round( double d, int cDigits ) {
		double dPower = Math.pow( 10, cDigits );
		double d1 = Math.round( d1 = d * dPower );
		return d1 / dPower;
	}
	
	
	public String getAllData(){
		String sResult = "QF" + getId() + " = " + round( m_dC, 3 );
		int iState = 0;
		for( iState = 0 ; iState < m_cStates ; iState++ )
			sResult += " + " + round( m_adAs[iState], 3 ) + "(" + round( m_adBs[iState], 3 ) + " - b" + iState + ")";
		return sResult;
	}
	
	public BeliefState getWitness(){
		return m_bsWitness;
	}

	public double getMinValue() {
		return m_dC;
	}
	

}
