package pomdp.utilities;

import java.io.Serializable;
import java.util.Random;

public class RandomGenerator  implements Serializable{
	/*
	private static Random m_rndGenerator = null;
	
	private static void init(){
		if( m_rndGenerator == null ){
			//int iSeed = (int)System.nanoTime();
			int iSeed = (int)System.currentTimeMillis();
			//iSeed = 1;
			//iSeed = -1669075182;
			System.out.println( "Initializing generator with random seed " + iSeed );
			m_rndGenerator = new Random( iSeed );
		}
	}
	
	public static void init( int iSeed ){
		System.out.println( "Initializing generator with random seed " + iSeed );
		m_rndGenerator = new Random( iSeed );
	}
	
	public static int nextInt( int iMax ){
		init();
		return m_rndGenerator.nextInt( iMax );
	}
	
	public static int nextInt(){
		init();
		return m_rndGenerator.nextInt();
	}
	
	public static double nextDouble(){
		init();
		return m_rndGenerator.nextDouble();
	}

	public static double nextDouble( double dMax ){
		init();
		return m_rndGenerator.nextDouble() * dMax;
	}

	public static double nextDouble( double dLowerBound, double dUpperBound ){
		init();
		return m_rndGenerator.nextDouble() * ( dUpperBound - dLowerBound ) + dLowerBound;
	}
	*/
	private Random m_rndGenerator;
	private String m_sName;
	
	public RandomGenerator( String sName ){
		//this( sName, System.currentTimeMillis() );
		this( sName, System.nanoTime() );
	}
	
	public RandomGenerator( String sName, long iSeed ){
		m_rndGenerator = new Random( iSeed );
		m_sName = sName;
		System.out.println( "Initializing generator " + m_sName + " with random seed " + iSeed );
	}
		
	public void init( long iSeed, boolean bNotify ){
		m_rndGenerator = new Random( iSeed );
		if( bNotify )
			System.out.println( "Initializing generator " + m_sName + " with random seed " + iSeed );
	}
	
	public void init( long iSeed ){
		m_rndGenerator = new Random( iSeed );
		System.out.println( "Initializing generator " + m_sName + " with random seed " + iSeed );
	}
	
	public int nextInt( int iMax ){
		return m_rndGenerator.nextInt( iMax );
	}
	
	public int nextInt(){
		return m_rndGenerator.nextInt();
	}
	
	public double nextDouble(){
		return m_rndGenerator.nextDouble();
	}

	public double nextDouble( double dMax ){
		return m_rndGenerator.nextDouble() * dMax;
	}

	public double nextDouble( double dLowerBound, double dUpperBound ){
		return m_rndGenerator.nextDouble() * ( dUpperBound - dLowerBound ) + dLowerBound;
	}
	
	
	public static void main( String[] args ){
		int[] ac = new int[5];
		int i = 0, rnd = 0;
		
		for( i = 0 ; i < 5 ; i++ )
			ac[i] = 0;
		
		RandomGenerator rndGen = new RandomGenerator( "General", 1 );
		
		for( i = 0 ; i < 1000000 ; i++ ){
			rnd = Math.abs( rndGen.nextInt() ) % 5;
			ac[rnd]++;
		}
		
		for( i = 0 ; i < 5 ; i++ )
			System.out.println( i + "=" + ac[i] + ", "  );
		
		Random rnd0 = new Random( 0 );
		Random rnd1 = new Random( 1 );
		String sOutput0 = "Rnd( 0 ) = ";
		for( i = 0 ; i < 10 ; i++ )
			sOutput0 += rnd0.nextInt( 9 ) + ", " ;
		System.out.println( sOutput0 );
		String sOutput1 = "Rnd( 1 ) = ";
		for( i = 0 ; i < 10 ; i++ )
			sOutput1 += rnd1.nextInt( 9 ) + ", " ;
		System.out.println( sOutput1 );
		rnd0 = new Random( 0 );
		rnd1 = new Random( 1 );
		sOutput0 = "Rnd( 0 ) = ";
		sOutput1 = "Rnd( 1 ) = ";
		for( i = 0 ; i < 10 ; i++ ){
			sOutput0 += rnd0.nextInt( 9 ) + ", " ;
			sOutput1 += rnd1.nextInt( 9 ) + ", " ;
		}
		System.out.println( sOutput0 );
		System.out.println( sOutput1 );
	}

}
