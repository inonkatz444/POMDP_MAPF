package pomdp.valuefunction;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.Serializable;
import java.util.Collection;
import java.util.Iterator;
import java.util.Vector;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import pomdp.algorithms.PolicyStrategy;
import pomdp.environments.POMDP;
import pomdp.utilities.AlphaVector;
import pomdp.utilities.BeliefState;
import pomdp.utilities.BeliefStateFactory;
import pomdp.utilities.ExecutionProperties;
import pomdp.utilities.RandomGenerator;
import pomdp.utilities.concurrent.DotProduct;
import pomdp.utilities.concurrent.ThreadPool;
import pomdp.utilities.datastructures.LinkedList;
import pomdp.utilities.skyline.SkylinePruning;

//import lpsolve.*;

public class LinearValueFunctionApproximation implements Serializable{
	//protected Vector<AlphaVector> m_vAlphaVectors;
	//protected ArrayList<AlphaVector> m_vAlphaVectorsRead, m_vAlphaVectorsWrite;
	protected LinkedList<AlphaVector> m_vAlphaVectors;
	protected RandomGenerator m_rndGenerator;
	protected int m_cValueFunctionChanges;
	protected double m_dEpsilon;
	protected boolean m_bCacheValues;
	protected double m_dMaxValue;
	private static boolean g_bUseMultithreadInDotProducts = false;
	private boolean m_bEvaluatingPolicy;
	private boolean m_bPruned;
	
	public LinearValueFunctionApproximation( double dEpsilon, boolean bCacheValues ){
		m_vAlphaVectors = new LinkedList<AlphaVector>();
		m_cValueFunctionChanges = 0;
		m_dEpsilon = dEpsilon;
		m_bCacheValues = true;
		m_dMaxValue = 0.0;
		m_bEvaluatingPolicy = false;
		m_bPruned = false;
		m_rndGenerator = new RandomGenerator( "LinearValueFunctionApproximation" );
	}

	public LinearValueFunctionApproximation(){
		this( 1.0, true );
	}

	public LinearValueFunctionApproximation( LinearValueFunctionApproximation vOtherValueFunction ) {
		copy( vOtherValueFunction );
		m_bCacheValues = vOtherValueFunction.m_bCacheValues;
	}
	
	public void finalize(){
		System.out.println( "LinearValueFunctionApproximation finalized" );
	}
	

	public double valueAt( BeliefState bs ){
		if( m_vAlphaVectors.size() == 0 )
			return Double.NEGATIVE_INFINITY;
		double dValue = bs.getMaxValue();
		int iTime = bs.getMaxValueTime(), cValueFunctionChanges = m_cValueFunctionChanges;
		if( ( iTime < cValueFunctionChanges ) || !m_bCacheValues ){
			AlphaVector avMaxAlpha = getMaxAlpha( bs );
			if( avMaxAlpha == null )
				return Double.NEGATIVE_INFINITY;
			dValue = avMaxAlpha.dotProduct( bs );
			if( m_bCacheValues ){
				bs.setMaxValue( dValue, cValueFunctionChanges );
			}
		}
		
		return dValue;
	}

	public AlphaVector getMaxAlpha( BeliefState bs ){
		int cElements = m_vAlphaVectors.size();
		if( cElements == 0 )
			return null;
		AlphaVector avMaxAlpha = bs.getMaxAlpha();
		double dMaxValue = bs.getMaxValue(), dValue = 0.0;
		int iBeliefStateLastCheckTime = bs.getMaxAlphaTime();
		int iCurrentTime = m_cValueFunctionChanges;
		
		if( !m_vAlphaVectors.contains( avMaxAlpha ) ){
			avMaxAlpha = null;
			dMaxValue = Double.NEGATIVE_INFINITY;
			iBeliefStateLastCheckTime = -1;
		}

		if( g_bUseMultithreadInDotProducts  && ExecutionProperties.useMultiThread() ){
			DotProduct[] m_dpTasks = new DotProduct[m_vAlphaVectors.size()];
			int i = 0;
			for( AlphaVector avCurrent : m_vAlphaVectors ){
				if( !m_bCacheValues || avCurrent.getInsertionTime() > iBeliefStateLastCheckTime ){
					m_dpTasks[i] = new DotProduct( avCurrent, bs );
					ThreadPool.getInstance().addTask( m_dpTasks[i] );
					i++;
				}
			}
			while( i > 0 ){
				i--;
				ThreadPool.getInstance().waitForTask( m_dpTasks[i] );
				dValue = m_dpTasks[i].getResult();
				if( dValue > dMaxValue ){
					dMaxValue = dValue;
					avMaxAlpha = m_dpTasks[i].getAlphaVector();
				}
			}
		}
		
		int iInsertionTime = Integer.MAX_VALUE;
		Iterator<AlphaVector> itBackward = m_vAlphaVectors.backwardIterator();
		boolean bDone = false;
		while( itBackward.hasNext() && !bDone ){
			AlphaVector avCurrent = itBackward.next();
			if( avCurrent != null ){
				iInsertionTime = avCurrent.getInsertionTime();
				if( m_bCacheValues && ( iBeliefStateLastCheckTime >= iInsertionTime ) )
					bDone = true;
				/*
				if( avCurrent.getId() == 1334 ){
					System.out.println( bs );
					System.out.println( avCurrent );
				}
				*/
				dValue = avCurrent.dotProduct( bs );
				if( ( dValue > dMaxValue ) || ( ( dValue == dMaxValue ) && ( avMaxAlpha != null ) && ( iInsertionTime > avMaxAlpha.getInsertionTime() )  ) ){
					dMaxValue = dValue;
					avMaxAlpha = avCurrent;
				}
			}
		}
		
		if( avMaxAlpha != null ){
			if( m_bCacheValues ){
				bs.setMaxAlpha( avMaxAlpha, iCurrentTime );
				bs.setMaxValue( dMaxValue, iCurrentTime );
			}
			
			avMaxAlpha.incrementHitCount();
		}
		return avMaxAlpha;
	}

	public int getBestAction( BeliefState bs ){
		AlphaVector avMaxAlpha = getMaxAlpha( bs );
		if( avMaxAlpha == null )
			return -1;
		return avMaxAlpha.getAction();
	}

	public Iterator<AlphaVector> iterator(){
		return m_vAlphaVectors.iterator();
	}

	public AlphaVector elementAt(int iElement ){
		return (AlphaVector) m_vAlphaVectors.get( iElement );
	}
	
	public void startEvaluation(){
		m_bEvaluatingPolicy = true;
	}
	public void endEvaluation(){
		m_bEvaluatingPolicy = false;
	}
	
	public boolean addPrunePointwiseDominated( AlphaVector avNew ){
		BeliefState bsWitness = null;
		double dNewValue = 0.0;
		
		while( m_bEvaluatingPolicy ){
			try {
				wait( 100 );
			} catch (Exception e) {
			}
		}

		Iterator<AlphaVector> it = m_vAlphaVectors.iterator();
		AlphaVector avExisting = null;
		while( it.hasNext() ){
			avExisting = it.next();
			if( avExisting.equals( avNew ) || avExisting.dominates( avNew ) ){
				return false;
			}
			else if( avNew.dominates( avExisting ) ){
				it.remove();
			}
		}		
		
		m_bPruned = false;
		
		m_cValueFunctionChanges++;
		addVector( avNew );
		if( m_bCacheValues ){		
			avNew.setInsertionTime( m_cValueFunctionChanges );
			bsWitness = avNew.getWitness();
			if( bsWitness != null ){
				dNewValue = avNew.dotProduct( bsWitness );
				bsWitness.setMaxAlpha( avNew, m_cValueFunctionChanges );
				bsWitness.setMaxValue( dNewValue, m_cValueFunctionChanges );
			}
		}
		
		if( avNew.getMaxValue() > m_dMaxValue )
			m_dMaxValue = avNew.getMaxValue();
		
		return true;
	}
	
	private void addVector( AlphaVector avNew ){
		m_vAlphaVectors.add( avNew );
	}
		
	public void initHitCounts(){
		try{
			for( AlphaVector av : m_vAlphaVectors ){
				av.initHitCount();
			}
		}
		catch( Exception e )
		{
			System.err.println( e + " retrying" );
			initHitCounts();
		}
	}
	
	public void pruneLowHitCountVectors( int cMinimalHitCount ){
		pruneLowHitCountVectors( cMinimalHitCount, Integer.MAX_VALUE );
	}
	
	public void pruneLowHitCountVectors( int cMinimalHitCount, int iMaximalTimeStamp ){
		while( m_bEvaluatingPolicy ){
			try {
				wait( 100 );
			} catch (Exception e) {
			}
		}
		int cPruned = 0, cNew = 0;
		LinkedList<AlphaVector> vAlphaVectorsWrite = new LinkedList<AlphaVector>();
		for( AlphaVector av : m_vAlphaVectors ){
			if( av.getInsertionTime() > iMaximalTimeStamp || av.getHitCount() > cMinimalHitCount ){
				vAlphaVectorsWrite.add( av );
			}
			if( av.getInsertionTime() > iMaximalTimeStamp ){
				cNew++;
			}
			if( av.getHitCount() <= cMinimalHitCount ){
				cPruned++;
			}
		}
		if( vAlphaVectorsWrite.size() > 0 ){
			System.out.println( "Pruned from " + m_vAlphaVectors.size() + " to " + vAlphaVectorsWrite.size() + ". pruned " + cPruned + ", new vectors " + cNew );
			m_bPruned = true;
			m_vAlphaVectors = vAlphaVectorsWrite;
		}
	}
	
	public boolean wasPruned(){
		return m_bPruned;
	}
	
	public void addBounded( AlphaVector avNew, int cMaxVectors ){
		
		addPrunePointwiseDominated( avNew );
		
		if( m_vAlphaVectors.size() > cMaxVectors ){
			int i = m_rndGenerator.nextInt( m_vAlphaVectors.size() );
			m_vAlphaVectors.remove( i );
		}
	}
	
	public void add( AlphaVector avNew, boolean bPruneDominated ){
		AlphaVector avExisting = null;
		BeliefState bsWitness = null;
		boolean bDominated = false;
		double dPreviousValue = 0.0, dNewValue = 0.0;
		
		m_cValueFunctionChanges++;
		if( bPruneDominated ){
			int iVector = 0;
			for( iVector = 0 ; iVector < m_vAlphaVectors.size() && !bDominated ; iVector++ ){
				avExisting = m_vAlphaVectors.get( iVector );
				if( avNew.dominates( avExisting ) ){
					m_vAlphaVectors.remove( avExisting );
				}
				else if( avExisting.dominates( avNew ) ){
					bDominated = true;
				}
			}
		}
		
		if( !bDominated ){
			m_vAlphaVectors.add( avNew );
		
			if( m_bCacheValues ){		
				avNew.setInsertionTime( m_cValueFunctionChanges );
				bsWitness = avNew.getWitness();
				if( bsWitness != null ){
					dNewValue = avNew.dotProduct( bsWitness );
					bsWitness.setMaxAlpha( avNew, m_cValueFunctionChanges );
					bsWitness.setMaxValue( dNewValue, m_cValueFunctionChanges );
				}
			}
	
			if( avNew.getMaxValue() > m_dMaxValue )
				m_dMaxValue = avNew.getMaxValue();
		}
		
	}


	public void clear() {
		for( AlphaVector av : m_vAlphaVectors ){
			av.release();
		}
		m_vAlphaVectors.clear();
		m_cValueFunctionChanges = 0;
		
	}

	public void addAll( LinearValueFunctionApproximation vOtherValueFunction ){
		m_vAlphaVectors.addAll( vOtherValueFunction.m_vAlphaVectors );
	}

	public int size(){
		return m_vAlphaVectors.count();
	}
	
	public boolean equals( LinearValueFunctionApproximation vOther ){
		return vOther.m_vAlphaVectors.containsAll( m_vAlphaVectors ) && 
			m_vAlphaVectors.containsAll( vOther.m_vAlphaVectors );
	}

	public void copy( LinearValueFunctionApproximation vOtherValueFunction ){
		while( m_bEvaluatingPolicy ){
			try {
				wait( 100 );
			} catch (Exception e) {
			}
		}
		m_vAlphaVectors = new LinkedList<AlphaVector>( vOtherValueFunction.m_vAlphaVectors );
		//m_cValueFunctionChanges = vOtherValueFunction.m_cValueFunctionChanges;
		m_dEpsilon = vOtherValueFunction.m_dEpsilon;
		m_rndGenerator = vOtherValueFunction.m_rndGenerator;
		m_cValueFunctionChanges = vOtherValueFunction.m_cValueFunctionChanges;
		m_dEpsilon = vOtherValueFunction.m_dEpsilon;
		m_bCacheValues = vOtherValueFunction.m_bCacheValues;
		m_dMaxValue = vOtherValueFunction.m_dMaxValue;
		m_bEvaluatingPolicy = vOtherValueFunction.m_bEvaluatingPolicy;
	}

	public void add( AlphaVector avNew ){
		add( avNew, false );
	}

	public void remove( AlphaVector av ){
		m_vAlphaVectors.remove( av );
	}

	public double approximateValueAt( BeliefState bs ){
		if( m_vAlphaVectors.size() == 0 )
			return -1 * Double.MAX_VALUE;
		int iBeliefStateMaxAlphaTime = bs.getApproximateValueTime();
		double dMaxValue = bs.getApproximateValue(), dValue = 0.0;
		AlphaVector avCurrent = null;
		
		if( iBeliefStateMaxAlphaTime < m_cValueFunctionChanges ){
			int iVector = 0;
			for( iVector = 0 ; iVector < m_vAlphaVectors.size() ; iVector++ ){
				avCurrent = m_vAlphaVectors.get( iVector );
				if( avCurrent.getInsertionTime() > iBeliefStateMaxAlphaTime ){
					dValue = avCurrent.approximateDotProduct( bs );
					if( dValue > dMaxValue ){
						dMaxValue = dValue;
					}
				}
			}
			bs.setApproximateValue( dMaxValue, m_cValueFunctionChanges );
		}
		
		return dMaxValue;
	}

	public int getChangesCount() {
		return m_cValueFunctionChanges;
	}
	
	public void setCaching( boolean bCache ){
		m_bCacheValues = bCache;
	}

	public boolean contains( AlphaVector av ){
		return m_vAlphaVectors.contains( av );
	}
	
	public String toString(){
		String sRetVal = "<";
		
		for( AlphaVector av : m_vAlphaVectors ){
			sRetVal += av.toString() + "\n";
		}
		
		return sRetVal;
	}
	
	public double getMaxValue(){
		return m_dMaxValue;
	}

	public AlphaVector getFirst() {
		return m_vAlphaVectors.getFirst();
	}

	public AlphaVector getLast() {
		return m_vAlphaVectors.getLast();
	}
	
	public Element getDOM( Document doc ) throws Exception{
		Element eValueFunction = doc.createElement( "ValueFunction" ), eAlphaVector = null;
		AlphaVector avCurrent = null;
		
		eValueFunction = doc.createElement( "ValueFunction" );
		eValueFunction.setAttribute( "AlphaVectorCount", m_vAlphaVectors.size() + "" );
		eValueFunction.setAttribute( "Epsilon", m_dEpsilon + "" );
		eValueFunction.setAttribute( "CacheValue", m_bCacheValues + "" );
		eValueFunction.setAttribute( "MaxValue", m_dMaxValue + "" );		
		doc.appendChild( eValueFunction );
		
		int iVector = 0;
		for( iVector = 0 ; iVector < m_vAlphaVectors.size() ; iVector++ ){
			avCurrent = m_vAlphaVectors.get( iVector );
			eAlphaVector = avCurrent.getDOM( doc );
			eValueFunction.appendChild( eAlphaVector );
		}
		
		return eValueFunction;
	}
	
	public void save( String sFileName ) throws Exception{
		Document docValueFunction = DocumentBuilderFactory.newInstance().newDocumentBuilder().newDocument();
		Element eValueFunction = getDOM( docValueFunction );
		
		// Use a Transformer for output
		TransformerFactory tFactory = TransformerFactory.newInstance();
		Transformer transformer = tFactory.newTransformer();
		
		DOMSource source = new DOMSource( eValueFunction );
		StreamResult result = new StreamResult( new FileOutputStream( sFileName ) );
		transformer.transform( source, result );
	}
	
	public void parseDOM( Element eValueFunction, POMDP pomdp ) throws Exception{
		Element eVector = null;
		NodeList nlVectors = null;
		int cVectors = 0, iVector = 0;
		AlphaVector avNew = null;
		
		cVectors = Integer.parseInt( eValueFunction.getAttribute( "AlphaVectorCount" ) );
		nlVectors = eValueFunction.getChildNodes();
		
		m_dEpsilon = Double.parseDouble( eValueFunction.getAttribute( "Epsilon" ) );
		m_bCacheValues = Boolean.parseBoolean( eValueFunction.getAttribute( "CacheValue" ) );
		m_dMaxValue = Double.parseDouble( eValueFunction.getAttribute( "MaxValue" ) );

		for( iVector = 0 ; iVector < cVectors ; iVector++ ){
			eVector = (Element)nlVectors.item( iVector );
			avNew = AlphaVector.parseDOM( eVector, pomdp );
			m_vAlphaVectors.add( avNew );
		}
	}
	
	public void load( String sFileName, POMDP pomdp ) throws Exception{
		DocumentBuilder builder = DocumentBuilderFactory.newInstance().newDocumentBuilder();
		Document docValueFunction = builder.parse( new FileInputStream( sFileName ) );
		Element eValueFunction = (Element)docValueFunction.getChildNodes().item( 0 );
		
		parseDOM( eValueFunction, pomdp );
	}

	
	public void removeFirst() {
		m_vAlphaVectors.remove( 0 );
	}

	public Collection<AlphaVector> getVectors() {
		return m_vAlphaVectors;
	}

	public void setVectors( Vector<AlphaVector> v ) {
		m_vAlphaVectors = new LinkedList<AlphaVector>( v );		
	}

	public int countEntries() {
		AlphaVector avCurrent = null;
		int cEntries = 0;
		
		int iVector = 0;
		for( iVector = 0 ; iVector < m_vAlphaVectors.size() ; iVector++ ){
			avCurrent = m_vAlphaVectors.get( iVector );
			cEntries += avCurrent.countEntries();
		}
		return cEntries;
	}

	public double getAvgAlphaVectorSize() {
		double cNodes = 0;
		for( AlphaVector av : m_vAlphaVectors )
			cNodes += av.countEntries();
		return cNodes / m_vAlphaVectors.size();
	}
	
	public void skylinePruning( BeliefState bsStart, int cStates ){
		BeliefState bs = bsStart.copy();
		AlphaVector av0 = null, av1 = null;
		double dMax = Double.NEGATIVE_INFINITY;
		double dValue = 0.0;
		for( AlphaVector avCurrent : m_vAlphaVectors ){
			dValue = avCurrent.dotProduct( bs );
			if( dValue > dMax ){
				dMax = dValue;
				av0 = avCurrent;
			}
		}
		
		int iState = av0.getNonZeroEntries().next().getKey();
		double dT = 0.0, dMinT = Double.POSITIVE_INFINITY;
		double dStateValue = av0.valueAt( iState );
		for( AlphaVector avCurrent : m_vAlphaVectors ){
			if( avCurrent != av0 ){
				dT = ( avCurrent.dotProduct( bs ) - dMax ) / ( dStateValue - avCurrent.valueAt( iState ) );
				if( dT < dMinT ){
					dMinT = dT;
					av1 = avCurrent;
				}
			}
		}
		
		
		
		
	}

	static{
		try{
			/*
			String sLibraryPath = System.getProperty("java.library.path");
			System.setProperty("java.library.path", sLibraryPath + ";" + "C:\\Windows\\System32\\lpsolve");
			sLibraryPath = System.getProperty("java.library.path");
			System.out.println( sLibraryPath );
			*/
			System.loadLibrary( "lpsolve55j" );
			System.out.println( "lpsolve loaded" );
		}
		catch( UnsatisfiedLinkError err ){
			System.out.println( "unable to load dll " + err );
		}
	}

	private long m_cLPIterations = 0;
	
	private double[] dominateLP( AlphaVector avCurrent, LinkedList<AlphaVector> vAlphaVectors, int cStates ){
		/*
		try {
			// Create a problem with 4 variables and 0 constraints
			//cStates++;
			LpSolve solver = LpSolve.makeLp( 0, cStates + 1 );
			double[] adConstraint = new double[cStates + 1]; //states + delta
			int iState = 0;
			String sConstraint = "";
			
			//constraints for finding a witness for avCurrent
			adConstraint[cStates] = -1;//-delta
			for( AlphaVector av : vAlphaVectors ){
				if( ( av != avCurrent ) && ( !av.isDominated() ) ){
					sConstraint = "";
					for( iState = 0 ; iState < cStates ; iState++ ){
						adConstraint[iState] = avCurrent.valueAt( iState ) - av.valueAt( iState );
						sConstraint += adConstraint[iState] + " ";
					}
					//x*a >= delta + x*a'  ==> x*a - x*a' - delta >= 0
					sConstraint += "-1";
					//solver.addConstraint( adConstraint, LpSolve.GE, 0.0 );
					solver.strAddConstraint( sConstraint, LpSolve.GE, 0.0 );
				}
			}
			
			//add belief constraints
			adConstraint[cStates] = 0;//delta
			sConstraint = "";
			for( iState = 0 ; iState < cStates ; iState++ ){
				adConstraint[iState] = 1;
				sConstraint += "1 ";
			}
			sConstraint += "0";
			//solver.addConstraint( adConstraint, LpSolve.EQ, 1.0 );
			solver.strAddConstraint( sConstraint, LpSolve.EQ, 1.0 );
			
			//set objective function - maximize delta
			adConstraint[cStates] = 1;//delta
			sConstraint = "";
			for( iState = 0 ; iState < cStates ; iState++ ){
				adConstraint[iState] = 0;
				sConstraint += "0 ";
			}
			sConstraint += "1";
			//solver.setObjFn( adConstraint );
			solver.strSetObjFn( sConstraint );
			solver.setMaxim();
			
			//solver.setLowbo( cStates + 1, -1 * solver.getInfinite() ); //we are interested only in delta greater than 0.0
			//solver.setLowbo( cStates, 0 ); //we are interested only in delta greater than 0.0
			
			for( iState = 1 ; iState <= cStates ; iState++ ){
				solver.setUpbo( iState, 1.0 );
				solver.setLowbo( iState, 0.0 );
			}			
			
			
			// solve the problem
			solver.setOutputfile( "" );
			//solver.printLp();
			int iResult = solver.solve();
			double[] adSolution = null;
			double dMaxDelta = solver.getObjective();
			m_cLPIterations += solver.getTotalIter();
			if( dMaxDelta > 0.0001 ){
				adSolution = solver.getPtrVariables();
			}
			
			// delete the problem and free memory
			solver.deleteLp();
			return adSolution;
		}
		catch (LpSolveException e) {
			e.printStackTrace();
			System.exit( 0 );
		}
		*/
		return null;
	}

	public void pruneLP( POMDP pPOMDP ) {
		if( m_vAlphaVectors.size() < 2 )
			return;
		LinkedList<AlphaVector> vDirtyList = new LinkedList<AlphaVector>( m_vAlphaVectors );
		LinkedList<AlphaVector> vCleanList = new LinkedList<AlphaVector>();
		SkylinePruning sp = new SkylinePruning( this, pPOMDP.getStateCount() );
		
		int iState = 0;
		BeliefState bsCurrent = null;
		double[] adBelief = null;
		AlphaVector avCurrent = null, avMax = null;
		double dValue = 0.0, dMax = 0.0;
		m_cLPIterations = 0;
/*
		for( iState = 0 ; iState < pPOMDP.getStateCount() ; iState++ ){
			bsCurrent = pPOMDP.getBeliefStateFactory().getDeterministicBeliefState( iState );
			avMax = getMaxAlpha( bsCurrent );
			if( !vCleanList.contains( avMax ) ){
				avMax.setWitness( bsCurrent );
				vDirtyList.remove( avMax );
				vCleanList.add( avMax );
			}
		}
		
		while( vDirtyList.size() > 0 ){
			avCurrent = vDirtyList.getFirst();
			//adBelief = dominateLP( avCurrent, vCleanList, pPOMDP.getStateCount() );
			adBelief = sp.findWitness( vCleanList, avCurrent );
			m_cLPIterations = sp.getProcessedCount();
				
			if( adBelief != null ){
				
				dMax = Double.NEGATIVE_INFINITY;
				for( AlphaVector av : vDirtyList ){
					dValue = av.dotProduct( adBelief );
					if( dValue > dMax ){
						dMax = dValue;
						avMax = av;
					}						
				}
				avMax.setWitness( pPOMDP.getBeliefStateFactory().newBeliefState( adBelief ) );
				vDirtyList.remove( avMax );
				vCleanList.add( avMax );
			}
			else{
				avCurrent.setWitness( null );
				vDirtyList.remove( avCurrent );
			}
		}
*/
		if( true ){
			for( AlphaVector av : vDirtyList ){
				av.setDominated( false );
			}
			for( AlphaVector av : vDirtyList ){
				//adBelief = dominateLP( av, vDirtyList, pPOMDP.getStateCount() );
				adBelief = sp.findWitness( vDirtyList, av );
				m_cLPIterations = sp.getProcessedCount();
				if( adBelief == null ){
					//System.out.println( av );
					av.setDominated( true );
				}
				else{
					/*
					BeliefState bs = pPOMDP.getBeliefStateFactory().newBeliefState( adBelief );
					System.out.println( av + " * " + bs + " = " + av.dotProduct( bs ) );
					for( AlphaVector avTag : vDirtyList ){
						if( avTag != av ){
							System.out.println( avTag + " * " + bs + " = " + avTag.dotProduct( bs ) );
						}
					}
					*/
					vCleanList.add( av );
				}
			}
		}
		System.out.println( "LP: Pruned the lower bound from " + m_vAlphaVectors.size() + " to " + vCleanList.size() + ", iterations = " + m_cLPIterations );
		m_vAlphaVectors = vCleanList;
	}
	
	public void pruneRandomSampling( BeliefStateFactory bsf, int cSamples ) {
		if( m_vAlphaVectors.size() < 2 )
			return;
		LinkedList<AlphaVector> vDirtyList = new LinkedList<AlphaVector>( m_vAlphaVectors );
		LinkedList<AlphaVector> vCleanList = new LinkedList<AlphaVector>();
		int iSample = 0;
		BeliefState bsCurrent = null;
		AlphaVector avMax = null;
		double dValue = 0.0, dMax = 0.0, dSum = 0.0;
	
		for( iSample = 0 ; ( iSample < cSamples ) && ( vDirtyList.size() > 0.0 ) ; iSample++ ){
			dSum = 0.0;
			bsCurrent = bsf.getRandomBeliefState();
			
			dMax = Double.NEGATIVE_INFINITY;
			
			for( AlphaVector av : vDirtyList ){
				dValue = av.dotProduct( bsCurrent );
				if( dValue > dMax ){
					dMax = dValue;
					avMax = av;
				}						
			}
			for( AlphaVector av : vCleanList ){
				dValue = av.dotProduct( bsCurrent );
				if( dValue > dMax ){
					avMax = null;
					break;
				}						
			}
			
			if( avMax != null ){
				vCleanList.add( avMax );
				vDirtyList.remove( avMax );
			}
		}
		System.out.println( "Pruned the lower bound from " + m_vAlphaVectors.size() + " to " + vCleanList.size() );
		m_vAlphaVectors = vCleanList;
	}
	public void pruneRandomSampling( POMDP pPOMDP, int cSamples ) {
		if( m_vAlphaVectors.size() < 2 )
			return;
		LinkedList<AlphaVector> vDirtyList = new LinkedList<AlphaVector>( m_vAlphaVectors );
		LinkedList<AlphaVector> vCleanList = new LinkedList<AlphaVector>();
		int iState = 0, iSample = 0, cStates = pPOMDP.getStateCount();
		BeliefState bsCurrent = null;
		double[] adBelief = new double[cStates];
		AlphaVector avMax = null;
		double dValue = 0.0, dMax = 0.0, dSum = 0.0;
		for( iState = 0 ; iState < cStates ; iState++ ){
			bsCurrent = pPOMDP.getBeliefStateFactory().getDeterministicBeliefState( iState );
			avMax = getMaxAlpha( bsCurrent );
			if( !vCleanList.contains( avMax ) ){
				vCleanList.add( avMax );
				vDirtyList.remove( avMax );
			}
		}
	
		for( iSample = 0 ; ( iSample < cSamples ) && ( vDirtyList.size() > 0.0 ) ; iSample++ ){
			dSum = 0.0;
			for( iState = 0 ; iState < cStates ; iState++ ){
				adBelief[iState] = m_rndGenerator.nextDouble();
				dSum += adBelief[iState];
			}
			for( iState = 0 ; iState < cStates ; iState++ ){
				adBelief[iState] /= dSum;
			}
			
			for( AlphaVector av : vDirtyList ){
				dValue = av.dotProduct( adBelief );
				if( dValue > dMax ){
					dMax = dValue;
					avMax = av;
				}						
			}
			for( AlphaVector av : vCleanList ){
				dValue = av.dotProduct( adBelief );
				if( dValue > dMax ){
					avMax = null;
					break;
				}						
			}
			
			if( avMax != null ){
				vCleanList.add( avMax );
				vDirtyList.remove( avMax );
			}
		}
		System.out.println( "Pruned the lower bound from " + m_vAlphaVectors.size() + " to " + vCleanList.size() );
		m_vAlphaVectors = vCleanList;
	}
	public void pruneTrials( POMDP pPOMDP, int cTrials, int cSteps, PolicyStrategy ps ){
		initHitCounts();
		double dSimulatedADR = pPOMDP.computeAverageDiscountedReward( cTrials, cSteps, ps );
		int cBefore = m_vAlphaVectors.size();
		pruneLowHitCountVectors( 0 );
		System.out.println( "Pruned the lower bound from " + cBefore + " to " + m_vAlphaVectors.size() );
	}
	public boolean pruneSkyline( POMDP pPOMDP ){
		int cBefore = m_vAlphaVectors.size(), cPruned = 0;
		LinkedList<AlphaVector> vCleanList = new LinkedList<AlphaVector>();
		SkylinePruning sp = new SkylinePruning( this, pPOMDP.getStateCount() );
		for( AlphaVector av : m_vAlphaVectors ){
			av.clearWitnesses();
			av.setDominated( true );
		}
		sp.runSkylineWitness();
		for( AlphaVector av : m_vAlphaVectors ){
			//if( !av.isDominated() && av.countWitnesses() > 1 )
			if( !av.isDominated() )
				vCleanList.add( av );
			else
				cPruned++;
		}
		m_vAlphaVectors = vCleanList;
		System.out.println( "Skyline: Pruned the lower bound from " + cBefore + " to " + m_vAlphaVectors.size()
				+ ", iterations = " + sp.getProcessedCount() );
		return cBefore > m_vAlphaVectors.size();
	}
}
