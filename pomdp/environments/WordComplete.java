package pomdp.environments;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.Collection;
import java.util.Iterator;
import java.util.Map;
import java.util.SortedSet;
import java.util.TreeMap;
import java.util.Vector;
import java.util.Map.Entry;

import pomdp.algorithms.PolicyStrategy;
import pomdp.algorithms.pointbased.ForwardSearchValueIteration;
import pomdp.algorithms.pointbased.HeuristicSearchValueIteration;
import pomdp.environments.POMDP.IntegerCollection;
import pomdp.environments.POMDP.RewardType;
import pomdp.utilities.AlphaVector;
import pomdp.utilities.BeliefState;
import pomdp.utilities.BeliefStateFactory;
import pomdp.utilities.InvalidModelFileFormatException;
import pomdp.utilities.LineReader;
import pomdp.utilities.MDPValueFunction;
import pomdp.utilities.WordCompleteBeliefStateFactory;
import pomdp.utilities.WordCompleteHeuristicPolicy;

public class WordComplete extends POMDP {
	private Vector<Double> m_vPopularity;
	private Vector<String> m_vItems;
	private Vector<Integer> m_vStates;
	private int[][] m_aiStateIndexes;
	private int m_cItems, m_cMaxItemLength;
	
	public WordComplete(){
		super();
		m_cMaxItemLength = 0;
		m_cItems = 0;
		m_vItems = new Vector<String>();
		m_vPopularity = new Vector<Double>();
	}
	public void load( String sFileName ) throws IOException{
		LineReader lrInput = new LineReader( sFileName );
		String sLine = "";
		String sItemName = "";
		int iItem = -1;
		double dPopularity = 0.0, dMaxPopularity = 0.0, dMinPopularity = Double.POSITIVE_INFINITY;
		double dSumPopularity = 0.0;
		while( !lrInput.endOfFile() ){
			sLine = lrInput.readLine().trim();
			if( sLine.length() > 0 ){
				iItem = Integer.parseInt( sLine.substring( 0, sLine.indexOf( '\t' ) ) );
				sLine = sLine.substring( sLine.indexOf( '\t' ) + 1 );
				sItemName = sLine.substring( 0, sLine.indexOf( '\t' ) );
				sLine = sLine.substring( sLine.indexOf( '\t' ) + 1 );
				dPopularity = Double.parseDouble( sLine );
				dSumPopularity += dPopularity;
				m_vItems.add( sItemName );
				m_vPopularity.add( dPopularity );
				if( sItemName.length() >= m_cMaxItemLength )
					m_cMaxItemLength = sItemName.length() + 1;
				m_cItems++;
			}
			if( m_cItems == 200 )
				break;
		}
		for( iItem = 0 ; iItem < m_cItems ; iItem++ ){
			dPopularity = m_vPopularity.elementAt( iItem ) / dSumPopularity;
			m_vPopularity.set( iItem, dPopularity );
			if( dPopularity > dMaxPopularity ){
				dMaxPopularity = dPopularity;
			}
			if( dPopularity < dMinPopularity )
				dMinPopularity = dPopularity;
		}
		computeValidStates();
		m_bsFactory = new WordCompleteBeliefStateFactory( this );
		System.out.println( "Done loading items from " + sFileName );
		System.out.println( "Found " + m_cItems + " items. Max popularity " + dMaxPopularity + " min " + dMinPopularity );
	}
	private void computeValidStates() {
		String sItem = "";
		int iItem = 0;
		int iState = 0, iPrefix = 0, cCharacters = 0;
		m_vStates = new Vector<Integer>();
		m_aiStateIndexes = new int[m_cItems][];
		for( iItem = 0 ; iItem < m_cItems ; iItem++ ){
			sItem = m_vItems.elementAt( iItem );
			cCharacters = sItem.length();
			m_aiStateIndexes[iItem] = new int[cCharacters + 1];
			for( iPrefix = 0 ; iPrefix <= cCharacters ; iPrefix++ ){
				m_aiStateIndexes[iItem][iPrefix] = m_vStates.size();
				iState = makeStateInfo( iItem, iPrefix );
				m_vStates.add( iState );
			}
		}
		m_cStates = m_vStates.size();
		System.out.println( "Found " + m_vStates.size() + " valid states." );
	}
	
	public int getStateInfo( int iState ){
		return m_vStates.elementAt( iState );
	}
	
	public int getItem( int iStateInfo ){
		return iStateInfo % m_cItems;
	}
	private int getPrefix( int iStateInfo ){
		int iPrefix = ( iStateInfo / m_cItems ) % m_cMaxItemLength;
		return iPrefix;
	}
	private int getMaxSharedSubstring( int iItem1, int iItem2, int iStart ){
		String sItem1 = m_vItems.elementAt( iItem1 );
		String sItem2 = m_vItems.elementAt( iItem2 );
		
		if( sItem1.length() < iStart || sItem2.length() < iStart )
			return 0;
		
		if( !sItem1.substring( 0, iStart ).equals( sItem2.substring( 0, iStart ) ) )
			return 0;
		
		int idx = 0, cCharacters = Math.min( sItem1.length(), sItem2.length() );
		for( idx = iStart ; idx < cCharacters ; idx++ ){
			if( sItem1.charAt( idx ) != sItem2.charAt( idx ) )
				return idx - iStart;
		}
		return idx - iStart;
	}
	
	public int execute( int iAction, int iStartState ){
		int iStateInfo = getStateInfo( iStartState );
		int iStartItem = getItem( iStateInfo );
		int iEndItem = iStartItem;
		int iStartPrefix = getPrefix( iStateInfo );
		int iEndPrefix = -1;
				
		int iMaxSharedSubstring = getMaxSharedSubstring( iStartItem, iAction, iStartPrefix );
		if( iMaxSharedSubstring > 0 ){
			iEndPrefix = iStartPrefix + iMaxSharedSubstring;
		}
		else{
			iEndPrefix = iStartPrefix;
		}
		return makeState( iEndItem, iEndPrefix );
	}
	
	private int makeState( int iItem, int iPrefix ) {
		int iStateInfo = makeStateInfo( iItem, iPrefix );
		int iState = m_aiStateIndexes[iItem][iPrefix]; 
		if( m_vStates.elementAt( iState ) != iStateInfo )
			System.out.println( "BUGBUG" );
		return iState;
	}
	private int makeStateInfo( int iItem, int iPrefix ) {
		int iStateInfo = iPrefix;
		iStateInfo *= m_cItems;
		iStateInfo += iItem;
		return iStateInfo;
	}
	public double tr( int iStartState, int iAction, int iEndState ){
		int iTrueEndState = execute( iAction, iStartState );
		if( iTrueEndState == iEndState)
			return 1.0;
		return 0.0;
	}
	
	public int observe( int iAction, int iEndState ){
		System.out.println( "BUGBUG - no observe for WordComplete" );
		return -1;
	}
	
	public int observeForward( int iStartState, int iAction ){
		int iStateInfo = getStateInfo( iStartState );
		int iItem = getItem( iStateInfo ); 
		
		if( iItem == iAction )
			return 0;
		
		int iPrefix = getPrefix( iStateInfo );
		int iMaxSharedSubstring = -1;
		if( comparePrefix( iItem, iAction, iPrefix ) ){	
			iMaxSharedSubstring = getMaxSharedSubstring( iItem, iAction, iPrefix );
			if( iMaxSharedSubstring > 0 ){
				return 0;//forward
			}
		}
		if( largerThan( iAction, iItem, iPrefix ) ){
			return 1;//up
		}
		else{
			return 2;//down
		}
	}
	
	public double O( int iAction, int iState, int iObservation ){
		System.out.println( "BUGBUG - no O for WordComplete" );
		return -1;
	}
	
	public double O( int iStartState, int iAction, int iEndState, int iObservation ){
		int iTrueObservation = observeForward( iStartState, iAction );
		if( iTrueObservation == iObservation )
			return 1.0;
		return 0.0;
	}
	
	private boolean comparePrefix( int iItem1, int iItem2, int iPrefix ) {
		String sItem1 = m_vItems.elementAt( iItem1 );
		String sItem2 = m_vItems.elementAt( iItem2 );
		int idx = 0;
		if( sItem1.length() < iPrefix || sItem2.length() < iPrefix )
			return false;
		for( idx = 0 ; idx < iPrefix ; idx++ ){
			if( sItem1.charAt( idx ) != sItem2.charAt( idx ) )
				return false;
		}
		return true;
	}
	private boolean largerThan( int iItem1, int iItem2, int iStart ){
		String sItem1 = m_vItems.elementAt( iItem1 );
		String sItem2 = m_vItems.elementAt( iItem2 );
		if( sItem1.length() <= iStart )
			return false;
		if( sItem2.length() <= iStart )
			return true;
		return  sItem1.charAt( iStart ) > sItem2.charAt( iStart );
	}
	
	public boolean isTerminalState( int iState ){
		int iStateInfo = getStateInfo( iState );
		int iItem = getItem( iStateInfo );
		int iPrefix = getPrefix( iStateInfo );
		String sItem = m_vItems.elementAt( iItem );
		if( iPrefix == sItem.length() )
			return true;
		return false;
	}
	
	public double R( int iState, int iAction ){
		int iStateInfo = getStateInfo( iState );
		int iItem = getItem( iStateInfo );
		int iPrefix = getPrefix( iStateInfo );

		if( iPrefix == m_vItems.elementAt( iItem ).length() )
			return 0;

		//int iMaxSharedSubstring = getMaxSharedSubstring( iItem, iAction, iPrefix );
		//return iMaxSharedSubstring;
		
		if( iItem != iAction )
			return 0;
		return 1;
		
	}
	
	protected double maxReward( int iState ){
		
		int iStateInfo = getStateInfo( iState );
		int iItem = getItem( iStateInfo );
		int iPrefix = getPrefix( iStateInfo );
		String sItem = m_vItems.elementAt( iItem );
		return sItem.length() - iPrefix;
		
		//return 1;
	}

	public boolean terminalStatesDefined(){
		return true;
	}

	public int getStateCount() {
		return m_vStates.size();
	}

	public int getActionCount() {
		return m_cItems;
	}

	public int getObservationCount() {
		return 3;
	}

	public Iterator<Entry<Integer,Double>> getNonZeroTransitions( int iStartState, int iAction ) {
		Map<Integer, Double> m = new TreeMap<Integer, Double>();
		m.put( execute( iAction, iStartState ), 1.0 );
		return m.entrySet().iterator();
	}

	public Collection<Entry<Integer,Double>> getNonZeroBackwardTransitions( int iAction, int iEndState ) {
		System.out.println( "BUGBUG - no getNonZeroBackwardTransitions for WordComplete" );
		return null;
	}

	public double probStartState( int iState ){
		int iStateInfo = getStateInfo( iState );
		int iItem = getItem( iStateInfo );
		int iPrefix = getPrefix( iStateInfo );
		if( iPrefix == 0 )
			return m_vPopularity.elementAt( iItem );
		return 0.0;
	}
	
	private int m_iStartItem = 0;
	
	public int chooseStartState(){
		/*
		int iItem = -1;
		double dInitialProb = m_rndGenerator.nextDouble();
		double dProb = dInitialProb;
		while( dProb > 0 ){
			iItem++;
			dProb -= m_vPopularity.elementAt( iItem );
		}
		*/
		//assert iStartState >= 0 && iStartState < m_cStates;
		int iStartState =  makeState( m_iStartItem, 0 );
		m_iStartItem = ( m_iStartItem + 1 ) % m_cItems; 
		return iStartState;
	}
	
	public double getMinR(){
		return 0.0;
	}
	
	public double getMaxR(){
		return m_cMaxItemLength;
	}
	
	public double getMaxMinR(){
		return 0;
	}

	public int getStartStateCount() {
		return m_cItems;
	}

	public Iterator<Entry<Integer, Double>> getStartStates() {
		Map<Integer, Double> m = new TreeMap<Integer, Double>();
		int iItem = 0, iState = -1;
		for( iItem = 0 ; iItem < m_cItems ; iItem++ ){
			iState = makeState( iItem, 0 );
			m.put( iState, m_vPopularity.elementAt( iItem ) );
		}
		return m.entrySet().iterator();
	}
	
	public String getName(){
		return "WordComplete_" + m_cItems + "_" + m_cMaxItemLength;
	}
	/*
	public Collection<Integer> getValidStates(){
		return m_vStates;
	}
*/
	
	private int m_iStartState = 0;
	private double m_dSteps = 0;
	
	public double computeDiscountedRewardII( int cMaxStepsToGoal, PolicyStrategy policy, Vector<BeliefState> vObservedBeliefPoints, boolean bExplore, int[] aiActionCount ){
		double dDiscountedReward = 0.0, dCurrentReward = 0.0, dDiscountFactor = 1.0;
		int iStep = 0, iAction = 0, iObservation = 0;
		int iState = makeState( m_iStartState, 0 ), iNextState = 0;
		BeliefState bsCurrentBelief = getBeliefStateFactory().getInitialBeliefState(), bsNext = null;
		boolean bDone = false;
		int cRewards = 0;
		
		if( m_iStartState == 0 )
			m_dSteps = 0.0;
		
		for( iStep = 0 ; ( iStep < cMaxStepsToGoal ) && !bDone ; iStep++ ){			
			iAction = policy.getAction( bsCurrentBelief );
			
			iNextState = execute( iAction, iState );
			iObservation = observeForward( iState, iAction );

			dCurrentReward = R( iState, iAction ); //R(s,a)
			dDiscountedReward += dCurrentReward * dDiscountFactor;
			dDiscountFactor *= m_dGamma;
			
			if( dCurrentReward > 0 )
				cRewards++;
			//if( dCurrentReward < 0.0 )
			//	dCurrentReward = R( iState, iAction );

			bDone = endADR( iNextState, dCurrentReward );
			
			bsNext = bsCurrentBelief.nextBeliefState( iAction, iObservation );
			 
			iState = iNextState;
			bsCurrentBelief = bsNext;			
		}	
		
		m_dSteps += iStep * m_vPopularity.elementAt( m_iStartState );
		
		dDiscountedReward *= m_vPopularity.elementAt( m_iStartState ) * m_cItems;
		m_iStartState = ( m_iStartState + 1 ) % m_cItems; 
		
		if( m_iStartState == 0 )
			System.out.println( "Expected steps to goal " + m_dSteps );
		
		return dDiscountedReward;
	}
	
	public Collection<Integer> getRelevantActions( BeliefState bs ) {
		Iterator<Entry<Integer, Double>> itNonZero = bs.getNonZeroEntries().iterator();
		Entry<Integer, Double> e = null;
		int iState = 0, iStateInfo = 0, iItem = 0;
		Vector<Integer> vActions = new Vector<Integer>();
		while( itNonZero.hasNext() ){
			e = itNonZero.next();
			iState = e.getKey();
			iStateInfo = getStateInfo( iState );
			iItem = getItem( iStateInfo );
			vActions.add( iItem );
		}
		return vActions;
	}

	
	public double[] computeDiscountedRewardForPopularity(){
		int iItem = 0;
		double dSumDiscountedRewards = 0.0, dSteps = 0.0;
		double[] adRes = null;
		for( iItem = 0 ; iItem < m_cItems ; iItem++ ){
			adRes = computeDiscountedRewardForPopularity( iItem );
			dSumDiscountedRewards += adRes[0];
			dSteps += adRes[1];
		}
		return new double[]{dSumDiscountedRewards, dSteps};
	}
	
	public double[] computeDiscountedRewardForPopularity( int iStartItem ){
		double dDiscountedReward = 0.0, dCurrentReward = 0.0, dDiscountFactor = 1.0, dSteps = 0;
		int iStep = 0, iAction = 0, iObservation = 0;
		int iState = makeState( iStartItem, 0 ), iNextState = 0, iMostLikeliState = 0;
		BeliefState bsCurrentBelief = getBeliefStateFactory().getInitialBeliefState(), bsNext = null;
		boolean bDone = false;
		int cRewards = 0;
		
		while( !bDone ){	
			
			iMostLikeliState = bsCurrentBelief.getMostLikeliState();
			
			
			iAction = getItem( m_vStates.elementAt( iMostLikeliState ) );
			
			iNextState = execute( iAction, iState );
			iObservation = observeForward( iState, iAction );

			dCurrentReward = R( iState, iAction ); //R(s,a)
			dDiscountedReward += dCurrentReward * dDiscountFactor;
			dDiscountFactor *= m_dGamma;
			
			if( dCurrentReward > 0 )
				cRewards++;

			bDone = endADR( iNextState, dCurrentReward );
			
			bsNext = bsCurrentBelief.nextBeliefState( iAction, iObservation );
			 			
			iState = iNextState;
			bsCurrentBelief = bsNext;	
			iStep++;
			dSteps++;
		}	
		
		dDiscountedReward *= m_vPopularity.elementAt( iStartItem );
		dSteps *= m_vPopularity.elementAt( iStartItem );
		
		return new double[]{ dDiscountedReward, dSteps };
	}
	
	public double[] computeDiscountedRewardForBinarySearch(){
		int iItem = 0;
		double dSumDiscountedRewards = 0.0, dSteps = 0.0;
		double[] adRes = null;
		for( iItem = 0 ; iItem < m_cItems ; iItem++ ){
			adRes = computeDiscountedRewardForBinarySearch( iItem );
			dSumDiscountedRewards += adRes[0];
			dSteps += adRes[1];
		}
		return new double[]{dSumDiscountedRewards, dSteps};
	}
	
	public double[] computeDiscountedRewardForBinarySearch( int iStartItem ){
		double dDiscountedReward = 0.0, dCurrentReward = 0.0, dDiscountFactor = 1.0, dSteps = 0.0;
		int iStep = 0, iAction = 0, iObservation = 0;
		double dSumProbs = 0.0;
		TreeMap<String, Double> mItemProb = new TreeMap<String, Double>();
		Iterator<Entry<Integer, Double>> itNonZeroEntries = null;
		Entry<Integer, Double> eItemProb = null;
		Iterator<Entry<String, Double>> itItems = null;
		Entry<String, Double> eItem = null;
		int iItem = 0, iMedianItem = 0;
		int iState = makeState( iStartItem, 0 ), iNextState = 0;
		BeliefState bsCurrentBelief = getBeliefStateFactory().getInitialBeliefState(), bsNext = null;
		boolean bDone = false;
		int cRewards = 0;
		
		while( !bDone ){	
			
			itNonZeroEntries = bsCurrentBelief.getNonZeroEntries().iterator();
			mItemProb.clear();
			while( itNonZeroEntries.hasNext() ){
				eItemProb = itNonZeroEntries.next();
				iItem = getItem( getStateInfo( eItemProb.getKey() ) );
				mItemProb.put( m_vItems.elementAt( iItem ), eItemProb.getValue() );
			}
			dSumProbs = 0.0;
			itItems = mItemProb.entrySet().iterator();
			while( itItems.hasNext() && dSumProbs < 0.5 ){
				eItem = itItems.next();
				dSumProbs += eItem.getValue();
			}
			iMedianItem = m_vItems.indexOf( eItem.getKey() );
			
			iAction = getItem( iMedianItem );
			
			iNextState = execute( iAction, iState );
			iObservation = observeForward( iState, iAction );

			dCurrentReward = R( iState, iAction ); //R(s,a)
			dDiscountedReward += dCurrentReward * dDiscountFactor;
			dDiscountFactor *= m_dGamma;
			
			if( dCurrentReward > 0 )
				cRewards++;

			bDone = endADR( iNextState, dCurrentReward );
			
			bsNext = bsCurrentBelief.nextBeliefState( iAction, iObservation );
			 			
			iState = iNextState;
			bsCurrentBelief = bsNext;	
			iStep++;
			dSteps++;
		}	
		
		dDiscountedReward *= m_vPopularity.elementAt( iStartItem );
		dSteps *= m_vPopularity.elementAt( iStartItem );
		
		return new double[]{ dDiscountedReward, dSteps };
	}
	
	public static void main( String[] args ){
		WordComplete wc = new WordComplete();
		try {
			wc.load( "D:/NetFlix/RatingsCount.txt" );
		} catch (Exception e) {
			e.printStackTrace();
		} 
		AlphaVector.setAllowCaching( true );
		ForwardSearchValueIteration fsvi = new ForwardSearchValueIteration( wc, new WordCompleteHeuristicPolicy( wc ) );
		fsvi.valueIteration( 200, 0.05 );
		double[] adRes = null;
		adRes = wc.computeDiscountedRewardForPopularity();
		System.out.println( "Popularity = " + adRes[0] + ", " + adRes[1] );
		adRes = wc.computeDiscountedRewardForBinarySearch();
		System.out.println( "Binary search = " + adRes[0] + ", " + adRes[1] );
	}
}
