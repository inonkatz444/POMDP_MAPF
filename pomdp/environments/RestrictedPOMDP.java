package pomdp.environments;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Collection;
import java.util.Comparator;
import java.util.Iterator;
import java.util.TreeMap;
import java.util.Vector;
import java.util.Map.Entry;

import pomdp.algorithms.AlphaVectorsPolicy;
import pomdp.algorithms.PolicyStrategy;
import pomdp.algorithms.pointbased.ForwardSearchValueIteration;
import pomdp.algorithms.pointbased.HeuristicSearchValueIteration;
import pomdp.algorithms.pointbased.PerseusValueIteration;
import pomdp.algorithms.pointbased.StateBasedValueIteration;
import pomdp.environments.FactoredPOMDP.BeliefType;
import pomdp.utilities.AlphaVector;
import pomdp.utilities.BeliefState;
import pomdp.utilities.Logger;
import pomdp.utilities.MDPValueFunction;
import pomdp.utilities.RandomGenerator;
import pomdp.utilities.factored.AlgebraicDecisionDiagram.VariableTranslator;
import pomdp.utilities.factored.FactoredAlphaVector;
import pomdp.valuefunction.LinearValueFunctionApproximation;

public class RestrictedPOMDP extends FactoredPOMDP {

	protected FactoredPOMDP m_pFullPOMDP;
	protected Vector<Integer> m_vStateVariablesMap;
	protected Vector<Integer> m_vActionMap;
	protected double m_dSumStartStateProb;
	protected int m_iStartOption;
	
	public RestrictedPOMDP( FactoredPOMDP pFullPOMDP, Vector<Integer> vStateVariables, Vector<Integer> vActions ){
		this( pFullPOMDP, vStateVariables, vActions, 0 );
	}
	
	public RestrictedPOMDP( FactoredPOMDP pFullPOMDP, Vector<Integer> vStateVariables, Vector<Integer> vActions, int iStartOption ){
		String sMessage = "Started initializing restricted POMDPs, Variables = <";
		for( int iVar : vStateVariables ){
			sMessage += pFullPOMDP.getVariableName( iVar ) + ",";
		}
		sMessage += ">, Actions = ";
		for( int iAction : vActions ){
			sMessage += pFullPOMDP.getActionName( iAction ) + ",";
		}
		Logger.getInstance().log( "RestrictedPOMDP", 0, "<init>", sMessage );
		m_pFullPOMDP = pFullPOMDP;
		m_vStateVariablesMap = vStateVariables;
		m_vActionMap = vActions;
		m_cStateVariables = m_vStateVariablesMap.size();
		m_cActions = m_vActionMap.size();
		m_cObservations = pFullPOMDP.getObservationCount();
		m_btFactored = BeliefType.Factored;
		m_cStates = (int)Math.pow( 2, m_cStateVariables );
		m_dGamma = m_pFullPOMDP.m_dGamma;
		initBeliefStateFactory();
		getBeliefStateFactory().cacheBeliefStates( false );

		m_iStartOption = iStartOption;
		initStartStateProb();
		
		initADDs();
		//double dMdpAdr = computeMDPAverageDiscountedReward( 200, 250 );
		//System.out.println( "MDP ADR = " + dMdpAdr );
		//MDPValueFunction.PERSIST_FUNCTION = true;

		System.out.println( "Done initializing restricted POMDPs" );
	}
	
	public String getStateName( int iState ){
		int[][] aiRealState = restrictedStateToRealState( iState );
		return m_pFullPOMDP.getStateName( aiRealState );
	}
	
	private void initStartStateProb() {
		m_dSumStartStateProb = 0.0;
		int iStartState = 0;
		int[][] iRealStartState = null;
		for( iStartState = 0 ; iStartState < m_cStates ; iStartState++ ){
			iRealStartState = restrictedStateToRealState( iStartState );
			m_dSumStartStateProb += m_pFullPOMDP.probStartState( iRealStartState, m_iStartOption );
		}
	}
	
	public Vector<Integer> getConsistentStates( int iState ){
		int[][] aiRealState = restrictedStateToRealState( iState );
		Vector<int[][]> vConsistentRealStates = m_pFullPOMDP.getConsistentStates( aiRealState );
		Vector<Integer> vConsistentStates = new Vector<Integer>();
		for( int[][] aiState : vConsistentRealStates ){
			vConsistentStates.add( realStateToRestrictedState( aiState ) );
		}
		return vConsistentStates;
	}

	//avoid writing and reading the restricted POMDP models
	public void writeADDs( String sFileName ) throws IOException{
	}
	
	public boolean readADDs( String sFileName ) throws Exception{
		return false;
	}
	
	protected int[] getRewardRelevantVariables( int iAction ){
		return getRelevantVariables( iAction );
	}

	@Override
	public boolean[] indexToState( int iState ) {
		boolean[] abValues = new boolean[m_cStateVariables];
		int iVar = 0;
		for( iVar = 0 ; iVar < m_cStateVariables ; iVar++ ){
			abValues[iVar ] = ( iState % 2 == 1 );
			iState /= 2;
		}
		return abValues;
	}
		
	@Override
	protected int stateToIndex( boolean[] abState ) {
		int iVar = 0;
		int iState = 0;
		for( iVar = 0 ; iVar < m_cStateVariables ; iVar++ ){
			iState *= 2;
			if( abState[m_cStateVariables - iVar - 1] )
				iState++;
		}
		return iState;
	}

	@Override
	protected int stateToIndex( int[] aiStateVariableIndexes, boolean[] abStateVariableValues ) {
		int iVar = 0, cVars = aiStateVariableIndexes.length;
		int iState = 0, iValue = 0;
		for( iVar = 0 ; iVar < cVars ; iVar++ ){
			if( abStateVariableValues[iVar] ){
				iValue = 1 << aiStateVariableIndexes[iVar];
				iState += iValue;
			}
		}
		return iState;
	}

	public boolean isTerminalState( int iState ){
		int[][] aiRealState = restrictedStateToRealState( iState );
		return m_pFullPOMDP.isTerminalState( aiRealState );
	}
	
	public boolean terminalStatesDefined(){
		return m_pFullPOMDP.terminalStatesDefined();
	}
	
	@Override
	public int[] getRelevantVariables( int iAction ) {
		int iRealAction = getRealAction( iAction );
		int[] aiRealVariables = m_pFullPOMDP.getRelevantVariables( iRealAction );
		int[] aRelevant = getLocalVariables( aiRealVariables );
		return aRelevant;
	}

	protected int getRealAction( int iActionIndex ){
		return m_vActionMap.elementAt( iActionIndex );
	}
	
	protected int[] getRealVariables( int[] aiVariables ){
		int iVar = 0;
		int cVars = aiVariables.length;
		int[] aiRealVariables = new int[cVars];
		for( iVar = 0 ; iVar < cVars ; iVar++ ){
			aiRealVariables[iVar] = m_vStateVariablesMap.elementAt( aiVariables[iVar] );
		}
		return aiRealVariables;
	}
	
	protected int[] getRealVariables(){
		int iVar = 0;
		int[] aiRealVariables = new int[m_cStateVariables];
		for( iVar = 0 ; iVar < m_cStateVariables ; iVar++ ){
			aiRealVariables[iVar] = m_vStateVariablesMap.elementAt( iVar );
		}
		return aiRealVariables;
	}
	
	protected int[] getLocalVariables( int[] aiRealVariables ){
		int iVar = 0, iLocalVar = 0;
		int cVars = 0;
		for( int iReal : aiRealVariables ){
			if( m_vStateVariablesMap.contains( iReal ) ){
				cVars++;
			}
		}
		int[] aiVariables = new int[cVars];
		for( int iReal : aiRealVariables ){
			iLocalVar = m_vStateVariablesMap.indexOf( iReal );
			if( iLocalVar != -1 ){
				aiVariables[iVar] = iLocalVar;
				iVar++;
			}
		}
		return aiVariables;
	}
	
	public double probStartState( int iState ){
		int[][] aiRealState = restrictedStateToRealState( iState );
		double dProb = m_pFullPOMDP.probStartState( aiRealState, m_iStartOption );
		return dProb / m_dSumStartStateProb;
	}
	
	
	public double R( int iState, int iAction ){
		int[][] aiRealState = restrictedStateToRealState( iState );
		return m_pFullPOMDP.R( aiRealState, getRealAction( iAction ) );
	}
	
	public double O( int iAction, int iState, int iObservation ){
		int[][] aiRealState = restrictedStateToRealState( iState );
		double dProb = m_pFullPOMDP.O( getRealAction( iAction ), aiRealState, iObservation );
		return dProb;
	}
	
	public double tr( int iStartState, int iAction, int iEndState ){
		int[][] aiRealStartState = restrictedStateToRealState( iStartState );
		int[][] aiRealEndState = restrictedStateToRealState( iEndState );
		return m_pFullPOMDP.tr( aiRealStartState, getRealAction( iAction ), aiRealEndState );
	}
/*	
	protected double R( int[] aiVariables, boolean[] abValues, int iAction ){
		int[] aiRealVariables = getRealVariables( aiVariables );
		int iState = m_pFullPOMDP.partialToFull( aiRealVariables, abValues );
		return m_pFullPOMDP.R( iState, getRealAction( iAction ) );
	}
	*/
	public int observe( int iAction, int iState ){
		int[][] aiRealState = restrictedStateToRealState( iState );
		int iObservation = m_pFullPOMDP.observe( getRealAction( iAction ), aiRealState );
		return iObservation;
	}
	/*	
	public String getStateName( int iState ){
		int iRealState = restrictedStateToRealState( iState );
		return m_pFullPOMDP.getStateName( iRealState );
	}
	
	private void initStartStateProb() {
		m_dSumStartStateProb = 0.0;
		int iStartState = 0, iRealStartState = 0;
		for( iStartState = 0 ; iStartState < m_cStates ; iStartState++ ){
			iRealStartState = restrictedStateToRealState( iStartState );
			m_dSumStartStateProb += m_pFullPOMDP.probStartState( iRealStartState );
		}
	}
	public boolean isTerminalState( int iState ){
		int iRealState = restrictedStateToRealState( iState );
		return m_pFullPOMDP.isTerminalState( iRealState );
	}
	public double probStartState( int iState ){
		int iRealState = restrictedStateToRealState( iState );
		double dProb = m_pFullPOMDP.probStartState( iRealState );
		return dProb / m_dSumStartStateProb;
	}
	public double R( int iState, int iAction ){
		int iRealState = restrictedStateToRealState( iState );
		return m_pFullPOMDP.R( iRealState, getRealAction( iAction ) );
	}
	public double tr( int iStartState, int iAction, int iEndState ){
		int iRealStartState = restrictedStateToRealState( iStartState );
		int iRealEndState = restrictedStateToRealState( iEndState );
		return m_pFullPOMDP.tr( iRealStartState, getRealAction( iAction ), iRealEndState );
	}
	protected double R( int[] aiVariables, boolean[] abValues, int iAction ){
		int[] aiRealVariables = getRealVariables( aiVariables );
		int iState = m_pFullPOMDP.partialToFull( aiRealVariables, abValues );
		return m_pFullPOMDP.R( iState, getRealAction( iAction ) );
	}
	public int observe( int iAction, int iState ){
		int iRealState = restrictedStateToRealState( iState );
		int iObservation = m_pFullPOMDP.observe( getRealAction( iAction ), iRealState );
		return iObservation;
	}
	public int execute( int iAction, int iState ){
		int iRealState = restrictedStateToRealState( iState );
		int iRealNextState = m_pFullPOMDP.execute( getRealAction( iAction ), iRealState );
		int iNextState = realStateToRestrictedState( iRealNextState );
		return iNextState;
	}
*/
	public int execute( int iAction, int iState ){
		int[][] aiRealState = restrictedStateToRealState( iState );
		int[][] aiRealNextState = m_pFullPOMDP.execute( getRealAction( iAction ), aiRealState );
		int iNextState = realStateToRestrictedState( aiRealNextState );
		return iNextState;
	}

	public String getActionName( int iAction ){
		return m_pFullPOMDP.getActionName( getRealAction( iAction ) );
	}
	
	protected int[][] restrictedStateToRealState( int iRestrictedState ){
		int[] abValues = indexToState2( iRestrictedState );
		int[] aiRealVariables = getRealVariables();
		return new int[][]{ aiRealVariables, abValues };
	}
	protected int realStateToRestrictedState( int[][] aiRealState ){
		int[] aiRealVariables = getRealVariables();
		boolean[] abValues = new boolean[aiRealVariables.length];
		int iVarIdx = 0, iRealVarIdx = 0;
		for( iVarIdx = 0 ; iVarIdx < aiRealState[1].length && iRealVarIdx < aiRealVariables.length ; iVarIdx++ ){
			if( aiRealState[0][iVarIdx] == aiRealVariables[iRealVarIdx] ){
				abValues[iRealVarIdx] = aiRealState[1][iVarIdx] == 1;
				iRealVarIdx++;
			}
		}
		int iRestrictedState = stateToIndex( abValues );
		return iRestrictedState;
	}
	/*
	protected int restrictedStateToRealState( int iRestrictedState ){
		boolean[] abValues = indexToState( iRestrictedState );
		int[] aiRealVariables = getRealVariables();
		int iRealState = m_pFullPOMDP.stateToIndex( aiRealVariables, abValues );
		return iRealState;
	}
	protected int realStateToRestrictedState( int iRealState ){
		int[] aiRealVariables = getRealVariables();
		boolean[] abValues = m_pFullPOMDP.fullToPartial( iRealState, aiRealVariables );
		int iRestrictedState = stateToIndex( abValues );
		return iRestrictedState;
	}
	public Iterator<Entry<Integer,Double>> getNonZeroTransitions( int iStartState, int iAction ) {
		int iRealState = restrictedStateToRealState( iStartState );
		Iterator<Entry<Integer, Double>> itNonZeroTransitions = m_pFullPOMDP.getNonZeroTransitions( iRealState, getRealAction( iAction ) );
		TreeMap<Integer, Double> mNonZero = new TreeMap<Integer, Double>();
		Entry<Integer,Double> e = null;
		int iRealEndState = 0, iRestrictedEndState = 0;
		double dProb = 0.0;
		
		while( itNonZeroTransitions.hasNext() ){
			e = itNonZeroTransitions.next();
			iRealEndState = e.getKey();
			dProb = e.getValue();
			iRestrictedEndState = realStateToRestrictedState( iRealEndState );
			mNonZero.put( iRestrictedEndState, dProb );
		}
		return mNonZero.entrySet().iterator();
	}
	public Iterator<Entry<boolean[], Double>> getNonZeroTransitions( int[] aiRelevantVariables, 
			boolean[] abStateValues, int iAction ){
		int iStartState = partialToFull( aiRelevantVariables, abStateValues ), iEndState = 0;
		boolean[] abEndState = null;
		double dProb = 0.0;
		Iterator<Entry<Integer,Double>> itNonZeroTransitions = getNonZeroTransitions( iStartState, iAction );//use the local action since it will get converted in the called function
		TreeMap<boolean[], Double> mNonZero = new TreeMap<boolean[], Double>( new ArrayComparator() );
		Entry<Integer,Double> e = null;
		while( itNonZeroTransitions.hasNext() ){
			e = itNonZeroTransitions.next();
			iEndState = e.getKey();
			dProb = e.getValue();
			abEndState = fullToPartial( iEndState, aiRelevantVariables );
			mNonZero.put( abEndState, dProb );
		}
		return mNonZero.entrySet().iterator();
	}
	
	public double O( int iAction, int[] aiRelevantVariables, boolean[] abEndState, int iObservation ){
		int iEndState = stateToIndex( aiRelevantVariables, abEndState );
		return O( iAction, iEndState, iObservation );
	}

	
	*/
	public Iterator<Entry<Integer,Double>> getNonZeroTransitions( int iStartState, int iAction ) {
		int[][] aiRealState = restrictedStateToRealState( iStartState );
		Iterator<Entry<int[][], Double>> itNonZeroTransitions = m_pFullPOMDP.getNonZeroTransitions( aiRealState, getRealAction( iAction ) );
		TreeMap<Integer, Double> mNonZero = new TreeMap<Integer, Double>();
		Entry<int[][],Double> e = null;
		int[][] aiRealEndState = null;
		int iRestrictedEndState = 0;
		double dProb = 0.0, dSumProbs = 0.0;
		
		while( itNonZeroTransitions.hasNext() ){
			e = itNonZeroTransitions.next();
			aiRealEndState = e.getKey();
			dProb = e.getValue();
			dSumProbs += dProb;
			iRestrictedEndState = realStateToRestrictedState( aiRealEndState );
			mNonZero.put( iRestrictedEndState, dProb );
		}
		
		return mNonZero.entrySet().iterator();
	}

	
	public Iterator<Entry<boolean[], Double>> getNonZeroTransitions( int[] aiRelevantVariables, 
			boolean[] abStateValues, int iAction ){
		int iStartState = partialToFull( aiRelevantVariables, abStateValues ), iEndState = 0;
		boolean[] abEndState = null;
		double dProb = 0.0;
		Iterator<Entry<Integer,Double>> itNonZeroTransitions = getNonZeroTransitions( iStartState, iAction );//use the local action since it will get converted in the called function
		TreeMap<boolean[], Double> mNonZero = new TreeMap<boolean[], Double>( new ArrayComparator() );
		Entry<Integer,Double> e = null;
		while( itNonZeroTransitions.hasNext() ){
			e = itNonZeroTransitions.next();
			iEndState = e.getKey();
			dProb = e.getValue();
			abEndState = fullToPartial( iEndState, aiRelevantVariables );
			mNonZero.put( abEndState, dProb );
		}
		return mNonZero.entrySet().iterator();
	}
	
	public double O( int iAction, int[] aiRelevantVariables, boolean[] abEndState, int iObservation ){
		int iEndState = stateToIndex( aiRelevantVariables, abEndState );
		return O( iAction, iEndState, iObservation );
	}

	
	@Override
	public boolean changingComponent(int component, int action, int observation) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public int[] getIndependentComponentVariables(int component) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public int getIndependentComponentsCount() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getInitialComponenetValueProbability(int component, int value) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected double getInitialVariableValueProbability(int variable, int value) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getInitialVariableValueProbability(int variable, boolean value) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected String getObservationName() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	protected int[] getObservationRelevantVariables(int action) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	protected int[] getObservationRelevantVariablesMultiValue(int action) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	protected int getRealStateVariableCount() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected String getRealVariableName(int variable) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public int[] getRelevantComponents(int action, int observation) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public int[] getRelevantComponentsForComponent(int action, int component) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public int[] getRelevantVariablesForComponent(int action, int component) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	protected int[] getRelevantVariablesMultiValue(int action, int variable) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	protected int[] getRewardRelevantVariablesMultiValue(int action) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	protected int getValueCount(int variable) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected String getValueName(int variable, int value) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	protected String getVariableName(int variable) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public double observationGivenRelevantVariables(int action,
			int observation, int[] aiRelevantVariables, boolean[] abValues) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected double observationGivenRelevantVariablesMultiValue(int action,
			int observation, int[] aiRelevantVariables, int[] aiValues) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected boolean relevantTransitionRealVariable(int action, int variable) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	protected boolean relevantTransitionVariable(int action, int variable) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public double rewardGivenRelevantVariables(int action,
			int[] aiRelevantVariables, boolean[] abValues) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected double rewardGivenRelevantVariablesMultiValue(int action,
			int[] aiRelevantVariables, int[] aiValues) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected double stateVariableObservation(int[] aiStateVariableIndexes,
			int action, boolean[] abStateVariableValues, int observation) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected double stateVariableTransition(int[] aiStateVariableIndexes,
			boolean[] abStateVariableValuesBefore, int action,
			boolean[] abStateVariableValuesAfter) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected double stateVariableTransition(int state, int action,
			int stateVariable) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double transitionGivenRelevantVariables(int action, int variable,
			boolean value, int[] aiRelevantVariables, boolean[] abValues) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double transitionGivenRelevantVariables(int action,
			int[] aiComponent, boolean[] abComponentValues,
			int[] aiRelevantVariables, boolean[] abRelevantValues) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected double transitionGivenRelevantVariablesMultiValue( int iAction,
			int iVariable, int value, int[] aiRelevantVariables, int[] atValues) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double O(int realAction, int[][] aiRealState, int observation) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double R(int[][] aiRealState, int realAction) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public int[][] execute(int realAction, int[][] aiRealState) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public String getStateName(int[][] aiRealState) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public boolean isTerminalState(int[][] aiRealState) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public int observe(int realAction, int[][] aiRealState) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double probStartState(int[][] aiRealState) {
		// TODO Auto-generated method stub
		return 0;
	}
	@Override
	public double probStartState(int[][] aiRealState, int iOption) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double tr(int[][] aiRealStartState, int realAction,
			int[][] aiRealEndState) {
		// TODO Auto-generated method stub
		return 0;
	}
	

	@Override
	public Iterator<Entry<int[][], Double>> getNonZeroTransitions(
			int[][] aiState, int action) {
		// TODO Auto-generated method stub
		return null;
	}
	@Override
	public int[][] chooseStartState2() {
		// TODO Auto-generated method stub
		return null;
	}

	
	public double computeDiscountedReward( int cMaxStepsToGoal, PolicyStrategy policy, Vector<BeliefState> vObservedBeliefPoints, boolean bExplore, int[] aiActionCount ){
		return computeDiscountedRewardII( cMaxStepsToGoal, policy, vObservedBeliefPoints, bExplore, aiActionCount );
	}
	
	public void translateVectorsToReal( LinearValueFunctionApproximation vRestrictedValueFunction, LinearValueFunctionApproximation vRealValueFunction, boolean bCombine ) {
		if( bCombine )
			translateVectorsToRealAndCombine( vRestrictedValueFunction, vRealValueFunction );
		else
			translateVectorsToReal( vRestrictedValueFunction, vRealValueFunction );			
	}
	
	public void translateVectorsToRealAndCombine( LinearValueFunctionApproximation vRestrictedValueFunction, LinearValueFunctionApproximation vRealValueFunction ) {
		FactoredAlphaVector favNew = null, favExisting = null, favCombined = null;
		Vector<AlphaVector> vCombined = new Vector<AlphaVector>();
		Collection<AlphaVector> vOriginal = vRealValueFunction.getVectors();
		VariableTranslator vt = new RestrictedToRealVariableTranslator();
		FactoredAlphaVector fav0 = (FactoredAlphaVector) vRestrictedValueFunction.getMaxAlpha( getBeliefStateFactory().getInitialBeliefState() );
		boolean bFound = false;
		
		for( AlphaVector avNew : vRestrictedValueFunction.getVectors() ){
			favNew = (FactoredAlphaVector)avNew.copy();
			favNew.attach( m_pFullPOMDP );
			favNew.translate( vt );
			favNew.setAction( getRealAction( avNew.getAction() ) );
			favNew.finalizeValues();
			
			favExisting = null;
			for( AlphaVector avExisting : vOriginal ){
				if( avExisting.getAction() == favNew.getAction() ){
					favExisting = (FactoredAlphaVector)avExisting;
					break;
				}
			}
			if( favExisting != null ){
				vOriginal.remove( favExisting );
				favExisting.combine( favNew );
				vCombined.add( favExisting );
			}
			else{
				vCombined.add( favNew );
			}
		}
		vRealValueFunction.clear();
		for( AlphaVector av : vOriginal ){
			vRealValueFunction.addPrunePointwiseDominated( av );
		}
		for( AlphaVector av : vCombined ){
			vRealValueFunction.addPrunePointwiseDominated( av );
		}
	}

	
	public void translateVectorsToReal( LinearValueFunctionApproximation vRestrictedValueFunction, LinearValueFunctionApproximation vRealValueFunction ) {
		FactoredAlphaVector fav = null;
		VariableTranslator vt = new RestrictedToRealVariableTranslator();
		FactoredAlphaVector fav0 = (FactoredAlphaVector) vRestrictedValueFunction.getMaxAlpha( getBeliefStateFactory().getInitialBeliefState() );
		for( AlphaVector av : vRestrictedValueFunction.getVectors() ){
			fav = (FactoredAlphaVector)av.copy();
			fav.attach( m_pFullPOMDP );
			fav.translate( vt );
			fav.setAction( getRealAction( av.getAction() ) );
			fav.finalizeValues();
			vRealValueFunction.addPrunePointwiseDominated( fav );
		}
	}
	
	public void translateVectorsToReal( LinearValueFunctionApproximation vRestrictedValueFunction, LinearValueFunctionApproximation vRealValueFunction, Vector<Integer> vVariables ) {
		FactoredAlphaVector fav = null;
		VariableTranslator vt = new RestrictedToRealVariableTranslator();
		for( AlphaVector av : vRestrictedValueFunction.getVectors() ){
			fav = (FactoredAlphaVector)av.copy();
			fav.attach( m_pFullPOMDP );
			fav.translate( vt );
			fav.assumeWorstCase( vVariables );
			fav.setAction( getRealAction( av.getAction() ) );
			fav.finalizeValues();
			vRealValueFunction.addPrunePointwiseDominated( fav );
		}
	}
	
	private class RestrictedToRealVariableTranslator implements VariableTranslator{

		@Override
		public int translate( int iVar ) {
			return m_vStateVariablesMap.elementAt( iVar );
		}

		@Override
		public int translateVariableCount( int cVariables ) {
			return m_pFullPOMDP.m_cStateVariables;
		}
		
	}
	
	private class ArrayComparator implements Comparator<boolean[]>{

		@Override
		public int compare( boolean[] arg0, boolean[] arg1 ) {
			int i = 0;
			for( i = 0 ; i < arg0.length ; i++ ){
				if( arg0[i] ){
					if( !arg1[i] )
						return 1;
				}
				else{
					if( arg1[i] )
						return -1;
				}
			}
			return 0;
		}
		
	}

	
	public static void addMachineActions( int iMachine, int cMachines, Vector<Integer> vActions ){		
		vActions.add( iMachine ); //restart
		vActions.add( cMachines + iMachine ); //ping 
	}
	
	public static void testNetwork( int cMachines, int cMachinesPerModel ){
		NetworkManagement pomdp = new NetworkManagement( cMachines, BeliefType.Flat );
		LinearValueFunctionApproximation vValueFunction = new LinearValueFunctionApproximation();
		AlphaVectorsPolicy policy = new AlphaVectorsPolicy();
		policy.setValueFunction( vValueFunction );
		long lTimeBefore = System.currentTimeMillis();
		int iMachine = 0;

		for( iMachine = 0 ; iMachine < cMachines ; iMachine++ ){
			Vector<Integer> vVariables = new Vector<Integer>(), vActions = new Vector<Integer>();;
			int iVar = 0, iVariable = 0;
			for( iVar = 0 ; iVar < cMachinesPerModel ; iVar++ ){
				iVariable = ( iMachine + iVar ) % cMachines;
				vVariables.add( iVariable );
				addMachineActions( iVariable, cMachines, vActions );
			}
			vActions.add( 2 * cMachines ); //no-op

			vVariables = sort( vVariables );
			vActions = sort( vActions );
			
			RestrictedPOMDP rpomdp = new RestrictedPOMDP( pomdp, vVariables, vActions );
			//ForwardSearchValueIteration hsvi = new ForwardSearchValueIteration( rpomdp );
			HeuristicSearchValueIteration hsvi = new HeuristicSearchValueIteration( rpomdp );
			hsvi.valueIteration( 10 );	
			rpomdp.translateVectorsToReal( hsvi.getValueFunction(), vValueFunction, vVariables );
			//rpomdp.translateVectorsToReal( hsvi.getValueFunction(), vValueFunction, false );			
		}		
		long lTimeAfter = System.currentTimeMillis();
		System.out.println( "Total time for restricted POMDPs " + ( lTimeAfter - lTimeBefore ) / 1000 );
		pomdp.computeAverageDiscountedReward( 200, 100, policy );
		//System.exit( 0 );
		MDPValueFunction.PERSIST_FUNCTION = true;
		//ForwardSearchValueIteration hsvi = new ForwardSearchValueIteration( pomdp );
		HeuristicSearchValueIteration hsvi = new HeuristicSearchValueIteration( pomdp );
		//hsvi.valueIteration( 10 );
		//pomdp.computeAverageDiscountedReward( 1000, 100, hsvi );
		System.out.println( "QMDP policy" );
		pomdp.computeAverageDiscountedReward( 200, 100, hsvi.getMDPValueFunction() );
	}
	

	public static int testRockSample( int cRocks, int cRocksPerModel, boolean bCombine ) throws Exception{
		int cX = 8, cY = 8, cXYBits = 6;
		BeliefType bt = BeliefType.Factored;
		if( cX > 8 || cY > 8 || cRocks > 8 )
			bt = BeliefType.Independent;
		ModifiedRockSample pomdp = new ModifiedRockSample( cX, cY, cRocks, bt );
		LinearValueFunctionApproximation vValueFunction = new LinearValueFunctionApproximation();
		
		if(cRocksPerModel == 0)
		{
			ForwardSearchValueIteration fsvi = new ForwardSearchValueIteration( pomdp );
			//fsvi.setValueFunction( vValueFunction );
			
			fsvi.valueIteration( 20 );	
			return fsvi.getValueFunction().size();
		}
		
		
		Vector<Integer> vStartPositions = pomdp.getStartOptions();
		Logger.getInstance().setOutputStream( pomdp.getName() + "_" + cRocksPerModel + ".txt" );
		AlphaVectorsPolicy policy = new AlphaVectorsPolicy();
		long lTimeBefore = 0, cTotalTime = 0;
		policy.setValueFunction( vValueFunction );
		int iRock = 0, iOtherRock = 0;
		RandomGenerator rnd = new RandomGenerator( "RestrictedPOMDP" );
		MDPValueFunction.PERSIST_FUNCTION = false;

		for( iRock = 0 ; iRock < cRocks ; iRock++ ){
			
			System.out.println( "Starting rock " + iRock );
			
			lTimeBefore = System.currentTimeMillis();
			Vector<Integer> vVariables = new Vector<Integer>(), vActions = new Vector<Integer>();
			int iVar = 0;
			Vector<Integer> vRocks = new Vector<Integer>();
			vRocks.add( iRock );
			if( bCombine ){
				for(iOtherRock = iRock + 1 ; iOtherRock - iRock <= cRocksPerModel ; iOtherRock++ ){
					vRocks.add( iOtherRock % cRocks );
				}
				iRock = iOtherRock;
			}
			else{				
				while( vRocks.size() < cRocksPerModel ){
					iOtherRock = rnd.nextInt( cRocks );
					//iOtherRock = pomdp.getClosestRock( vRocks );
					if( !vRocks.contains( iOtherRock ) )
						vRocks.add( iOtherRock );
				}
			}
			vRocks = sort( vRocks );
			
			for( iVar = 0 ; iVar < cXYBits ; iVar++ ){
				vVariables.add( iVar );
			}
			for( int i : vRocks )
				vVariables.add( cXYBits + i );
			//4 actions - north east south west
			for( iVar = 0 ; iVar < 4 ; iVar++ ){
				vActions.add( iVar );
			}
			//appropriate check actions
			for( int i : vRocks )
				vActions.add( 4 + i );
			vActions.add( 4 + cRocks );

			//for( int iStartOption : vStartPositions ){
				//RestrictedPOMDP rpomdp = new RestrictedPOMDP( pomdp, vVariables, vActions, iStartOption );
			RestrictedPOMDP rpomdp = new RestrictedPOMDP( pomdp, vVariables, vActions );
				//StateBasedValueIteration sbvi = new StateBasedValueIteration( rpomdp );
				ForwardSearchValueIteration fsvi = new ForwardSearchValueIteration( rpomdp );
				//HeuristicSearchValueIteration hsvi = new HeuristicSearchValueIteration( rpomdp );
				fsvi.valueIteration( 20 );
				//sbvi.setValueFunction( fsvi.getValueFunction() );
				//sbvi.valueIteration( 2 );
				//hsvi.setValueFunction( fsvi.getValueFunction() );
				//hsvi.valueIteration( 10 );	
				long lTimeAfter = System.currentTimeMillis();
				cTotalTime += lTimeAfter - lTimeBefore;
				/*
				hsvi.getValueFunction().initHitCounts();
				double d = rpomdp.computeAverageDiscountedReward( 100, 100, hsvi );
				int cBefore = hsvi.getValueFunction().getVectors().size();
				hsvi.getValueFunction().pruneLowHitCountVectors( 0 );
				System.out.println( "Policy evaluation over restricted POMDP, ADR = " + d + " pruned from " + cBefore + " to " +  hsvi.getValueFunction().getVectors().size() + ", time " + cTotalTime / 1000 );
				*/
				
				rpomdp.translateVectorsToReal( fsvi.getValueFunction(), vValueFunction, bCombine );
			//}
				/*
			int cBefore = vValueFunction.size();
			vValueFunction.pruneRandomSampling( pomdp.getBeliefStateFactory(), 10000 );
			System.out.println( "Policy evaluation over restricted POMDP, pruned from " + cBefore + " to " +  vValueFunction.size() + ", time " + cTotalTime / 1000 );
			
			if( iRock == 8 ){
			double d1 = rpomdp.computeAverageDiscountedReward( 100, 100, fsvi );
			double d2 = pomdp.computeAverageDiscountedReward( 500, 100, policy );
			System.out.println( "Policy evaluation over true POMDP, ADR = " + d1 + " restricted " + d2 );
			}
			*/
		}		
		System.out.println( "Total time for restricted POMDPs " + cTotalTime / 1000 + " RockSample "+ cX + "," + cY + "," + cRocks + "," + cRocksPerModel + " |V|=" + vValueFunction.size() );
		return vValueFunction.size();
		/*
		pomdp.computeAverageDiscountedReward( 500, 100, policy );
		System.exit( 0 );
		MDPValueFunction.PERSIST_FUNCTION = true;
		//ForwardSearchValueIteration hsvi = new ForwardSearchValueIteration( pomdp );
		HeuristicSearchValueIteration hsvi = new HeuristicSearchValueIteration( pomdp );
		hsvi.setValueFunction( vValueFunction );
		hsvi.valueIteration( 10 );	
		pomdp.computeAverageDiscountedReward( 500, 100, hsvi );
		System.out.println( "QMDP policy" );
		pomdp.computeAverageDiscountedReward( 1000, 100, hsvi.getMDPValueFunction() );
		*/
	}
	
	private static Vector<Integer> sort( Vector<Integer> v) {
		Vector<Integer> vSorted = new Vector<Integer>();
		Integer nMin = null;
		while( !v.isEmpty() ){
			nMin = null;
			for( Integer n : v ){
				if( ( nMin == null ) || ( nMin.doubleValue() > n.doubleValue() ) )
					nMin = n;
			}
			v.remove( nMin );
			vSorted.add( nMin );
		}
		return vSorted;
	}
	
	private static int testLogistics( int cPackages, int cTrucks, int cPackagesPerModel, boolean bAllTrucks ) throws Exception{
		int cCities = 4, cCityBits = 2, cPackagesBits = cPackages * cCityBits, cTrucksBits = cTrucks * cCityBits;
		BeliefType bt = BeliefType.Factored;
		if( cPackages > 4 )
			bt = BeliefType.Independent;
		Logistics pomdp = new Logistics( cCities, cTrucks, cPackages, bt );
		if( bAllTrucks ){
			Logger.getInstance().setOutputStream( pomdp.getName() + "_" + cPackagesPerModel + "_" + "AllTrucks" + ".txt" );
		}
		else{
			Logger.getInstance().setOutputStream( pomdp.getName() + "_" + cPackagesPerModel + "_" + "OneTruck" + ".txt" );			
		}
		LinearValueFunctionApproximation vValueFunction = new LinearValueFunctionApproximation();
		AlphaVectorsPolicy policy = new AlphaVectorsPolicy();
		long lTimeBefore = System.currentTimeMillis();
		policy.setValueFunction( vValueFunction );
		int iPackage = 0, iActiveTruck = 0, iOtherPackage = 0;
		RandomGenerator rnd = new RandomGenerator( "RestrictedPOMDP" );
		MDPValueFunction.PERSIST_FUNCTION = false;
		
		
		if(cPackagesPerModel == 0)
		{
			ForwardSearchValueIteration fsvi = new ForwardSearchValueIteration( pomdp );
			//fsvi.setValueFunction( vValueFunction );
			System.out.println( "Optimal policy" );
			fsvi.valueIteration( 20 );	
			return fsvi.getValueFunction().size();
			
		
		}
		
		for( iActiveTruck = 0 ; iActiveTruck < cTrucks ; iActiveTruck++ ){
			Vector<Integer> vTrucks = new Vector<Integer>();
			if( bAllTrucks ){
				for( int iTruck = 0 ; iTruck < cTrucks ; iTruck++ ){
					vTrucks.add( iTruck );
				}
			}
			else{
				vTrucks.add( iActiveTruck );
			}
			for( iPackage = 0 ; iPackage < cPackages ; iPackage++ ){
				Vector<Integer> vVariables = new Vector<Integer>(), vActions = new Vector<Integer>();
				int iVar = 0;
				Vector<Integer> vPackages = new Vector<Integer>();
				vPackages.add( iPackage );
				while( vPackages.size() < cPackagesPerModel ){
					iOtherPackage = rnd.nextInt( cPackages );
					if( !vPackages.contains( iOtherPackage ) )
						vPackages.add( iOtherPackage );
				}
				vPackages = sort( vPackages );
				
				for( int i : vPackages ){
					for( iVar = pomdp.getFirstBitForPackage( i ) ; iVar < pomdp.getFirstBitForPackage( i + 1 ) ; iVar++ )
						vVariables.add( iVar );
				}
				for( int iTruck : vTrucks ){
					for( iVar = pomdp.getFirstBitForTruck( iTruck ) ; iVar < pomdp.getFirstBitForTruck( iTruck + 1 ) ; iVar++ ){
						vVariables.add( iVar );
					}
				}
				
				for( int iAction = 0 ; iAction < pomdp.getActionCount() ; iAction++ ){
					if( pomdp.isLoadUnloadAction( iAction ) && vPackages.contains( pomdp.getActivePackage( iAction ) ) && vTrucks.contains( pomdp.getActiveTruck( iAction ) ) ){
						vActions.add( iAction );
					}
					else if( pomdp.isPingPackage( iAction ) && vPackages.contains( pomdp.getActivePackage( iAction ) ) ){
							vActions.add( iAction );
					}
					else{
						if( ( pomdp.isDriveAction( iAction ) || pomdp.isPingTruck( iAction ) ) && vTrucks.contains( pomdp.getActiveTruck( iAction ) ) )
							vActions.add( iAction );
					}
				}
	
				RestrictedPOMDP rpomdp = new RestrictedPOMDP( pomdp, vVariables, vActions );
				ForwardSearchValueIteration hsvi = new ForwardSearchValueIteration( rpomdp );
				//HeuristicSearchValueIteration hsvi = new HeuristicSearchValueIteration( rpomdp );
				hsvi.valueIteration( 20 );	
				/*
				hsvi.getValueFunction().initHitCounts();
				double d = rpomdp.computeAverageDiscountedReward( 500, 100, hsvi );
				int cBefore = hsvi.getValueFunction().getVectors().size();
				hsvi.getValueFunction().pruneLowHitCountVectors( 0 );
				System.out.println( "Policy evaluation over restricted POMDP, ADR = " + d + " pruned from " + cBefore + " to " +  hsvi.getValueFunction().getVectors().size() );
				*/
				rpomdp.translateVectorsToReal( hsvi.getValueFunction(), vValueFunction );
				
				
			}
			if( bAllTrucks )
				break;
		}
		long lTimeAfter = System.currentTimeMillis();
		System.out.println( "Total time for restricted POMDPs " + ( lTimeAfter - lTimeBefore ) / 1000 + " Logisitcs " + cCities + "," + cTrucks + "," + cPackages + " - " + cPackagesPerModel + " all trucks " + bAllTrucks + ", |V|=" + vValueFunction.size() );
		return vValueFunction.size();
		//pomdp.computeAverageDiscountedReward( 500, 100, policy );
		
		/*
		//System.exit( 0 );
		//MDPValueFunction.PERSIST_FUNCTION = true;
		ForwardSearchValueIteration hsvi = new ForwardSearchValueIteration( pomdp );
		//HeuristicSearchValueIteration hsvi = new HeuristicSearchValueIteration( pomdp );
		hsvi.setValueFunction( vValueFunction );
		System.out.println( "Optimal policy" );
		hsvi.valueIteration( 10 );	
		pomdp.computeAverageDiscountedReward( 500, 100, hsvi );
		*/
		//System.out.println( "QMDP policy" );
		//hsvi.getMDPValueFunction().makeVectors();
		//pomdp.computeAverageDiscountedReward( 1000, 100, hsvi.getMDPValueFunction() );
	}
	
	private static void testLogistics( int cPackages, int cTrucks, int cPackagesPerModel ){
		int cCities = 4, cCityBits = 2, cPackagesBits = cPackages * cCityBits, cTrucksBits = cTrucks * cCityBits;
		Logistics pomdp = new Logistics( cCities, cTrucks, cPackages, BeliefType.Factored );
		LinearValueFunctionApproximation vValueFunction = new LinearValueFunctionApproximation();
		AlphaVectorsPolicy policy = new AlphaVectorsPolicy();
		long lTimeBefore = System.currentTimeMillis();
		policy.setValueFunction( vValueFunction );
		int iPackage = 0, iActiveTruck = 0, iOtherPackage = 0;
		RandomGenerator rnd = new RandomGenerator( "RestrictedPOMDP" );
		MDPValueFunction.PERSIST_FUNCTION = false;
		
		//for( iActiveTruck = 0 ; iActiveTruck < cTrucks ; iActiveTruck++ ){
			Vector<Integer> vTrucks = new Vector<Integer>();
			//vTrucks.add( iActiveTruck );
			vTrucks.add( 0 );
			vTrucks.add( 1 );
			for( iPackage = 0 ; iPackage < cPackages ; iPackage++ ){
				Vector<Integer> vVariables = new Vector<Integer>(), vActions = new Vector<Integer>();
				int iVar = 0;
				Vector<Integer> vPackages = new Vector<Integer>();
				vPackages.add( iPackage );
				while( vPackages.size() < cPackagesPerModel ){
					iOtherPackage = rnd.nextInt( cPackages );
					if( !vPackages.contains( iOtherPackage ) )
						vPackages.add( iOtherPackage );
				}
				vPackages = sort( vPackages );
				
				for( int i : vPackages ){
					for( iVar = pomdp.getFirstBitForPackage( i ) ; iVar < pomdp.getFirstBitForPackage( i + 1 ) ; iVar++ )
						vVariables.add( iVar );
				}
				for( int iTruck : vTrucks ){
					for( iVar = pomdp.getFirstBitForTruck( iTruck ) ; iVar < pomdp.getFirstBitForTruck( iTruck + 1 ) ; iVar++ ){
						vVariables.add( iVar );
					}
				}
				
				for( int iAction = 0 ; iAction < pomdp.getActionCount() ; iAction++ ){
					if( pomdp.isLoadUnloadAction( iAction ) && vPackages.contains( pomdp.getActivePackage( iAction ) ) && vTrucks.contains( pomdp.getActiveTruck( iAction ) ) ){
						vActions.add( iAction );
					}
					else if( pomdp.isPingPackage( iAction ) && vPackages.contains( pomdp.getActivePackage( iAction ) ) ){
							vActions.add( iAction );
					}
					else{
						if( ( pomdp.isDriveAction( iAction ) || pomdp.isPingTruck( iAction ) ) && vTrucks.contains( pomdp.getActiveTruck( iAction ) ) )
							vActions.add( iAction );
					}
				}
	
				RestrictedPOMDP rpomdp = new RestrictedPOMDP( pomdp, vVariables, vActions );
				ForwardSearchValueIteration hsvi = new ForwardSearchValueIteration( rpomdp );
				//HeuristicSearchValueIteration hsvi = new HeuristicSearchValueIteration( rpomdp );
				hsvi.valueIteration( 50 );	
				rpomdp.translateVectorsToReal( hsvi.getValueFunction(), vValueFunction );			
				//double d = pomdp.computeAverageDiscountedReward( 500, 100, policy );
				//System.out.println( "Policy evaluation over true POMDP, ADR = " + d );
			}
		//}
		long lTimeAfter = System.currentTimeMillis();
		System.out.println( "Total time for restricted POMDPs " + ( lTimeAfter - lTimeBefore ) / 1000 + " Logisitcs " + cCities + "," + cTrucks + "," + cPackages + " - " + cPackagesPerModel );
		pomdp.computeAverageDiscountedReward( 500, 100, policy );
		
		
		//System.exit( 0 );
		//MDPValueFunction.PERSIST_FUNCTION = true;
		ForwardSearchValueIteration hsvi = new ForwardSearchValueIteration( pomdp );
		//HeuristicSearchValueIteration hsvi = new HeuristicSearchValueIteration( pomdp );
		hsvi.setValueFunction( vValueFunction );
		System.out.println( "Optimal policy" );
		hsvi.valueIteration( 10 );	
		pomdp.computeAverageDiscountedReward( 500, 100, hsvi );
		
		//System.out.println( "QMDP policy" );
		//hsvi.getMDPValueFunction().makeVectors();
		//pomdp.computeAverageDiscountedReward( 1000, 100, hsvi.getMDPValueFunction() );
	}
	
	public static void main( String[] args ) throws Exception{
		
		//testNetwork( 6, 3 );
		/*
		int[][] a = new int[10][9];
		for(int i = 0 ; i < 8 ; i++)
			for(int j = 0 ; j < 10 ; j++)
			{
				try
				{
				a[j][i] = testRockSample( 8, i, false );
				}
				catch(Exception e)
				{
					a[j][i] = -1;
				
				}
				catch(Error e)
				{
					a[j][i] = -1;
				}
				
			}
		
		for(int i = 0 ; i < 8 ; i++)
		{
			for(int j = 0 ; j < 10 ; j++)
				System.out.print(a[j][i] + ",");
			System.out.println();
		}
		*/
		
		int[][] a = new int[10][9];
		for(int i = 0 ; i < 3 ; i++)
			for(int j = 0 ; j < 10 ; j++)
			{
				try
				{
					a[j][i] = testLogistics( 4, 2, i, false );
				}
				catch(Exception e)
				{
					a[j][i] = -1;
				
				}
				catch(Error e)
				{
					a[j][i] = -1;
				}
				
			}
		
		for(int i = 0 ; i < 8 ; i++)
		{
			for(int j = 0 ; j < 10 ; j++)
				System.out.print(a[j][i] + ",");
			System.out.println();
		}
				
	}


}
