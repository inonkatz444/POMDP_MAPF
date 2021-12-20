package pomdp.environments;

import java.io.IOException;
import java.util.Collection;
import java.util.Iterator;
import java.util.Vector;
import java.util.Map.Entry;

import pomdp.algorithms.PolicyStrategy;
import pomdp.environments.POMDP.RewardType;
import pomdp.utilities.AlphaVector;
import pomdp.utilities.BeliefState;
import pomdp.utilities.InvalidModelFileFormatException;
import pomdp.utilities.LimitedBeliefStateFactory;
import pomdp.utilities.MDPValueFunction;

public class POMDPAdapter extends POMDP {
	private static final long serialVersionUID = 8346810645393914200L;
	protected POMDP m_pPOMDP;
	
	public POMDPAdapter( POMDP pomdp ){
		super();
		m_pPOMDP = pomdp;		
	}
	
	public void load( String sFileName ) throws IOException, InvalidModelFileFormatException{
		m_pPOMDP.load( sFileName );
	}
	
	public MDPValueFunction getMDPValueFunction(){
		return m_pPOMDP.getMDPValueFunction();
	}
	
	public boolean useClassicBackup(){
		return m_pPOMDP.useClassicBackup();
	}
	
	protected void initStoredRewards(){
		m_pPOMDP.initStoredRewards();
	}
	
	public double tr( int iState1, int iAction, int iState2 ){
		return m_pPOMDP.tr( iState1, iAction, iState2 );
	}
	
	/**
	 * Immediate reward function of the POMDP. Checks reward type (R(s,a,s'),R(s,a) or R(s)) before accessing the reward function. 
	 * @param iStartState
	 * @param iAction
	 * @param iEndState
	 * @return immediate reward
	 */
	public double R( int iStartState, int iAction, int iEndState ){
		return m_pPOMDP.R( iStartState, iAction, iEndState );
	}
	
	/**
	 * Immediate reward function of the POMDP.
	 * @param iStartState
	 * @return immediate reward
	 */
	public double R( int iStartState ){
		return m_pPOMDP.R( iStartState );
	}
	
	/**
	 * Immediate reward function of the POMDP of the form R(s,a). If reward structure is of the form R(s,a,s') it sums over all possible s'. 
	 * @param iStartState
	 * @param iAction
	 * @return immediate reward
	 */
	public double R( int iStartState, int iAction ){
		return m_pPOMDP.R( iStartState, iAction );
	}
	
	public double O( int iAction, int iState, int iObservation ){
		return m_pPOMDP.O( iAction, iState, iObservation );
	}
	
	public double O( int iStartState, int iAction, int iEndState, int iObservation ){
		return m_pPOMDP.O( iStartState, iAction, iEndState, iObservation );
	}
	
	public String getActionName( int iAction ){
		return m_pPOMDP.getActionName( iAction );
	}
	public int getActionIndex( String sAction ){
		return m_pPOMDP.getActionIndex( sAction );
	}
	
	public String getStateName( int iState ){
		return m_pPOMDP.getStateName( iState );
	}
	
	public int getStateIndex( String sState ){
		return m_pPOMDP.getStateIndex( sState );
	}
	public int getObservationIndex( String sObservation ){
		return m_pPOMDP.getObservationIndex( sObservation );
	}
	
	public int execute( int iAction, int iState ){
		return m_pPOMDP.execute( iAction, iState );
	}
	
	public int observe( int iAction, int iState ){
		return m_pPOMDP.observe( iAction, iState );
	}
	public int observe( BeliefState bs, int iAction ){
		return m_pPOMDP.observe( bs, iAction );
	}
	
	public double computeAverageDiscountedReward( int cTests, int cMaxStepsToGoal, PolicyStrategy policy, boolean bOutputMessages, boolean bUseMultiThread ){
		return m_pPOMDP.computeAverageDiscountedReward( cTests, cMaxStepsToGoal, policy, bOutputMessages, bUseMultiThread );
	}
	
	protected double computeMDPAverageDiscountedReward( int cTests, int cMaxStepsToGoal ){
		return m_pPOMDP.computeMDPAverageDiscountedReward( cTests, cMaxStepsToGoal );
	}

	public int chooseStartState(){
		return m_pPOMDP.chooseStartState();
	}
	
	public boolean isTerminalState( int iState ){
		return m_pPOMDP.isTerminalState( iState );
	}
	
	protected double maxReward( int iState ){
		return m_pPOMDP.maxReward( iState );
	}

	public boolean terminalStatesDefined(){
		return m_pPOMDP.terminalStatesDefined();
	}

	public int getStateCount() {
		return m_pPOMDP.getStateCount();
	}

	public int getActionCount() {
		return m_pPOMDP.getActionCount();
	}

	public int getObservationCount() {
		return m_pPOMDP.getObservationCount();
	}

	public double getDiscountFactor() {
		return m_pPOMDP.getDiscountFactor();
	}
 
	public Iterator<Entry<Integer,Double>> getNonZeroTransitions( int iStartState, int iAction ) {
		return m_pPOMDP.getNonZeroTransitions( iStartState, iAction );
	}

	public Collection<Entry<Integer,Double>> getNonZeroBackwardTransitions( int iAction, int iEndState ) {
		return m_pPOMDP.getNonZeroBackwardTransitions( iAction, iEndState );
	}

	public double probStartState( int iState ){
		return m_pPOMDP.probStartState( iState );
	}
	
	public double getMinR(){
		return m_pPOMDP.getMinR();
	}
	
	public double getMaxR(){
		return m_pPOMDP.getMaxR();
	}
	
	public double getMaxMinR(){
		return m_pPOMDP.getMaxMinR();
	}

	public int getStartStateCount() {
		return m_pPOMDP.getStartStateCount();
	}

	public Iterator<Entry<Integer, Double>> getStartStates() {
		return m_pPOMDP.getStartStates();
	}

	public void initRandomGenerator( long iSeed ){
		m_rndGenerator.init( iSeed );		
	}

	public void setRandomSeed( long iSeed ){
		m_iRandomSeed = iSeed;		
	}

	public AlphaVector newAlphaVector() {
		return m_pPOMDP.newAlphaVector();
	}
	public boolean isValid( int iState ){
		return m_pPOMDP.isValid( iState );
	}
	
	public Collection<Integer> getValidStates(){
		return m_pPOMDP.getValidStates();
	}

	public boolean isFactored() {
		return m_pPOMDP.isFactored();
	}
	
	public String getName(){
		return m_pPOMDP.getName();
	}
	
	/**
	 * Computes the immediate reward for a belief state over all actions
	 * @param bs
	 * @return
	 */
	public double immediateReward( BeliefState bs ){
		return m_pPOMDP.immediateReward( bs );
	}

	
	/**
	 * Computes the immediate reward for a belief state and a specific action
	 * @param bs - belief state
	 * @param iAction - action index
	 * @return
	 */
	public double immediateReward( BeliefState bs, int iAction ){
		return m_pPOMDP.immediateReward( bs, iAction );
	}

	public Vector<Integer> getObservationRelevantStates() {
		return m_pPOMDP.getObservationRelevantStates();
	}

	public RewardType getRewardType() {
		return m_pPOMDP.getRewardType();
	}

	public void initBeliefStateFactory() {
		m_pPOMDP.initBeliefStateFactory();		
	}

	public double R( BeliefState bsCurrent, int iAction, BeliefState bsNext ){
		return m_pPOMDP.R( bsCurrent, iAction, bsNext );
	}

	public Collection<Integer> getRelevantActions( BeliefState bs ) {
		return m_pPOMDP.getRelevantActions( bs );
	}


}
