package pomdp.utilities;

import pomdp.environments.WordComplete;


public class WordCompleteHeuristicPolicy extends HeuristicPolicy {
	private WordComplete m_wcPOMDP;
	
	public WordCompleteHeuristicPolicy( WordComplete wcPOMDP ){
		m_wcPOMDP = wcPOMDP;
	}
	public int getObservation( int iStartState, int iAction, int iEndState ){
		return m_wcPOMDP.observeForward( iStartState, iAction );
	}
	public int getBestAction( int iState, BeliefState bsBelief ){
		int iMostLikeliState = bsBelief.getMostLikeliState();
		int iStateInfo = m_wcPOMDP.getStateInfo( iMostLikeliState );
		int iItem = m_wcPOMDP.getItem( iStateInfo );
		return iItem;
	}
}
