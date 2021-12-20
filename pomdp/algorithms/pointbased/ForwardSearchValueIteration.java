package pomdp.algorithms.pointbased;

import java.util.Iterator;
import java.util.Map;
import java.util.SortedMap;
import java.util.TreeMap;
import java.util.Vector;
import java.util.Map.Entry;

import pomdp.algorithms.ValueIteration;
import pomdp.environments.DeterministicPOMDP;
import pomdp.environments.FactoredPOMDP;
import pomdp.environments.POMDP;
import pomdp.environments.FactoredPOMDP.BeliefType;
import pomdp.utilities.AlphaVector;
import pomdp.utilities.BeliefState;
import pomdp.utilities.BeliefStateFactory;
import pomdp.utilities.ExecutionProperties;
import pomdp.utilities.HeuristicPolicy;
import pomdp.utilities.JProf;
import pomdp.utilities.LimitedBeliefSpaceMDP;
import pomdp.utilities.Logger;
import pomdp.utilities.MDPValueFunction;
import pomdp.utilities.ObsevationAwareMDPValueFunction;
import pomdp.utilities.Pair;
import pomdp.utilities.TabularAlphaVector;
import pomdp.utilities.factored.AlgebraicDecisionDiagramVertex;
import pomdp.utilities.factored.FactoredBeliefState;
import pomdp.utilities.factored.FactoredBeliefStateComparator;
import pomdp.valuefunction.LinearValueFunctionApproximation;

public class ForwardSearchValueIteration extends ValueIteration {
	protected ObsevationAwareMDPValueFunction m_pObservationAwareMDP;
	protected LimitedBeliefSpaceMDP m_pLimitedBeliefMDP;
	protected int m_iLimitedBeliefMDPState;
	protected int m_iLimitedBeliefObservation;
	protected DeterministicPOMDP m_pDetPOMDP;
	protected LinearValueFunctionApproximation m_vDetermisticPOMDPValueFunction;
	protected BeliefState m_bsDeterministicPOMDPBeliefState;
	protected HeuristicPolicy m_hpPolicy;
	protected int m_iDepth;
	protected int m_iIteration, m_iInnerIteration;
	protected long m_lLatestADRCheck, m_cTimeInADR, m_lCPUTimeTotal, m_lIterationStartTime;
	protected Pair m_pComputedADRs;
	protected int[] m_aiStartStates;
	protected SortedMap<Double,Integer>[][] m_amNextStates;
	protected Map<BeliefState,Integer> m_mCountUselessUpdates;
	protected Map<BeliefState,Integer> m_mCountLatestUselessUpdates;
	protected Map<BeliefState,Integer> m_mCountUsefullUpdates;
	//protected Vector<BeliefState> m_vLabeledBeliefs;
	private static boolean g_bCountUselessBackups = false;
	private HeuristicType m_htType;
	public static HeuristicType DEFAULT_HEURISTIC = HeuristicType.MDP;
	
	public enum HeuristicType{
		MDP, ObservationAwareMDP, DeterministicTransitionsPOMDP, DeterministicObservationsPOMDP, DeterministicPOMDP, LimitedBeliefMDP, HeuristicPolicy
	}
		
	public ForwardSearchValueIteration( POMDP pomdp ){
		this( pomdp, DEFAULT_HEURISTIC );
	}

	public ForwardSearchValueIteration( POMDP pomdp, HeuristicPolicy hpPolicy ){
		//this( pomdp, HeuristicType.DeterministicObservationsPOMDP );
		//this( pomdp, HeuristicType.LimitedBeliefMDP );
		this( pomdp, HeuristicType.MDP );
		m_hpPolicy = hpPolicy;
	}

	public ForwardSearchValueIteration( POMDP pomdp, HeuristicType htType ){
		super( pomdp );
		m_htType = htType;
		m_iDepth = 0;
		m_iIteration = 0;
		m_iInnerIteration = 0;
		m_lLatestADRCheck = 0;
		m_cTimeInADR = 0;
		m_lCPUTimeTotal = 0;
		m_lIterationStartTime = 0;
		m_pComputedADRs = null;
		m_aiStartStates = null;
		if( g_bCountUselessBackups ){
			m_mCountUselessUpdates = new TreeMap<BeliefState,Integer>( FactoredBeliefStateComparator.getInstance() );
			m_mCountLatestUselessUpdates = new TreeMap<BeliefState,Integer>( FactoredBeliefStateComparator.getInstance() );
			m_mCountUsefullUpdates = new TreeMap<BeliefState,Integer>( FactoredBeliefStateComparator.getInstance() );
		}
		m_vfMDP = null;
		m_pObservationAwareMDP = null;
		m_bsDeterministicPOMDPBeliefState = null;
		m_vDetermisticPOMDPValueFunction = null;
		m_pDetPOMDP = null;
		m_iLimitedBeliefObservation = -1;
		initHeuristic();
		//m_vLabeledBeliefs = new Vector<BeliefState>();
	}

	private void initHeuristic(){
		long lBefore = JProf.getCurrentThreadCpuTimeSafe(), lAfter = 0;
		if( m_htType == HeuristicType.MDP ){
			m_vfMDP = m_pPOMDP.getMDPValueFunction();
			m_vfMDP.valueIteration( 1000, ExecutionProperties.getEpsilon() );
		}
		else if( m_htType == HeuristicType.ObservationAwareMDP ){
			m_pObservationAwareMDP = new ObsevationAwareMDPValueFunction( m_pPOMDP, 0.0 );
			m_pObservationAwareMDP.valueIteration( 1000, 0.1 );
		}
		else if( m_htType == HeuristicType.LimitedBeliefMDP ){
			m_pLimitedBeliefMDP = new LimitedBeliefSpaceMDP( m_pPOMDP );
			m_pLimitedBeliefMDP.valueIteration( 1000, 0.1 );
			m_iLimitedBeliefMDPState = -1;
		}
		else if( m_htType == HeuristicType.DeterministicTransitionsPOMDP ){
			m_pDetPOMDP = new DeterministicPOMDP( m_pPOMDP, true, false );
		} 
		else if( m_htType == HeuristicType.DeterministicObservationsPOMDP ){
			m_pDetPOMDP = new DeterministicPOMDP( m_pPOMDP, false, true );
		}
		else if( m_htType == HeuristicType.DeterministicPOMDP ){
			m_pDetPOMDP = new DeterministicPOMDP( m_pPOMDP, true, true );
		}
		if( m_pDetPOMDP != null ){
			HeuristicSearchValueIteration hsvi = new HeuristicSearchValueIteration( m_pDetPOMDP );
			hsvi.valueIteration( 10, 1E-4 );
			//ForwardSearchValueIteration fsvi = new ForwardSearchValueIteration( m_pDetPOMDP, HeuristicType.MDP );
			//fsvi.valueIteration( null, 5, 0.01 );
			m_vDetermisticPOMDPValueFunction = hsvi.getValueFunction();
		}
		lAfter = JProf.getCurrentThreadCpuTimeSafe();
		Logger.getInstance().log( "FSVI", 0, "initHeurisitc", "Initialization time was " + ( lAfter - lBefore ) / 1000000 );
	}
	
	public void valueIteration( int cMaxSteps, double dEpsilon, double dTargetValue ){
		int iIteration = 0;
		boolean bDone = false;
		Pair pComputedADRs = new Pair();
		double dMaxDelta = 0.0;
		String sMsg = "";
		
		long lStartTime = System.currentTimeMillis(), lCurrentTime = 0;
		long lCPUTimeBefore = 0, lCPUTimeAfter = 0;
		Runtime rtRuntime = Runtime.getRuntime();
		
		long cDotProducts = AlphaVector.dotProductCount(), cVnChanges = 0, cStepsWithoutChanges = 0;
		m_cElapsedExecutionTime = 0;
		m_lCPUTimeTotal = 0;
		
		sMsg = "Starting " + getName() + " target ADR = " + round( dTargetValue, 3 );
        Logger.getInstance().log( "FSVI", 0, "VI", sMsg );

		//initStartStateArray();
		m_pComputedADRs = new Pair();
		
		for( iIteration = 0 ; ( iIteration < cMaxSteps ) && !bDone && !m_bTerminate ; iIteration++ ){
			lStartTime = System.currentTimeMillis();
			lCPUTimeBefore = JProf.getCurrentThreadCpuTimeSafe();
			AlphaVector.initCurrentDotProductCount();
			cVnChanges = m_vValueFunction.getChangesCount();
			m_iIteration = iIteration;
			m_iInnerIteration = 0;
			m_lLatestADRCheck = lCPUTimeBefore;
			m_cTimeInADR = 0;
			m_lIterationStartTime = lCPUTimeBefore;
			dMaxDelta = improveValueFunction();
			lCPUTimeAfter = JProf.getCurrentThreadCpuTimeSafe();
			lCurrentTime = System.currentTimeMillis();
			m_cElapsedExecutionTime += ( lCurrentTime - lStartTime - m_cTimeInADR );
			m_cCPUExecutionTime += ( lCPUTimeAfter - lCPUTimeBefore - m_cTimeInADR ) / 1000000;
			m_lCPUTimeTotal += lCPUTimeAfter - lCPUTimeBefore - m_cTimeInADR;
/*
			if( ( m_lCPUTimeTotal  / 1000000000 ) >= 1000 )
				bDone = true;
	*/		
			if( m_bTerminate )
				bDone = true;
			
			if( false ){
				/*
				int cFakeVectors = 500;
				for( int i = 0 ; i < cFakeVectors ; i++ ){
					AlphaVector av = new TabularAlphaVector( null, 0.0, m_pPOMDP );
					for( int j = 0 ; j < m_cStates ; j++ ){
						av.setValue( j, m_rndGenerator.nextDouble( 2.0 ) );
					}
					av.finalizeValues();
					m_vValueFunction.add( av );
				}
				*/
				
				
				LinearValueFunctionApproximation vCopy = new LinearValueFunctionApproximation( m_vValueFunction );
				System.out.println( "LP pruning" );
				vCopy.pruneLP( m_pPOMDP );
				//m_vValueFunction.pruneRandomSampling( m_pPOMDP, 100 );
				//m_vValueFunction.pruneTrials( m_pPOMDP, 10, 100, this );
				boolean bPruned = m_vValueFunction.pruneSkyline( m_pPOMDP );
				if( false ){
					//System.out.println( "*" );
					for( AlphaVector av : vCopy.getVectors() ){
						//System.out.println( av );
						if( !m_vValueFunction.contains( av ) ){
							System.out.println( av );
							/*
							for( double[] adBelief : av.getWitnesses() ){
								double dMax = 0.0, dValue = 0.0;
								AlphaVector avMax = null;
								for( AlphaVector avTag : m_vValueFunction.getVectors() ){
									dValue = avTag.dotProduct( adBelief );
									if( dValue > dMax ){
										dMax = dValue;
										avMax = avTag;
									}
								}
								System.out.println( "Max is " + avMax + ", " + dMax + " instead of " + av.dotProduct( adBelief ) );
								
							}
							*/
						}
					}
					for( AlphaVector av : vCopy.getVectors() ){
						//System.out.println( av );
					}
					vCopy.pruneLP( m_pPOMDP );
				}
				//System.exit( 0 );
				
			}
			
			if( ExecutionProperties.getReportOperationTime() ){
				try{
					sMsg = "G: - operations " + AlphaVector.getGComputationsCount() + " avg time " + 
							AlphaVector.getAvgGTime();
			        Logger.getInstance().log( "FSVI", 0, "VI", sMsg );

					if( m_pPOMDP.isFactored() && ((FactoredPOMDP) m_pPOMDP).getBeliefType() == BeliefType.Factored ){
						sMsg = "Tau: - operations " + FactoredBeliefState.getTauComputationCount() + " avg time " + 
								FactoredBeliefState.getAvgTauTime();
				        Logger.getInstance().log( "FSVI", 0, "VI", sMsg );

					}
					else{
						sMsg = "Tau: - operations " + m_pPOMDP.getBeliefStateFactory().getTauComputationCount() + " avg time " + 
								m_pPOMDP.getBeliefStateFactory().getAvgTauTime();
				        Logger.getInstance().log( "FSVI", 0, "VI", sMsg );
					}
					sMsg = "dot product - avg time = " + AlphaVector.getCurrentDotProductAvgTime();
			        Logger.getInstance().log( "FSVI", 0, "VI", sMsg );
					sMsg = "avg belief state size " + m_pPOMDP.getBeliefStateFactory().getAvgBeliefStateSize();
			        Logger.getInstance().log( "FSVI", 0, "VI", sMsg );
					sMsg = "avg alpha vector size " + m_vValueFunction.getAvgAlphaVectorSize();
			        Logger.getInstance().log( "FSVI", 0, "VI", sMsg );
					AlphaVector.initCurrentDotProductCount();
				}
				catch( Exception e ){
					System.out.println( e );
				}
			}
			if( ( ( m_lCPUTimeTotal  / 1000000000 ) >= 10 ) && ( iIteration >= 100 ) && ( iIteration % 10 == 0 ) && 
					m_vValueFunction.getChangesCount() > cVnChanges &&
					m_vValueFunction.size() > 5 ){
				
				
				
				cStepsWithoutChanges = 0;
				bDone |= checkADRConvergence( m_pPOMDP, dTargetValue, pComputedADRs );
		        
		        sMsg = "FSVI: Iteration " + iIteration + 
						" |Vn| = " + m_vValueFunction.size() +
						" simulated ADR " + round( ((Number) pComputedADRs.first()).doubleValue(), 3 ) +
						" filtered ADR " + round( ((Number) pComputedADRs.second()).doubleValue(), 3 ) +
						" max delta " + round( dMaxDelta, 3 ) +
						" depth " + m_iDepth +
						" V(b0) " + round( m_vValueFunction.valueAt( m_pPOMDP.getBeliefStateFactory().getInitialBeliefState() ), 2 ) +
						" time " + 	( lCurrentTime - lStartTime ) / 1000 +
						" CPU time " + ( lCPUTimeAfter - lCPUTimeBefore - m_cTimeInADR ) / 1000000000 +
						" CPU total " + m_lCPUTimeTotal  / 1000000000 +
						" #backups " + m_cBackups + 
						" V changes " + m_vValueFunction.getChangesCount() +
						" #dot product " + AlphaVector.dotProductCount() + 
						" |BS| " + m_pPOMDP.getBeliefStateFactory().getBeliefStateCount() +
						" memory: " + 
						" total " + rtRuntime.totalMemory() / 1000000 +
						" free " + rtRuntime.freeMemory() / 1000000 +
						" max " + rtRuntime.maxMemory() / 1000000 +
						"";
		        Logger.getInstance().log( "FSVI", 0, "VI", sMsg );
				
			}
			else{
				if( cVnChanges == m_vValueFunction.getChangesCount() ){
					cStepsWithoutChanges++;
					//if( cStepsWithoutChanges == 10 ){
					//	bDone = true;
					//}
				}
				sMsg = "FSVI: Iteration " + iIteration + 
						" |Vn| = " + m_vValueFunction.size() +
						" time " + 	( lCurrentTime - lStartTime ) / 1000 +
						" V changes " + m_vValueFunction.getChangesCount() +
						" max delta " + round( dMaxDelta, 3 ) +
						" depth " + m_iDepth +
						" V(b0) " + round( m_vValueFunction.valueAt( m_pPOMDP.getBeliefStateFactory().getInitialBeliefState() ), 2 ) +
						" CPU time " + ( lCPUTimeAfter - lCPUTimeBefore ) / 1000000000 +
						" CPU total " + m_lCPUTimeTotal  / 1000000000 +
						" #backups " + m_cBackups + 
						" |BS| " + m_pPOMDP.getBeliefStateFactory().getBeliefStateCount() +
						" memory: " + 
						" total " + rtRuntime.totalMemory() / 1000000 +
						" free " + rtRuntime.freeMemory() / 1000000 +
						" max " + rtRuntime.maxMemory() / 1000000 +
						"";
		        Logger.getInstance().log( "FSVI", 0, "VI", sMsg );

			
			}

		}	
		m_bConverged = true;

		m_cDotProducts = AlphaVector.dotProductCount() - cDotProducts;
		m_cElapsedExecutionTime /= 1000;
		m_cCPUExecutionTime /= 1000;
		
		sMsg = "Finished " + getName() + " - time : " + m_cElapsedExecutionTime + /*" |BS| = " + vBeliefPoints.size() +*/
				" |V| = " + m_vValueFunction.size() + 
				" backups = " + m_cBackups + 
				" GComputations = " + AlphaVector.getGComputationsCount() +
				" Dot products = " + m_cDotProducts;
        Logger.getInstance().log( "FSVI", 0, "VI", sMsg );

		if( g_bCountUselessBackups )
			writeUselessBackupsStatistics();
	}
	
	public String getName(){
		return "FSVI";
	}
	
	protected void writeUselessBackupsStatistics(){
		int cUselessUpdates = 0, cLatestUselessUpdates = 0, cUsefulBackups = 0;
		int cTotalUselessUpdates = 0, cTotalLatestUselessUpdates = 0;
		System.out.println();
		for( Entry<BeliefState,Integer> eCurrent : m_mCountUselessUpdates.entrySet() ){
			cUselessUpdates = eCurrent.getValue();
			if( m_mCountLatestUselessUpdates.containsKey( eCurrent.getKey() ) )
				cLatestUselessUpdates = m_mCountLatestUselessUpdates.get( eCurrent.getKey() );
			else
				cLatestUselessUpdates = 0;
			
			if( m_mCountUsefullUpdates.containsKey( eCurrent.getKey() ) )
				cUsefulBackups = m_mCountUsefullUpdates.get( eCurrent.getKey() );
			else
				cUsefulBackups = 0;
			System.out.println( "Useful backups " + cUsefulBackups + ", Useless updates " + cUselessUpdates + ", after convergence " + cLatestUselessUpdates );
			cTotalUselessUpdates += cUselessUpdates;
			cTotalLatestUselessUpdates += cLatestUselessUpdates;

		}
		System.out.println( "Total useless updates " + cTotalUselessUpdates + ", after convergence " + cTotalLatestUselessUpdates );
		System.out.println( "Average useless updates " + ( cTotalUselessUpdates / (double)m_mCountLatestUselessUpdates.size() ) + 
				", after convergence " + ( cTotalLatestUselessUpdates / (double)m_mCountLatestUselessUpdates.size() ) );
		
		
	}
	
	protected double forwardSearch( int iState, BeliefState bsCurrent, int iDepth ){
		double dDelta = 0.0, dNextDelta = 0.0;
		int iNextState = 0, iHeuristicAction = 0, iPOMDPAction = 0, iObservation = 0;
		BeliefState bsNext = null;
		AlphaVector avBackup = null, avMax = null;
		double dPreviousValue = 0.0, dNewValue = 0.0;
		
		if( m_bTerminate )
			return 0.0;
		
		if( ( m_pPOMDP.terminalStatesDefined() && isTerminalState( iState ) ) 
				||	( iDepth >= 100 ) ){
			m_iDepth = iDepth;
			System.out.println( "Ended at depth " + iDepth + ". isTerminalState(" + iState + ")=" + isTerminalState( iState ) );
		}
		else{
			iHeuristicAction = getAction( iState, bsCurrent, iDepth );
			/*
			iPOMDPAction = m_vValueFunction.getBestAction( bsCurrent );
			
			if( iMDPAction == iPOMDPAction )
				iMDPAction = m_rndGenerator.nextInt( m_cActions );
			*/
			iNextState = selectNextState( iState, iHeuristicAction );
			iObservation = getObservation( iState, iHeuristicAction, iNextState );
			bsNext = bsCurrent.nextBeliefState( iHeuristicAction, iObservation );
			
			
			//if( bsNext.equals( bsCurrent ) )
			//	System.out.println( "*" );
			
/*
			System.out.println( iDepth + ") s = " + m_pPOMDP.getStateName( iState ) + " pr(s|b)=" + bsCurrent.valueAt( iState ) +
					", a = " + m_pPOMDP.getActionName( iHeuristicAction ) +
					", o = " + iObservation + 
					", current = " + bsCurrent +
					", next = " + bsNext
					);
	*/	
			if( bsNext == null /*|| bsNext.equals( bsCurrent )*/ ){
				//bsNext = bsCurrent.nextBeliefState( iHeuristicAction, iObservation );
				System.out.println( "Ended at depth " + iDepth + " due to an error" );
				m_iDepth = iDepth;
			}
			else{

			double d = bsNext.valueAt( iNextState );
			if( bsNext.valueAt( iNextState ) == 0.0 ){
				System.out.println( bsCurrent + ", " + m_pPOMDP.getStateName( iState ) + ", pr = " + bsCurrent.valueAt( iState ) );
				System.out.println( bsNext + ", " + m_pPOMDP.getStateName( iNextState ) + ", pr = " + bsNext.valueAt( iNextState ) );
				bsNext = bsCurrent.nextBeliefState( iHeuristicAction, iObservation );
			}
			/*
				int iUnderlyingState = iState;//m_pObservationAwareMDP.getUnderlyingState( iState );
				System.out.println( iDepth + ") s = " + m_pPOMDP.getStateName( iUnderlyingState ) + 
						", a = " + m_pPOMDP.getActionName( iMDPAction ) +
						", o = " + iObservation + 
						", current = " + bsCurrent +
						", next = " + bsNext
						);
*/	
				dNextDelta = forwardSearch( iNextState, bsNext, iDepth + 1 );
			}
		}

		if( false ){ 
			BeliefState bsDeterministic = getDeterministicBeliefState( iState );
			/*
			for( int i = 0 ; i < m_cStates ; i++ ){
				if( bsDeterministic.valueAt(i) > 0.0 )
					System.out.println( "s = " + m_pPOMDP.getStateName(i) + ", pr = " + bsDeterministic.valueAt(i) );
			}
			*/
			
			avBackup = backup( bsDeterministic, iHeuristicAction );
			dPreviousValue = m_vValueFunction.valueAt( bsDeterministic );
			dNewValue = avBackup.dotProduct( bsDeterministic );
			dDelta = dNewValue - dPreviousValue;
			
			//System.out.println( iDepth + ") deterministic delta = " + round( dDelta, 3 ) + ", " + avBackup + " " + bsDeterministic );
			
			if( dDelta > ExecutionProperties.getEpsilon() ){
				m_vValueFunction.addPrunePointwiseDominated( avBackup );
				//m_vValueFunction.addBounded( avBackup, 100 );
				//if( m_vLabeledBeliefs.contains( bsDeterministic ) )
				//	System.out.println( "*" );
			}
			if( dDelta <= 0.0 ){
				///if( !m_vLabeledBeliefs.contains( bsDeterministic ) )
				//	m_vLabeledBeliefs.add( bsDeterministic );
			}
		}
		avBackup = backup( bsCurrent );
/*		
		AlphaVector avTmp = backup( bsCurrent, 6 );
		double d = avTmp.dotProduct( bsCurrent );
	*/	
		
		dPreviousValue = m_vValueFunction.valueAt( bsCurrent );
		dNewValue = avBackup.dotProduct( bsCurrent );
		dDelta = dNewValue - dPreviousValue;
		avMax = m_vValueFunction.getMaxAlpha( bsCurrent );
		
		//System.out.println( iDepth + ") " + " v_old " + round( dPreviousValue, 3 ) + " v_new " + round( dNewValue, 3 ) +
		//		" delta = " + round( dDelta, 3 ) + ", " + avBackup + " " + bsCurrent );

		if( dDelta > 0.0 ){
			m_vValueFunction.addPrunePointwiseDominated( avBackup );
			
			//if( m_vLabeledBeliefs.contains( bsCurrent ) )
			//	System.out.println( "*" );
			
			//m_vValueFunction.addBounded( avBackup, 100 );
			if( g_bCountUselessBackups ){
				m_mCountLatestUselessUpdates.put( bsCurrent, 0 );
				if( m_mCountUsefullUpdates.containsKey( bsCurrent ) ){
					int cUsefulBackups = m_mCountUsefullUpdates.get( bsCurrent );
					m_mCountUsefullUpdates.put( bsCurrent, cUsefulBackups + 1 );
				}
				else{
					m_mCountUsefullUpdates.put( bsCurrent, 1 );
				}
			}
		}
		else{
			if( dDelta <= 0.0 ){
				//if( !m_vLabeledBeliefs.contains( bsCurrent ) )
				//	m_vLabeledBeliefs.add( bsCurrent );
			}
			avBackup.release();
			if( g_bCountUselessBackups ){
				if( m_mCountUselessUpdates.containsKey( bsCurrent ) ){
					int cUselessUpdates = m_mCountUselessUpdates.get( bsCurrent );
					m_mCountUselessUpdates.put( bsCurrent, cUselessUpdates + 1 );
					cUselessUpdates = m_mCountLatestUselessUpdates.get( bsCurrent );
					m_mCountLatestUselessUpdates.put( bsCurrent, cUselessUpdates + 1 );
				}
				else{
					m_mCountUselessUpdates.put( bsCurrent, 1 );
					m_mCountLatestUselessUpdates.put( bsCurrent, 1 );			
				}
			}
		}		
		
		return Math.max( dDelta, dNextDelta );
	}

	private boolean isTerminalState( int iState ){
		if( m_htType == HeuristicType.ObservationAwareMDP ){
			int iUnderlyingState = m_pObservationAwareMDP.getUnderlyingState( iState );
			return m_pPOMDP.isTerminalState( iUnderlyingState );
		}
		return m_pPOMDP.isTerminalState( iState );
	}
	private BeliefState getDeterministicBeliefState( int iState ){
		if( m_htType == HeuristicType.ObservationAwareMDP ){
			int iUnderlyingState = m_pObservationAwareMDP.getUnderlyingState( iState );
			return m_pPOMDP.getBeliefStateFactory().getDeterministicBeliefState( iUnderlyingState );
		}
		return m_pPOMDP.getBeliefStateFactory().getDeterministicBeliefState( iState );
	}
	private int getAction( int iState, BeliefState bs, int iDepth ){
		double dExloprationFactor = 0.9;
		if( iDepth > 100 )
			dExloprationFactor = 0.9;
		if( m_htType == HeuristicType.MDP ){
			if( m_rndGenerator.nextDouble() < dExloprationFactor )
				return m_vfMDP.getAction( iState );
			return m_rndGenerator.nextInt( m_cActions );
		}
		else if( m_htType == HeuristicType.ObservationAwareMDP ){
			return m_pObservationAwareMDP.getAction( iState );
		}
		else if( ( m_htType == HeuristicType.DeterministicTransitionsPOMDP )
					|| ( m_htType == HeuristicType.DeterministicObservationsPOMDP )
					|| ( m_htType == HeuristicType.DeterministicPOMDP ) ){
			return m_vDetermisticPOMDPValueFunction.getBestAction( m_bsDeterministicPOMDPBeliefState );
		}
		else if( m_htType == HeuristicType.LimitedBeliefMDP ){
			int iAction = m_pLimitedBeliefMDP.getAction( m_iLimitedBeliefMDPState );
			return iAction;
		}
		else if( m_htType == HeuristicType.HeuristicPolicy ){
			return m_hpPolicy.getBestAction( iState, bs );
		}
		return -1;
	}
	private int getObservation( int iStartState, int iAction, int iEndState ){
		if( m_htType == HeuristicType.MDP ){
			return m_pPOMDP.observe( iAction, iEndState );
		}
		else if( m_htType == HeuristicType.ObservationAwareMDP ){
			return m_pObservationAwareMDP.observe( iAction, iEndState );
		}
		else if( ( m_htType == HeuristicType.DeterministicTransitionsPOMDP )
				|| ( m_htType == HeuristicType.DeterministicObservationsPOMDP )
				|| ( m_htType == HeuristicType.DeterministicPOMDP ) ){
			int iObservation = m_pDetPOMDP.observe( iAction, iEndState );
			m_bsDeterministicPOMDPBeliefState = m_bsDeterministicPOMDPBeliefState.nextBeliefState( iAction, iObservation );
			return iObservation;
		}
		else if( m_htType == HeuristicType.LimitedBeliefMDP ){
			//System.out.println( iEndState + ", " + m_pPOMDP.getActionName( iAction ) + ", " + iObservation + " => " + m_pLimitedBeliefMDP.getStateName( m_iLimitedBeliefMDPState ) );
			return m_iLimitedBeliefObservation;
		}
		else if( m_htType == HeuristicType.HeuristicPolicy ){
			int iObservation = m_hpPolicy.getObservation( iStartState, iAction, iEndState );
			if( iObservation == -1 )
				return m_pPOMDP.observe( iAction, iEndState );
			return iObservation;
		}
		return -1;
	}
	private int selectNextState( int iState, int iAction ) {
		if( m_htType == HeuristicType.MDP ){
			return m_pPOMDP.execute( iAction, iState );
		}
		else if( m_htType == HeuristicType.LimitedBeliefMDP ){
			m_iLimitedBeliefObservation = m_pLimitedBeliefMDP.observe( m_iLimitedBeliefMDPState, iAction );
			m_iLimitedBeliefMDPState = m_pLimitedBeliefMDP.getSuccessor( m_iLimitedBeliefMDPState, iAction, m_iLimitedBeliefObservation );
			int iRealState = m_pLimitedBeliefMDP.sampleRealState( m_iLimitedBeliefMDPState );
			//System.out.println( iRealState + ", " + m_pLimitedBeliefMDP.getStateName( m_iLimitedBeliefMDPState ) );
			return iRealState;
		}
		else if( m_htType == HeuristicType.ObservationAwareMDP ){
			return m_pObservationAwareMDP.execute( iAction, iState );
		}
		else if( ( m_htType == HeuristicType.DeterministicTransitionsPOMDP )
				|| ( m_htType == HeuristicType.DeterministicObservationsPOMDP )
				|| ( m_htType == HeuristicType.DeterministicPOMDP ) ){
			return m_pDetPOMDP.execute( iAction, iState );
		}
		else if( m_htType == HeuristicType.HeuristicPolicy ){
			int iNextState = m_hpPolicy.getNextState( iState, iAction );
			if( iNextState == -1 )
				return m_pPOMDP.execute( iAction, iState );
		}
		return -1;
		
		/* does not work - does not visit enough places??
		int iNextState = -1, iMaxNext = -1;
		double dTr = 0.0, dValue = 0.0, dMaxValue = Double.NEGATIVE_INFINITY;
		Iterator itNonZero = m_pPOMDP.getNonZeroTransitions( iState, iAction );
		Map.Entry e = null;
		while( itNonZero.hasNext() ){
			e = (Entry) itNonZero.next();
			iNextState = ((Number)e.getKey()).intValue();
			if( iNextState != iState ){
				dTr = ((Number)e.getValue()).doubleValue();
				dValue = m_vfMDP.getValue( iNextState );
				if( dValue > dMaxValue ){
					dMaxValue = dValue;
					iMaxNext = iNextState;
				}
			}
		}
		return iMaxNext;
		*/
	}

	private void removeNextState( int iState, int iAction, int iNextState ){
		m_amNextStates[iState][iAction].remove( m_amNextStates[iState][iAction].lastKey() );
	}

	protected int getNextState( int iAction, int iState ){
		int iNextState = -1;
		double dTr = 0.0, dValue = 0.0;

		if( m_amNextStates[iState][iAction] == null )
			m_amNextStates[iState][iAction] = new TreeMap<Double,Integer>();
		if( m_amNextStates[iState][iAction].isEmpty() ){
			Iterator itNonZero = m_pPOMDP.getNonZeroTransitions( iState, iAction );
			Map.Entry e = null;
			String sDescription = "";
			while( itNonZero.hasNext() ){
				e = (Entry) itNonZero.next();
				iNextState = ((Number)e.getKey()).intValue();
				dTr = ((Number)e.getValue()).doubleValue();
				dValue = m_vfMDP.getValue( iNextState );
				sDescription += "V(" + iNextState + ") = " + round( dValue, 3 ) + ", ";
				m_amNextStates[iState][iAction].put( dValue, iNextState );
			}
			//System.out.println( "next " + iState + "," + iAction + " = [" + sDescription + "]" );
		}
		iNextState = m_amNextStates[iState][iAction].get( m_amNextStates[iState][iAction].lastKey() );
		return iNextState;
	}
	
	protected void initStartStateArray(){
		int cStates = m_pPOMDP.getStartStateCount(), iState = 0;
		Iterator<Entry<Integer,Double>> itStartStates = m_pPOMDP.getStartStates();
		Entry<Integer,Double> e = null;
		m_aiStartStates = new int[cStates];
		for( iState = 0 ; iState < cStates ; iState++ ){
			e = itStartStates.next();
			m_aiStartStates[iState] = e.getKey();
		}
		if( m_amNextStates == null ){
			m_amNextStates = new SortedMap[m_cStates][m_cActions];
		}
	}
	
	protected int chooseStartState(){
		int cStates = m_pPOMDP.getStartStateCount(), iState = 0, iMaxValueState = -1;
		double dValue = 0.0, dMaxValue = MIN_INF;
		for( iState = 0 ; iState < cStates ; iState++ ){
			if( m_aiStartStates[iState] != -1 ){
				dValue = m_vfMDP.getValue( iState );
				if( dValue > dMaxValue ){
					dMaxValue = dValue;
					iMaxValueState = iState;
				}
			}
		}
		if( iMaxValueState == -1 ){
			initStartStateArray();
			return chooseStartState();
		}
		iState = m_aiStartStates[iMaxValueState];
		m_aiStartStates[iMaxValueState] = -1;
		return iState;
	}
	
	
	protected double improveValueFunction(){
		//int iInitialState = chooseStartState();
		int iInitialState = -1;
		do{
			iInitialState = m_pPOMDP.chooseStartState();
		}while( m_pPOMDP.isTerminalState( iInitialState ) );
		if( m_htType == HeuristicType.ObservationAwareMDP ){
			iInitialState = m_pObservationAwareMDP.getIntialState( iInitialState );
		}
		BeliefState bsInitial = m_pPOMDP.getBeliefStateFactory().getInitialBeliefState();
		if( m_pDetPOMDP != null )
			m_bsDeterministicPOMDPBeliefState = m_pDetPOMDP.getBeliefStateFactory().getInitialBeliefState();
		if( m_pLimitedBeliefMDP != null ){
			m_iLimitedBeliefMDPState = 0;
			//System.out.println( iInitialState + " => " + m_pLimitedBeliefMDP.getStateName( m_iLimitedBeliefMDPState ) );
		}
		System.out.println( "Starting at state " + m_pPOMDP.getStateName( iInitialState ) );
		//BeliefState bsInitial = m_pPOMDP.getBeliefStateFactory().getDeterministicBeliefState( iInitialState );
		m_iDepth = 0;
		System.out.println( "Begin improve" );
		double dDelta = forwardSearch( iInitialState, bsInitial, 0 );
		System.out.println( "End improve, |V| = " + 
				m_vValueFunction.size() + ", delta = " + dDelta );
		return dDelta;
	}
	
}
