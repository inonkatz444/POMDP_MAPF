package pomdp.algorithms.pointbased;

import java.util.Iterator;
import java.util.Vector;

import pomdp.algorithms.ValueIteration;
import pomdp.environments.POMDP;
import pomdp.utilities.AlphaVector;
import pomdp.utilities.BeliefState;
import pomdp.utilities.BeliefStateFactory;
import pomdp.utilities.ExecutionProperties;
import pomdp.utilities.JProf;
import pomdp.utilities.Pair;
import pomdp.utilities.concurrent.Backup;
import pomdp.utilities.concurrent.ComputeFarthestSuccessors;
import pomdp.utilities.concurrent.ThreadPool;
import pomdp.valuefunction.LinearValueFunctionApproximation;

public class PointBasedValueIteration extends ValueIteration {

	protected Iterator m_itCurrentIterationPoints;
	protected boolean m_bSingleValueFunction = true;
	protected boolean m_bRandomizedActions;
	
	public PointBasedValueIteration( POMDP pomdp ){
		super( pomdp );
		
		//initValueFunctionUsingQMDP();
		
		m_itCurrentIterationPoints = null;
		m_bRandomizedActions = false;
	}

	public PointBasedValueIteration( POMDP pomdp, boolean bRadnomizedActionExpansion ){
		super( pomdp );
		
		//initValueFunctionUsingQMDP();
		
		m_itCurrentIterationPoints = null;
		m_bRandomizedActions = bRadnomizedActionExpansion;
	}

	protected Vector<BeliefState> expand( Vector<BeliefState> vBeliefPoints ){
		Vector vExpanded = new Vector( vBeliefPoints );
		Iterator it = vBeliefPoints.iterator();
		BeliefState bsCurrent = null, bsNext = null;

		boolean bPrevious = m_pPOMDP.getBeliefStateFactory().cacheBeliefStates( false );
				
		while( it.hasNext() && !m_bTerminate ){
			bsCurrent = (BeliefState) it.next();
			if( m_bRandomizedActions )
				bsNext = m_pPOMDP.getBeliefStateFactory().computeRandomFarthestSuccessor( vBeliefPoints, bsCurrent );
			else
				bsNext = m_pPOMDP.getBeliefStateFactory().computeFarthestSuccessor( vBeliefPoints, bsCurrent );
			if( ( bsNext != null ) && ( !vExpanded.contains( bsNext ) ) ){
				vExpanded.add( bsNext );
				//System.out.println( bsNext );
			}
		}
		
		m_pPOMDP.getBeliefStateFactory().cacheBeliefStates( bPrevious );
		
		return vExpanded;
	}
	
	protected Vector<BeliefState> expandMultiThread( Vector<BeliefState> vBeliefPoints ){
		Vector<BeliefState> vExpanded = new Vector<BeliefState>( vBeliefPoints );
		Vector<BeliefState> vSuccessors = null;
		int iThread = 0, cThreads = ExecutionProperties.getHighLevelThreadCount();
		ComputeFarthestSuccessors[] abThreads = new ComputeFarthestSuccessors[cThreads]; 
			
		boolean bPrevious = m_pPOMDP.getBeliefStateFactory().cacheBeliefStates( false );
		
		for( iThread = 0 ; iThread < cThreads ; iThread++ ){
			abThreads[iThread] = new ComputeFarthestSuccessors( vBeliefPoints );
			abThreads[iThread].setPOMDP( m_pPOMDP );
		}
		
		iThread = 0;
		for( BeliefState bs : vBeliefPoints ){
			abThreads[iThread].addBelief( bs );
			iThread = ( iThread + 1 ) % cThreads;
		}
		
		for( iThread = 0 ; iThread < cThreads ; iThread++ ){
			ThreadPool.getInstance().addTask( abThreads[iThread] );
		}
		for( iThread = 0 ; iThread < cThreads ; iThread++ ){
			ThreadPool.getInstance().waitForTask( abThreads[iThread] );
		}

		
		for( iThread = 0 ; iThread < cThreads ; iThread++ ){
			vSuccessors = abThreads[iThread].getSuccessors();
			for( BeliefState bs : vSuccessors ){
				if( !vExpanded.contains( bs ) ){
					vExpanded.add( bs );
					//System.out.println( bs );
				}
			}
		}
			
		m_pPOMDP.getBeliefStateFactory().cacheBeliefStates( bPrevious );
		
		return vExpanded;
	}
	
	public void valueIteration( int cIterations, double dEpsilon, double dTargetValue ){
		int iIteration = 0;
		Pair pComputedADRs = new Pair(0.0, 0.0);
		boolean bDone = false, bDoneInternal = false;
		
		long lStartTime = System.currentTimeMillis(), lCurrentTime = 0;
		Runtime rtRuntime = Runtime.getRuntime();
		
		int cInternalIterations = 10, iInternalIteration = 0;
		double dDelta = 1.0, dMinDelta = 0.01;
		int cBeliefPoints = 0;
		
		m_cElapsedExecutionTime = 0;
		m_cCPUExecutionTime = 0;
		
		long lCPUTimeBefore = 0, lCPUTimeAfter = 0, lCPUTimeTotal = 0;
		
		int cValueFunctionChanges = 0;

		System.out.println( "Begin " + getName() );
		
		cIterations = 200;
		
		
		Vector<BeliefState>	vBeliefPoints = new Vector<BeliefState>();
		vBeliefPoints.add( m_pPOMDP.getBeliefStateFactory().getInitialBeliefState() );
		

		for( iIteration = 0 ; iIteration < cIterations && !bDone  && !m_bTerminate ; iIteration++ ){
			lStartTime = System.currentTimeMillis();
			lCPUTimeBefore = JProf.getCurrentThreadCpuTimeSafe();
			
			if( iIteration > 0 ){
				System.out.println( "Expanding belief space" );
				m_dFilteredADR = 0.0;
				cBeliefPoints = vBeliefPoints.size();
				if( ExecutionProperties.useHighLevelMultiThread() )
					vBeliefPoints = expandMultiThread( vBeliefPoints );
				else
					vBeliefPoints = expand( vBeliefPoints );
				System.out.println( "Expanded belief space - |B| = " + vBeliefPoints.size() );
				if( vBeliefPoints.size() == cBeliefPoints )
					bDone = true;
			}
			
			dDelta = 1.0;
			bDoneInternal = false;
			for( iInternalIteration = 0 ; 
				( iInternalIteration < cInternalIterations ) && ( dDelta > dMinDelta ) && !bDoneInternal ; iInternalIteration++ ){
				
				cValueFunctionChanges = m_vValueFunction.getChangesCount();
				if( ExecutionProperties.useHighLevelMultiThread() )
					dDelta = improveValueFunctionMultiThreaded( vBeliefPoints );
				else
					dDelta = improveValueFunction( vBeliefPoints );
				
				lCurrentTime = System.currentTimeMillis();
				lCPUTimeAfter = JProf.getCurrentThreadCpuTimeSafe();
				m_cElapsedExecutionTime += ( lCurrentTime - lStartTime );
				m_cCPUExecutionTime += ( lCPUTimeAfter - lCPUTimeBefore ) / 1000000;
				lCPUTimeTotal += lCPUTimeAfter - lCPUTimeBefore;
				
				if( dDelta < dEpsilon && cValueFunctionChanges == m_vValueFunction.getChangesCount() ){
					System.out.println( "Value function did not change - iteration " + iIteration + " complete" );
					bDoneInternal = true;
				}
				else{
					if( iIteration > 2 ){
						bDone = bDone || checkADRConvergence( m_pPOMDP, dTargetValue, pComputedADRs );
						if( bDone )
							bDoneInternal = true;
					}
					
					rtRuntime.gc();
					System.out.println( "PBVI: Iteration " + iIteration + "," + iInternalIteration +
							" |Vn| = " + m_vValueFunction.size() +
							" |B| = " + vBeliefPoints.size() +
							" Delta = " + round( dDelta, 4 ) +
							" simulated ADR " + ((Number) pComputedADRs.first()).doubleValue() +
							" filtered ADR " + round( ((Number) pComputedADRs.second()).doubleValue(), 3 ) +
							" Time " + ( lCurrentTime - lStartTime ) / 1000 +
							" CPU time " + ( lCPUTimeAfter - lCPUTimeBefore ) / 1000000000 +
							" CPU total " + lCPUTimeTotal  / 1000000000 +
							" #backups " + m_cBackups + 
							" #dot product " + AlphaVector.dotProductCount() + 
							" |BS| " + m_pPOMDP.getBeliefStateFactory().getBeliefStateCount() +
							" memory: " + 
							" total " + rtRuntime.totalMemory() / 1000000 +
							" free " + rtRuntime.freeMemory() / 1000000 +
							" max " + rtRuntime.maxMemory() / 1000000 +
							"" );
				}
				
				m_cElapsedExecutionTime += ( lCurrentTime - lStartTime );
				
				lStartTime = System.currentTimeMillis();;
				lCPUTimeBefore = JProf.getCurrentThreadCpuTimeSafe();
			}
		}
		m_cElapsedExecutionTime /= 1000;
		m_cCPUExecutionTime /= 1000;

		System.out.println( "Finished " + getName() + " - time : " + m_cElapsedExecutionTime + " |BS| = " + vBeliefPoints.size() +
				" |V| = " + m_vValueFunction.size() + " backups = " + m_cBackups + " GComputations = " + AlphaVector.getGComputationsCount() );
	}

	protected double improveValueFunction( Vector vBeliefPoints ){
		LinearValueFunctionApproximation vNextValueFunction = new LinearValueFunctionApproximation( m_dEpsilon, true );
		BeliefState bsCurrent = null, bsMax = null;
		AlphaVector avBackup = null, avNext = null, avCurrentMax = null;
		double dMaxDelta = 1.0, dDelta = 0.0, dBackupValue = 0.0, dValue = 0.0;
		double dMaxOldValue = 0.0, dMaxNewValue = 0.0;
		int iBeliefState = 0;

		boolean bPrevious = m_pPOMDP.getBeliefStateFactory().cacheBeliefStates( false );
		
		if( m_itCurrentIterationPoints == null )
			m_itCurrentIterationPoints = vBeliefPoints.iterator();
		dMaxDelta = 0.0;
		while( m_itCurrentIterationPoints.hasNext() ){
			bsCurrent= (BeliefState) m_itCurrentIterationPoints.next();
			avCurrentMax = m_vValueFunction.getMaxAlpha( bsCurrent );
			avBackup = backup( bsCurrent );
			dBackupValue = avBackup.dotProduct( bsCurrent );
			dValue = avCurrentMax.dotProduct( bsCurrent );
			dDelta = dBackupValue - dValue;
			/*
			System.out.println( iBeliefState + ") " + bsCurrent + ", current max = " + avCurrentMax.getId() +
					", backup " + avBackup.getId() + ", v = " + round( dValue, 3 ) + 
					", backup value " + round( dBackupValue, 3 ) );
			*/
			
			if( dDelta > dMaxDelta ){
				dMaxDelta = dDelta;
				bsMax = bsCurrent;
				dMaxOldValue = dValue;
				dMaxNewValue = dBackupValue;
			}
			avNext = avBackup;
			/*
			if( dDelta > 0 )
				avNext = avBackup;
			else{
				if( vNextValueFunction.contains( avCurrentMax ) )
					avNext = null;
				else{
					avNext = avCurrentMax;
					avNext.setWitness( bsCurrent );
				}
			}
				*/
			//vNextValueFunction.add( avNext );
			if( avNext != null )
				//vNextValueFunction.add( avNext, true );
				vNextValueFunction.addPrunePointwiseDominated( avNext );
			
			iBeliefState++;
		}
		if( m_bSingleValueFunction ){
			Iterator it = vNextValueFunction.iterator();
			while( it.hasNext() ){
				avNext = (AlphaVector) it.next();
				m_vValueFunction.addPrunePointwiseDominated( avNext );
			}
		}
		else{
			m_vValueFunction.copy( vNextValueFunction );
		}
		
		
		if( !m_itCurrentIterationPoints.hasNext() )
			m_itCurrentIterationPoints = null;
		
		System.out.println( "Max delta over " + bsMax + 
				" from " + round( dMaxOldValue, 3 ) + 
				" to " + round( dMaxNewValue, 3 ) );
		
		m_pPOMDP.getBeliefStateFactory().cacheBeliefStates( bPrevious );
		
		return dMaxDelta;
	}
	
	protected double improveValueFunctionMultiThreaded( Vector<BeliefState> vBeliefPoints ){
		LinearValueFunctionApproximation vNextValueFunction = new LinearValueFunctionApproximation( m_dEpsilon, true );
		BeliefState bsCurrent = null, bsMax = null;
		AlphaVector avBackup = null, avNext = null, avCurrentMax = null;
		double dMaxDelta = 1.0, dDelta = 0.0, dBackupValue = 0.0, dValue = 0.0;
		double dMaxOldValue = 0.0, dMaxNewValue = 0.0;
		int iBeliefState = 0;
		int iThread = 0, cThreads = ExecutionProperties.getHighLevelThreadCount();
		Backup[] abThreads = new Backup[cThreads]; 
		int iVector = 0, cVectors = 0;
		
		boolean bPrevious = m_pPOMDP.getBeliefStateFactory().cacheBeliefStates( false );
		
		Iterator<BeliefState> itCurrentIterationPoints = vBeliefPoints.iterator();
		dMaxDelta = 0.0;
		
		for( iThread = 0 ; iThread < cThreads ; iThread++ ){
			abThreads[iThread] = new Backup( m_pPOMDP, m_vValueFunction );
		}
		
		iThread = 0;
		while( itCurrentIterationPoints.hasNext() ){
			bsCurrent= itCurrentIterationPoints.next();
			abThreads[iThread].addBelief( bsCurrent );
			iThread = ( iThread + 1 ) % cThreads;
		}
		
		for( iThread = 0 ; iThread < cThreads ; iThread++ ){
			ThreadPool.getInstance().addTask( abThreads[iThread] );
		}
		for( iThread = 0 ; iThread < cThreads ; iThread++ ){
			ThreadPool.getInstance().waitForTask( abThreads[iThread] );
		}

		
		for( iThread = 0 ; iThread < cThreads ; iThread++ ){
			cVectors = abThreads[iThread].getResultsCount();
			for( iVector = 0 ; iVector < cVectors ; iVector++ ){
				bsCurrent = abThreads[iThread].getBeliefState( iVector );
				avBackup = abThreads[iThread].getResult( iVector );
				m_cBackups++;
				dBackupValue = avBackup.dotProduct( bsCurrent );
				dValue = m_vValueFunction.valueAt( bsCurrent );
				dDelta = dBackupValue - dValue;
				if( dDelta > dMaxDelta ){
					dMaxDelta = dDelta;
					bsMax = bsCurrent;
					dMaxOldValue = dValue;
					dMaxNewValue = dBackupValue;
				}
				vNextValueFunction.addPrunePointwiseDominated( avBackup );
			}
		}
		
		if( m_bSingleValueFunction ){
			Iterator it = vNextValueFunction.iterator();
			while( it.hasNext() ){
				avNext = (AlphaVector) it.next();
				m_vValueFunction.addPrunePointwiseDominated( avNext );
			}
		}
		else{
			m_vValueFunction.copy( vNextValueFunction );
		}
		/*
		System.out.println( "Max delta over " + bsMax + 
				" from " + round( dMaxOldValue, 3 ) + 
				" to " + round( dMaxNewValue, 3 ) );
		*/
		m_pPOMDP.getBeliefStateFactory().cacheBeliefStates( bPrevious );
		
		return dMaxDelta;
	}
	
	public String getName(){
		return "PBVI";
	}
}
