package pomdp.algorithms.gridbased;

import java.util.Iterator;
import java.util.Map;
import java.util.Vector;

import pomdp.CreateBeliefSpaces;
import pomdp.environments.POMDP;
import pomdp.utilities.BeliefState;
import pomdp.utilities.BeliefStateFactory;

public class FixedSetGrid extends VariableResolutionGrid{
	protected Vector m_vBeliefPoints;
	
	public FixedSetGrid( POMDP pomdp ){
		super( pomdp );
		m_vBeliefPoints = null;
		g_cTrials = 100;
	}

	protected Vector computeResolutionBeliefPoints( int iResolution ){
		Iterator itPoints = m_vBeliefPoints.iterator();
		BeliefState bsCurrent = null;
		Vector vEntries = new Vector();
		Map mEntries = null;
		double[] adDirection = new double[m_cStates];
		int[] aiBase = new int[m_cStates], aiSortedPermutation = null;
		Map[] amVertices = null;
		int iVertice = 0;
		
		while( itPoints.hasNext() ){
			bsCurrent = (BeliefState) itPoints.next();
			
			//System.out.println( bsCurrent );
			
			m_cInterpolations++;
			
			computeBaseAndDirection( bsCurrent.getNonZeroEntriesMap(), iResolution, aiBase, adDirection );		
			aiSortedPermutation = getSortedPermutation( adDirection );
			amVertices = getSubSimplexVertices( aiBase, aiSortedPermutation, iResolution );
			
			for( iVertice = 0 ; iVertice < amVertices.length ; iVertice++ ){
				mEntries = amVertices[iVertice];
				if( ( mEntries != null ) && !vEntries.contains( mEntries ) )
					vEntries.add( mEntries );
			}
		}
		
		return vEntries;
	}
	
	public void valueIteration( int cMaxSteps, double dEpsilon, double dTargetValue ){
		m_vBeliefPoints = CreateBeliefSpaces.createRandomSpace( m_pPOMDP, m_rndGenerator.nextInt(), 1000 );
		super.valueIteration( cMaxSteps, dEpsilon, dTargetValue );
	}

	protected Iterator getLowerBoundPointsIterator() {
		return randomPermutation( m_vBeliefPoints ) ;
	}
/*
	protected void refineGrid(){
		int iResolution = ( m_iFinestResolution == 0 ) ? 1 : m_iFinestResolution * 2;
		Vector vEntries = computeResolutionBeliefPoints( iResolution );
		Iterator itEntries = vEntries.iterator();
		Map mEntries = null;
		BeliefState bs = null;
		
		while( itEntries.hasNext() ){
			mEntries = (Map) itEntries.next();
			bs = m_pPOMDP.getBeliefStateFactory().getBeliefState( mEntries );
			addGridPoint( mEntries, bs, iResolution );
		}
		m_iFinestResolution = iResolution;
		System.out.println( "Done refining the grid |V^| = " + m_vGridPoints.size() );
	}	
*/	
	protected void refineGrid(){
		double dMaxError = MIN_INF; 
		double dSumError = 0.0;
		Iterator itBSIterator = m_vBeliefPoints.iterator();
		BeliefState bsCurrent = null, bsMaxError = null;
		double dInterpolateValue = 0.0, dHValue = 0.0, dError = 0.0;
		int iResolution = 0;
		
		System.out.println( "FSG: Refining grid" );
		
		if( m_iFinestResolution == 0 )
			super.refineGrid();
		else{
			while( itBSIterator.hasNext() ){
				bsCurrent = (BeliefState)itBSIterator.next();
				if( !bsCurrent.isDeterministic() && !m_vGridPoints.contains( bsCurrent ) ){
					dInterpolateValue = interpolateValue( bsCurrent.getNonZeroEntriesMap() );
					dHValue = applyH( bsCurrent );
					dError = dInterpolateValue - dHValue;
					//if( dError < 0.0 )
					//	System.out.println( "BUGBUG: refineGrid " + bsCurrent + " err = " + dError );
					if( dError > dMaxError ){
						dMaxError = dError;
						bsMaxError = bsCurrent;
					}
					dSumError += dError;
				}
			}
			
			iResolution = findMaxResoltionCompleteSubSimplex( bsMaxError );
			addGridPointSimplex( bsMaxError, iResolution * 2 );
			
			System.out.println( "Max error point is " + bsMaxError +
					" M = " + iResolution +
					" err = " + dMaxError +
					" |V^| = " + m_vGridPoints.size() );
		}
	}

	protected void addGridPointSimplex( BeliefState bsMaxError, int iResolution ){
		Map mEntries = null;
		double[] adDirection = new double[m_cStates];
		int[] aiBase = new int[m_cStates], aiSortedPermutation = null;
		Map[] amVertices = null;
		int iVertice = 0;
		BeliefState bsVertice = null;
		
		m_cInterpolations++;
			
		computeBaseAndDirection( bsMaxError.getNonZeroEntriesMap(), iResolution, aiBase, adDirection );		
		aiSortedPermutation = getSortedPermutation( adDirection );
		amVertices = getSubSimplexVertices( aiBase, aiSortedPermutation, iResolution );
			
		for( iVertice = 0 ; iVertice < amVertices.length ; iVertice++ ){
			mEntries = amVertices[iVertice];
			if( mEntries != null ){
				bsVertice = m_pPOMDP.getBeliefStateFactory().getBeliefState( mEntries );
				
				//if( bsMaxError.getId() == 198 ){
				//	System.out.println( "Adding for 198 " + bsVertice );
				//}
				
				addGridPoint( mEntries, bsVertice, iResolution );
			}
		}
	}
	

}
