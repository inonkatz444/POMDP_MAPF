package pomdp.utilities.skyline;

import java.util.Collection;
import java.util.Vector;

import pomdp.utilities.AlphaVector;
import pomdp.utilities.Logger;
import pomdp.utilities.datastructures.LinkedList;
import pomdp.valuefunction.LinearValueFunctionApproximation;

public class SkylinePruning {
	private LinearValueFunctionApproximation m_vValueFunction;
	private LinkedList<EquationMatrixCreator> m_lOpenList;
	private Vector<int[]> m_vObservedNodes;
	private Vector<double[]> m_vObservedBeliefs;
	private int m_cStates;
	private int m_cProcessed;
	
	public SkylinePruning( LinearValueFunctionApproximation vValueFunction, int cStates ){
		Logger.getInstance().log( "SkylinePruning", 0, "<init>", "Started initializing skyline" );
		m_vValueFunction = vValueFunction;
		m_lOpenList = new LinkedList<EquationMatrixCreator>();
		m_cStates = cStates;
		m_vObservedNodes = new Vector<int[]>();
		m_vObservedBeliefs = new Vector<double[]>();
		
		for( AlphaVector av : m_vValueFunction.getVectors() ){
			av.setDominated( true );
		}		
		
		m_cProcessed = 0;
		
		Equation.m_cEquations = 0;
		
		Logger.getInstance().log( "SkylinePruning", 0, "<init>", "Done initializing skyline" );
	}
	
	public void runSkylineWitness(){
		Logger.getInstance().log( "SkylinePruning", 0, "runSkylineWitness", "Started running skyline" );
		EquationMatrix emCurrent = null, emPrevious = null;;
		int iCurrentVectorVariable = -1;
		try{
			EquationMatrix.m_cMatrixes = 0;
			EquationMatrix.m_cFinalized = 0;
			int cDominated = 0, cNonDomiated = 0, cIterations = 0;
			int cSteps = 0, cSumDominatedSteps = 0, cSumNonDominatedSteps = 0;
			double dDelta = 0.0;
			for( AlphaVector av : m_vValueFunction.getVectors() ){
				if( av.isDominated() ){
					cSteps = 0;
					cIterations++;
					emCurrent = new EquationMatrix( m_vValueFunction.getVectors(), m_cStates, 0 );
					//System.out.println( emCurrent );
					iCurrentVectorVariable = emCurrent.getVariable( av );
					m_vObservedNodes.add( emCurrent.getIntersectingFunctions() );
					m_vObservedBeliefs.add( emCurrent.getBeliefState() );
					while( emCurrent != null ){
						m_cProcessed++;
						dDelta = emCurrent.valueOf( iCurrentVectorVariable );
						emPrevious = emCurrent;
						emCurrent = emCurrent.nextVertex( iCurrentVectorVariable );
						cSteps++;
						//double dValue = emCurrent.valueOf( iCurrentVectorVariable );
						if( m_cProcessed % 1000 == 0 )
							Logger.getInstance().logFull( "SkylinePruning", 0, "runSkylineWitness", 
									"Done " + m_cProcessed + " vertexes. |L|=" + m_lOpenList.size() + 
									" #M=" + EquationMatrix.m_cMatrixes + 
									" #F=" + EquationMatrix.m_cFinalized + 
									" |O|=" + m_vObservedNodes.size() + 
									//" Interior = " + cInterior + ", Exterior = " + cExterior +
									//" |Zmin| = " + cMinZeroBeleifStates 
									"" );
					}
					if( dDelta > 0.0 ){
						m_vValueFunction.remove( av );
						cSumDominatedSteps += cSteps;
						cDominated++;
					}
					else{
						av.setDominated( false );
						cSumNonDominatedSteps += cSteps;
						cNonDomiated++;
					}
				}
			}
			
			if( cDominated == 0 )
				cDominated = 1;
			if( cNonDomiated == 0 )
				cNonDomiated = 1;
			
			Logger.getInstance().logFull( 
					"SkylinePruning", 0, "runSkylineWitness", 
					"Done " + m_cProcessed + " vertexes. " + 
					" iters " + cIterations +
					" avg dom=" + cSumDominatedSteps / cDominated +
					" avg non-dom=" + cSumNonDominatedSteps / cNonDomiated +
					"" );
		}
		catch( Error e ){
			e.printStackTrace();
		}
		
	}
	
	public void runWitness(){
		Logger.getInstance().log( "SkylinePruning", 0, "runWitness", "Started running skyline" );
		EquationMatrix emCurrent = null;
		int iCurrentVectorVariable = -1;
		try{
			EquationMatrix.m_cMatrixes = 0;
			EquationMatrix.m_cFinalized = 0;
			int cDominated = 0, cNonDomiated = 0, cIterations = 0;
			int cSteps = 0, cSumDominatedSteps = 0, cSumNonDominatedSteps = 0, iState = 0, iDeterministicState = -1;
			double dDelta = 0.0;
			double dMaxDifference = 0.0, dDifference = 0.0;
			for( AlphaVector av : m_vValueFunction.getVectors() ){
				if( true ){
					dMaxDifference = 0.0;
					for( AlphaVector avTag : m_vValueFunction.getVectors() ){
						if( av != avTag ){
							for( iState = 0 ; iState < m_cStates ; iState++ ){
								dDifference = av.valueAt( iState ) - avTag.valueAt( iState );
								//dDifference = avTag.valueAt( iState ) - av.valueAt( iState );
								if( dDifference > dMaxDifference ){
									dMaxDifference = dDifference;
									iDeterministicState = iState;
								}
							}
						}
					}	
					//dMaxDifference = 0.0;
					cSteps = 0;
					cIterations++;
					emCurrent = new EquationMatrix( m_vValueFunction.getVectors(), m_cStates, iDeterministicState, av, dMaxDifference );
					//System.out.println( emCurrent );
					iCurrentVectorVariable = emCurrent.getVariable( av );
					m_vObservedNodes.add( emCurrent.getIntersectingFunctions() );
					m_vObservedBeliefs.add( emCurrent.getBeliefState() );
					while( emCurrent != null ){
						m_cProcessed++;
						dDelta = emCurrent.valueOf( iCurrentVectorVariable );
						emCurrent = emCurrent.nextVertex( iCurrentVectorVariable );
						cSteps++;
						//double dValue = emCurrent.valueOf( iCurrentVectorVariable );
						if( m_cProcessed % 1000 == 0 )
							Logger.getInstance().logFull( "SkylinePruning", 0, "runWitness", 
									"Done " + m_cProcessed + " vertexes. |L|=" + m_lOpenList.size() + 
									" #M=" + EquationMatrix.m_cMatrixes + 
									" #F=" + EquationMatrix.m_cFinalized + 
									" |O|=" + m_vObservedNodes.size() + 
									//" Interior = " + cInterior + ", Exterior = " + cExterior +
									//" |Zmin| = " + cMinZeroBeleifStates 
									"" );
					}
					if( dDelta > dMaxDifference ){
						//m_vValueFunction.remove( av );
						av.setDominated( true );
						cSumDominatedSteps += cSteps;
						cDominated++;
					}
					else{
						av.setDominated( false );
						cSumNonDominatedSteps += cSteps;
						cNonDomiated++;
					}
				}
			}
			
			if( cDominated == 0 )
				cDominated = 1;
			if( cNonDomiated == 0 )
				cNonDomiated = 1;
			
			Logger.getInstance().logFull( 
					"SkylinePruning", 0, "runWitness", 
					"Done " + m_cProcessed + " vertexes. " + 
					" iters " + cIterations +
					" avg dom=" + cSumDominatedSteps / cDominated +
					" avg non-dom=" + cSumNonDominatedSteps / cNonDomiated +
					"" );
		}
		catch( Error e ){
			e.printStackTrace();
		}
		
	}
	
	public double[] findWitness( Collection<AlphaVector> cVectors, AlphaVector avSearchVector ){
		EquationMatrix emCurrent = null;
		int iCurrentVectorVariable = -1;

		int cSteps = 0, iState = 0, iDeterministicState = -1;
		double dDelta = 0.0;
		double dMaxDifference = 0.0, dDifference = 0.0;
		double[] adBeliefState = null;
		dMaxDifference = 0.0;
		for( AlphaVector avTag : cVectors ){
			if( avSearchVector != avTag ){
				for( iState = 0 ; iState < m_cStates ; iState++ ){
					dDifference = avSearchVector.valueAt( iState ) - avTag.valueAt( iState );
					if( dDifference > dMaxDifference ){
						dMaxDifference = dDifference;
						iDeterministicState = iState;
					}
				}
			}
		}	
		cSteps = 0;
		emCurrent = new EquationMatrix( cVectors, m_cStates, iDeterministicState, avSearchVector, dMaxDifference );
		//System.out.println( emCurrent );
		iCurrentVectorVariable = emCurrent.getVariable( avSearchVector );
		m_vObservedNodes.add( emCurrent.getIntersectingFunctions() );
		m_vObservedBeliefs.add( emCurrent.getBeliefState() );
		while( emCurrent != null ){
			dDelta = emCurrent.valueOf( iCurrentVectorVariable );
			adBeliefState = emCurrent.getBeliefState();
			emCurrent = emCurrent.nextVertex( iCurrentVectorVariable );
			m_cProcessed++;
			cSteps++;
		}
		if( dDelta > dMaxDifference ){
			return null;
		}
		else{
			return adBeliefState;
		}
	}
	
	public void run(){
		Logger.getInstance().log( "SkylinePruning", 0, "run", "Started running skyline" );
		EquationMatrixCreator emcCurrent = null;
		EquationMatrix emCurrent = null;
		LinkedList<EquationMatrix> lStartMatrixes = new LinkedList<EquationMatrix>();
		int cProcessed = 0, cInterior = 0, cExterior = 0, cMinZeroBeleifStates = 10000, iState = 0;
		try{
			EquationMatrix.m_cMatrixes = 0;
			EquationMatrix.m_cFinalized = 0;
			for( iState = 0 ; iState < m_cStates ; iState++ ){
				emCurrent = new EquationMatrix( m_vValueFunction.getVectors(), m_cStates, iState );
				m_vObservedNodes.add( emCurrent.getIntersectingFunctions() );
				m_vObservedBeliefs.add( emCurrent.getBeliefState() );
				lStartMatrixes.add( emCurrent );
				cProcessed++;
			}
			for( EquationMatrix em : lStartMatrixes ){
				for( EquationMatrixCreator emc : em.lazyNeighbors( m_vObservedNodes, m_vObservedBeliefs ) )
					m_lOpenList.addSorted( emc, EquationMatrixCreatorComparator.getInstance() );				
			}
			while( m_lOpenList.size() > 0 ){
				emcCurrent = m_lOpenList.removeFirst();
				if( emcCurrent.getZeroBeliefStates() < cMinZeroBeleifStates )
					cMinZeroBeleifStates = emcCurrent.getZeroBeliefStates();
				emCurrent = emcCurrent.getMatrix();
				if( emCurrent != null ){
					if( emCurrent.isInteriorPoint() )
						cInterior++;
					else
						cExterior++;
					cProcessed++;
					for( EquationMatrixCreator emcNeighbor : emCurrent.lazyNeighbors( m_vObservedNodes, m_vObservedBeliefs ) ){
						m_lOpenList.addSorted( emcNeighbor, EquationMatrixCreatorComparator.getInstance() );
					}
					if( cProcessed % 1000 == 0 )
						Logger.getInstance().logFull( "SkylinePruning", 0, "run", 
								"Done " + cProcessed + " vertexes. |L|=" + m_lOpenList.size() + 
								" #M=" + EquationMatrix.m_cMatrixes + 
								" #F=" + EquationMatrix.m_cFinalized + 
								" |O|=" + m_vObservedNodes.size() + 
								//" Interior = " + cInterior + ", Exterior = " + cExterior +
								//" |Zmin| = " + cMinZeroBeleifStates 
								"" );
				}
			}
			int cSumVertexes = 0, cMaxVertexes = 0, cVectors = 0;
			for( AlphaVector av : m_vValueFunction.getVectors() ){
				if( !av.isDominated() ){
					cVectors++;
					cSumVertexes += av.getWitnesses().size();
					if( av.getWitnesses().size()  > cMaxVertexes ){
						cMaxVertexes = av.getWitnesses().size();
					}
				}
			}
			
			Logger.getInstance().logFull( "SkylinePruning", 0, "run", 
					"Done " + cProcessed + " vertexes. " + 
					" #M=" + EquationMatrix.m_cMatrixes + 
					" |O|=" + m_vObservedNodes.size() + 
					" avg(|W|)=" + cSumVertexes / ( 1.0 * cVectors ) +
					" max(|W|)=" + cMaxVertexes +
					"");
		}
		catch(Error e){
			e.printStackTrace();
		}
	}
	
	public int getProcessedCount(){
		return Equation.m_cEquations;
	}
}
