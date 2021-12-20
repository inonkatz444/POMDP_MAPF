package pomdp.algorithms.pointbased;

import java.util.Vector;

import com.sun.org.apache.bcel.internal.generic.DLOAD;

import pomdp.environments.POMDP;
import pomdp.utilities.BeliefState;


public class PerfectInformationValueIteration extends HeuristicSearchValueIteration {

	public PerfectInformationValueIteration( POMDP pomdp ) {
		super(pomdp);
	}

	protected BeliefState getNextBeliefState( BeliefState bsCurrent, double dEpsilon, double dDiscount ){
		int iAction = getExplorationAction( bsCurrent );
		BeliefState bsSuccessor = getNextBeliefState( bsCurrent, iAction );
		if( bsSuccessor == null ){
			bsSuccessor = getNextBeliefState( bsCurrent, iAction );
		}
		return bsSuccessor;
	}

	protected BeliefState getNextBeliefState( BeliefState bsCurrent, int iCurrenBestAction ){
		BeliefState bsSuccessor = null;
		double dProbOGivenBandA = 0.0;
		double dBestActionValueWithoutSuccessor = 0.0, dBestActionSuccessorProb = 0.0;
		double dValueWithoutSuccessor = 0.0, dSuccessorProb = 0.0;
		int iObservation = 0, iAlternativeAction = 0, iAction = 0, iSuccessor = 0, iOtherSuccessor = 0;
		double dRegret = 0.0, dVPI = 0.0;
		double dMaxRegret = 0.0, dMaxVPI = 0.0;
		int iMaxRegretAction = -1;
		BeliefState bsMaxVPISucessor = null;
		
		Vector<BeliefState> vSuccessors = new Vector<BeliefState>();
		Vector<Double>[] avSuccessorProbs = (Vector<Double>[])new Vector[m_cActions];
		Vector<Double> vLowerBound = new Vector<Double>();
		Vector<Double> vUpperBound = new Vector<Double>();
		
		//caching all needed computations
		for( iAction = 0 ; iAction < m_cActions ; iAction++ ){
			avSuccessorProbs[iAction] = new Vector<Double>();
		}		
		
		for( iAction = 0 ; iAction < m_cActions ; iAction++ ){
			for( iObservation = 0 ; iObservation < m_cObservations ; iObservation++ ){
				dProbOGivenBandA = bsCurrent.probabilityOGivenA( iAction, iObservation );
				if( dProbOGivenBandA > 0 ){
					bsSuccessor = bsCurrent.nextBeliefState( iAction, iObservation );
					iSuccessor = vSuccessors.indexOf( bsSuccessor );
					if( iSuccessor == -1 ){
						vSuccessors.add( bsSuccessor );
						for( Vector<Double> v : avSuccessorProbs ){
							v.add( 0.0 );
						}
						vLowerBound.add( m_vValueFunction.valueAt( bsSuccessor ) );
						vUpperBound.add( m_vfUpperBound.valueAt( bsSuccessor ) );
						iSuccessor = vSuccessors.size() - 1;
					}
					dSuccessorProb = avSuccessorProbs[iAction].elementAt( iSuccessor );
					avSuccessorProbs[iAction].set( iSuccessor, dSuccessorProb + dProbOGivenBandA );
				}
			}
		}
		
		for( iSuccessor = 0 ; iSuccessor < vSuccessors.size() ; iSuccessor++ ){
			bsSuccessor = vSuccessors.elementAt( iSuccessor );
			dBestActionSuccessorProb = avSuccessorProbs[iCurrenBestAction].elementAt( iSuccessor );
			dBestActionValueWithoutSuccessor = m_pPOMDP.R( bsCurrent, iCurrenBestAction );
			for( iOtherSuccessor = 0 ; iOtherSuccessor < vSuccessors.size() ; iOtherSuccessor++ ){
				if( iSuccessor != iOtherSuccessor ){
					dBestActionValueWithoutSuccessor += avSuccessorProbs[iCurrenBestAction].elementAt( iOtherSuccessor ) * 
						vUpperBound.elementAt( iOtherSuccessor );
				}
			}
			
			dMaxRegret = 0.0;
			for( iAlternativeAction = 0 ; iAlternativeAction < m_cActions ; iAlternativeAction++ ){
				if( iAlternativeAction != iCurrenBestAction ){
					dValueWithoutSuccessor = m_pPOMDP.R( bsCurrent, iAlternativeAction );
					for( iOtherSuccessor = 0 ; iOtherSuccessor < vSuccessors.size() ; iOtherSuccessor++ ){
						if( iSuccessor != iOtherSuccessor ){
							dValueWithoutSuccessor += avSuccessorProbs[iAlternativeAction].elementAt( iOtherSuccessor ) * 
								vUpperBound.elementAt( iOtherSuccessor );
						}
					}
					dSuccessorProb = avSuccessorProbs[iAlternativeAction].elementAt( iSuccessor );
					
					dRegret = area( dBestActionValueWithoutSuccessor, dBestActionSuccessorProb, dValueWithoutSuccessor, dSuccessorProb,
							vLowerBound.elementAt( iSuccessor ), vUpperBound.elementAt( iSuccessor ) );
					double dGap = vUpperBound.elementAt( iSuccessor ) - vLowerBound.elementAt( iSuccessor );
					//dRegret = dGap;
					/*
					if( dRegret > 0.0 ){
						dRegret *= 100;
						dRegret += dGap;
					}
					*/
					/*
					if( dRegret < 0 )
						dRegret = area( dBestActionValueWithoutSuccessor, dBestActionSuccessorProb, dValueWithoutSuccessor, dSuccessorProb,
								vLowerBound.elementAt( iSuccessor ), vUpperBound.elementAt( iSuccessor ) );
						*/
					
					if( dRegret > dMaxRegret ){
						dMaxRegret = dRegret;
						iMaxRegretAction = iAlternativeAction;
					}
				}
				
			}
			dVPI = dMaxRegret;
			if( dVPI > dMaxVPI ){
				dMaxVPI = dVPI;
				bsMaxVPISucessor = bsSuccessor;
			}
			
		}
		if( bsMaxVPISucessor == bsCurrent )
			return null;
		return bsMaxVPISucessor;
	}
	/*
	protected BeliefState getNextBeliefState( BeliefState bsCurrent, int iCurrenBestAction ){
		BeliefState bsSuccessor = null;
		double dProbOGivenBandA = 0.0;
		double dBestActionValueWithoutSuccessor = 0.0, dBestActionSuccessorProb = 0.0;
		double dValueWithoutSuccessor = 0.0, dSuccessorProb = 0.0;
		int iObservation = 0, iAlternativeAction = 0, iAction = 0, iSuccessor = 0;
		double[][] adSuccessorProbs = new double[m_cActions][m_cObservations];
		BeliefState[][] abSuccessors = new BeliefState[m_cActions][m_cObservations];
		double[][] adSuccessorValuesLB = new double[m_cActions][m_cObservations]; // lower bound
		double[][] adSuccessorValuesUB = new double[m_cActions][m_cObservations]; // upper bound
		double dRegret = 0.0, dVPI = 0.0;
		double dMaxRegret = 0.0, dMaxVPI = 0.0;
		int iMaxRegretAction = -1;
		BeliefState bsMaxVPISucessor = null;
		
		//caching all needed computations
		for( iAction = 0 ; iAction < m_cActions ; iAction++ ){
			for( iObservation = 0 ; iObservation < m_cObservations ; iObservation++ ){
				adSuccessorProbs[iAction][iObservation] = bsCurrent.probabilityOGivenA( iAction, iObservation );
				if( adSuccessorProbs[iAction][iObservation] > 0 ){
					abSuccessors[iAction][iObservation] = bsCurrent.nextBeliefState( iAction, iObservation );
					if( abSuccessors[iAction][iObservation] == bsCurrent )
						adSuccessorProbs[iAction][iObservation] = 0.0;
					adSuccessorValuesLB[iAction][iObservation] = m_vValueFunction.valueAt( abSuccessors[iAction][iObservation] );
					adSuccessorValuesUB[iAction][iObservation] = m_vfUpperBound.valueAt( abSuccessors[iAction][iObservation] );
				}
			}
		}
		
		for( iObservation = 0 ; iObservation < m_cObservations ; iObservation++ ){
			dProbOGivenBandA = adSuccessorProbs[iCurrenBestAction][iObservation];
			if( dProbOGivenBandA > 0.0 ){
				bsSuccessor = abSuccessors[iCurrenBestAction][iObservation];
				dBestActionValueWithoutSuccessor = m_pPOMDP.R( bsCurrent, iCurrenBestAction );
				for( iSuccessor = 0 ; iSuccessor < m_cObservations ; iSuccessor++ ){
					if( iSuccessor != iObservation && abSuccessors[iCurrenBestAction][iSuccessor] != null ){
						dBestActionValueWithoutSuccessor += adSuccessorProbs[iCurrenBestAction][iSuccessor] * 
							adSuccessorValuesUB[iCurrenBestAction][iSuccessor];
					}
				}
				dBestActionSuccessorProb = adSuccessorProbs[iCurrenBestAction][iObservation];			
				dMaxRegret = 0.0;
				for( iAlternativeAction = 0 ; iAlternativeAction < m_cActions ; iAlternativeAction++ ){
					if( iAlternativeAction != iCurrenBestAction ){
						dValueWithoutSuccessor = m_pPOMDP.R( bsCurrent, iAlternativeAction );
						for( iSuccessor = 0 ; iSuccessor < m_cObservations ; iSuccessor++ ){
							if( iSuccessor != iObservation && abSuccessors[iAlternativeAction][iSuccessor] != null ){
								dValueWithoutSuccessor += adSuccessorProbs[iAlternativeAction][iSuccessor] * 
									adSuccessorValuesUB[iAlternativeAction][iSuccessor];
							}
						}
						dSuccessorProb = adSuccessorProbs[iAlternativeAction][iObservation];					
					}
					
					dRegret = area( dBestActionValueWithoutSuccessor, dBestActionSuccessorProb, dValueWithoutSuccessor, dSuccessorProb,
							adSuccessorValuesLB[iCurrenBestAction][iObservation], adSuccessorValuesUB[iCurrenBestAction][iObservation] );
					
					if( dRegret > dMaxRegret ){
						dMaxRegret = dRegret;
						iMaxRegretAction = iAlternativeAction;
					}
				}
				dVPI = dMaxRegret;
				if( dVPI > dMaxVPI ){
					dMaxVPI = dVPI;
					bsMaxVPISucessor = bsSuccessor;
				}
			}
		}
		if( bsMaxVPISucessor == bsCurrent )
			return null;
		return bsMaxVPISucessor;
	}

*/
	public String getName(){
		return "VPI";
	}
	
	private double area( double dBestActionValueWithoutSuccessor, double dBestActionSuccessorProb,
				double dValueWithoutSuccessor, double dSuccessorProb, double dSuccessorLowerBound, double dSuccessorUpperBound ){
		if( Math.abs( dBestActionSuccessorProb - dSuccessorProb ) < 0.0001 )
			return 0.0;
		double dCrossingPoint = ( dValueWithoutSuccessor - dBestActionValueWithoutSuccessor ) /
			( dBestActionSuccessorProb - dSuccessorProb );
		if( dCrossingPoint <= dSuccessorLowerBound ) //intersection beyond lower bound
			return 0.0;
		if( dCrossingPoint >= dSuccessorUpperBound )//intersection beyond upper bound
			return 0.0;
		double dArea = 0.0;
		double SuccessorLowerBoundForBestAction = dBestActionValueWithoutSuccessor + dBestActionSuccessorProb * dSuccessorLowerBound;
		double dSuccessorLowerBoundForAlternativeAction = dValueWithoutSuccessor + dSuccessorProb * dSuccessorLowerBound;
		double SuccessorUpperBoundForBestAction = dBestActionValueWithoutSuccessor + dBestActionSuccessorProb * dSuccessorUpperBound;
		double dSuccessorUpperBoundForAlternativeAction = dValueWithoutSuccessor + dSuccessorProb * dSuccessorUpperBound;
		if( SuccessorLowerBoundForBestAction < dSuccessorLowerBoundForAlternativeAction ){ //left side
			dArea = ( dSuccessorLowerBoundForAlternativeAction - SuccessorLowerBoundForBestAction ) * 
					( dCrossingPoint - dSuccessorLowerBound ) / 
					( 2  * ( dSuccessorUpperBound - dSuccessorLowerBound ) );
		}
		else{ //right side
			dArea = ( dSuccessorUpperBoundForAlternativeAction - SuccessorUpperBoundForBestAction ) * 
				( dSuccessorUpperBound - dCrossingPoint ) / 
				( 2  * ( dSuccessorUpperBound - dSuccessorLowerBound ) );
		}
		return dArea;
	}
	
	
	
	
	private class Alpha{
		public double d, c;
		public Alpha( double x, double y ){
			d = x;
			c = y;
		}
	}

	// Compute a set of "alpha" vectors given:
	//  cn: current MDP state (node)
	//  *next: the successor state we are determining whether to search
	//  *&best_alpha: the best alpha_vector to be returned (for the optimal action, however optimality is determined)
	//  maxUBAction: the optimal action (could be for expectation as in paper, or for UB, or even LB)
	//
	//Note that all alpha vectors consist of two numbers: a constant and a linear coefficient of the state whose // value we are letting vary between the lower and upper bounds.  So for each action, we need to compute // an alpha vector consisting of these two numbers.
	private Vector<Alpha> computeAlpha( BeliefState bsCurrent, BeliefState next, Alpha aBest, int maxUBAction) {

		// Reset counter for alpha vectors
		int alphaIndex = 0, iObservation = 0, iAction = 0;
		BeliefState bsNext = null;
		double dProb = 0.0;
		Vector<Alpha> vAlphas = new Vector<Alpha>();

		// For each action, compute an alpha vector
		for( iAction = 0 ; iAction < m_cActions ; iAction++ ) {
			Alpha alpha_tmp = new Alpha( 0.0, 0.0 );
			Alpha p_vec = null;

			for( iObservation = 0 ; iObservation < m_cObservations ; iObservation++ ) {
				bsNext = bsCurrent.nextBeliefState( iAction, iObservation );
				if( bsNext != null ){
					dProb = bsCurrent.probabilityOGivenA( iAction, iObservation );
					if( bsNext.equals( next ) )
						alpha_tmp.d += dProb;
					else
						alpha_tmp.c += dProb * ( m_vValueFunction.valueAt( bsNext ) + m_vfUpperBound.valueAt( bsNext ) ) * 0.5;
				}
			}
			alpha_tmp.d *= m_dGamma;
			alpha_tmp.c *= m_dGamma;
			alpha_tmp.c += m_pPOMDP.immediateReward( bsCurrent, iAction );

			vAlphas.add( alpha_tmp );

			// Keep track of the alpha-vector for the "optimal" action a* (from paper)
			if (maxUBAction == iAction) {
				aBest = alpha_tmp;
			}
		}
		return vAlphas;
	}

	//Given lower and upper bounds for state whose value we are letting vary and a 
	// vector alpha_comp for the current best action a*, compute the area between 
	// the upper bound of all alpha_vectors and alpha_comp.  We can do this by subtracting off 
	// alpha_comp from all alpha_vectors and then computing the area under the 
	// 1-dimensional convex hull formed by the upper bound of the alpha vectors.
	//
	double computeArea( Alpha alpha_comp, double lb, double ub, Vector<Alpha> vAlphas ) {

		double cur_left  = lb;
		Alpha  cur_alpha = new Alpha(0.0, 0.0);
		double next_left  = -1.0;
		Alpha  next_alpha = new Alpha(-1.0, -1.0);
		double cur_highest = 0.0;
		double area = 0.0;
		Alpha tmp_v = null;
		int i;

		// Normalize by subtracting off alpha_comp and keep track of best vector for LB
		for (i = 0; i < vAlphas.size() ; i++ ) {
			tmp_v = vAlphas.elementAt( i );
			tmp_v.c -= alpha_comp.c;
			tmp_v.d -= alpha_comp.d;

			double height = tmp_v.c + tmp_v.d*lb;
			if (height - cur_highest >= m_dEpsilon) {
				cur_highest = height;
				cur_alpha = tmp_v;
			}
		}

		// Find earliest intersection where current best alpha_vector becomes suboptimal then compute
		// area for piecewise portion before proceeding to next vector.  When reach UB or beyond, then done.
		while (cur_left != ub) {
			next_left = ub;
			next_alpha = cur_alpha;

			// Find highest intersection b/w cur_alpha & tmp_v above zero that is
			// greater than cur_left, but less than next_left (ub to start)
			for (i = 0; i < vAlphas.size(); i++ ) {
				tmp_v = vAlphas.elementAt( i );

				if (Math.abs(cur_alpha.d - tmp_v.d) <= m_dEpsilon) 
					continue;
				double intersection = (tmp_v.c - cur_alpha.c) / (cur_alpha.d - tmp_v.d);
				if (intersection <= (cur_left + m_dEpsilon) || intersection >= (next_left - m_dEpsilon))
					continue;

				next_left = intersection;
				next_alpha = tmp_v;
			}

			// Correct for potential numerical precision errors
			if (((next_alpha.c + next_alpha.d*next_left) < -m_dEpsilon)) {
				next_alpha.c = 0.0;
				next_alpha.d = 0.0;
			}

			area += ((cur_alpha.c + cur_alpha.d*cur_left) +
					(next_alpha.c + next_alpha.d*next_left))
					* (next_left - cur_left) * 0.5;

			cur_left = next_left;
			cur_alpha = next_alpha;
		}

		double diff = ub - lb;
		if (diff < 1e-4)
			area = 0;
		else
			area /= (ub - lb);

		return area;
	}


}
