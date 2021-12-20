package pomdp;

import pomdp.algorithms.AlgorithmsFactory;
import pomdp.algorithms.ValueIteration;
import pomdp.environments.POMDP;
import pomdp.utilities.Logger;
import pomdp.utilities.MDPValueFunction;
import pomdp.utilities.concurrent.PolicyEvaluationTask;
import pomdp.utilities.concurrent.Task;
import pomdp.utilities.concurrent.ThreadPool;
import pomdp.utilities.concurrent.ValueIterationTask;
import pomdp.utilities.concurrent.VectorPruningTask;
import pomdp.valuefunction.LinearValueFunctionApproximation;

public class ParallelSolver {

	/**
	 * @param args
	 */
	public static void runDomain(String sModelName) {
		String[] aMethods = new String[]{ "PVI", "FSVI", "PBVI", "HSVI", "Pruning", "Evaluate" };
		Task[] aTasks = new Task[aMethods.length];
		int iTask = 0, cTasks = aMethods.length;
		POMDP pomdp = POMDPSolver.getPOMDP( sModelName );
		LinearValueFunctionApproximation vValueFunction = null;
		ValueIteration viAlgorithm = null;
		MDPValueFunction vfQMDP = pomdp.getMDPValueFunction();
		vfQMDP.persistQValues( true );
		vfQMDP.valueIteration( 100, 0.001 );
		ThreadPool.createInstance( pomdp );
		ThreadPool tp = ThreadPool.getInstance();
		PolicyEvaluationTask tEvaluate = null;
		for( iTask = 0 ; iTask < cTasks ; iTask++ ){
			if( aMethods[iTask] == "Pruning" ){
				aTasks[iTask] = new VectorPruningTask(pomdp, vValueFunction );
			}
			else if( aMethods[iTask] == "Evaluate" ){
				aTasks[iTask] = new PolicyEvaluationTask( pomdp, vValueFunction, aMethods.toString(), 25, 100000, 250, 150 );
				tEvaluate = (PolicyEvaluationTask) aTasks[iTask];
			}
			else{
				viAlgorithm = AlgorithmsFactory.getAlgorithm( aMethods[iTask], pomdp );
				if( vValueFunction == null )
					vValueFunction = viAlgorithm.getValueFunction();
				else
					viAlgorithm.setValueFunction( vValueFunction );
				aTasks[iTask] = new ValueIterationTask( viAlgorithm, 100, 0.001 );
			}
			tp.addTask( aTasks[iTask] );
		}
		tp.waitForTask( tEvaluate );
		for( iTask = 0 ; iTask < cTasks ; iTask++ )
			aTasks[iTask].terminate();
		for( iTask = 0 ; iTask < cTasks ; iTask++ )
			tp.waitForTask( aTasks[iTask] );
		tp.killAll();
		
	}
	public static void main(String[] args) throws Exception {
		//Logger.getInstance().setOutputStream( "Network12_16Threads.txt" );
		//runDomain( "Network" );
		//Logger.getInstance().setOutputStream( "RockSample8X8X14_16Threads.txt" );
		//runDomain( "RockSample" );
		Logger.getInstance().setOutputStream( "Logistics4X1X6_16Threads.txt" );
		runDomain( "Logistics" );
	}
}
