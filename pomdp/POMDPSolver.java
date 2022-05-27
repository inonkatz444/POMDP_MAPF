/*
 * Created on May 3, 2005
 *
 * To change the template for this generated file go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
package pomdp;

import pomdp.algorithms.AlgorithmsFactory;
import pomdp.algorithms.ValueIteration;
import pomdp.environments.*;
import pomdp.environments.FactoredPOMDP.BeliefType;
import pomdp.utilities.ExecutionProperties;
import pomdp.utilities.JProf;
import pomdp.utilities.Logger;
import pomdp.utilities.MDPValueFunction;

/**
 * @author shanigu, inonk
 *
 *
 * To change the template for this generated type comment go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
public class POMDPSolver {
	
	public static POMDP getPOMDP( String sModelName ){
		POMDP pomdp = null;
		try{
			if( sModelName.equals( "RockSample" ) ){
				int cX = 8, cY = 8, cRocks = 14;
				pomdp = new ModifiedRockSample( cX, cY ,cRocks, BeliefType.Flat );
			}
			else if( sModelName.equals( "Network")){
				int cMachines = 12;
				pomdp = new NetworkManagement( cMachines, BeliefType.Flat );
			}
			else if( sModelName.equals( "Logistics")){
				int cPackages = 6, cCities = 4, cTrucks = 1;
				pomdp = new Logistics( cCities, cTrucks, cPackages, BeliefType.Flat );
			}
			else if( sModelName.equals( "test_grid")){
				pomdp = new Grid(1);
				pomdp.load(ExecutionProperties.getPath() + sModelName + ".POMDP", 'a');
			}
			else{
				pomdp = new POMDP();
				pomdp.load( ExecutionProperties.getPath() + sModelName + ".POMDP", 'a');
			}
		}
		catch( Exception e ){
			System.out.println( e );
			e.printStackTrace();
			System.exit( 0 );
		}
		return pomdp;
	}

	public static void main(String[] args){
		JProf.getCurrentThreadCpuTimeSafe();
		
		String sPath = ExecutionProperties.getPath();
		//String sModelName = "Network-3-2-4-6";
		String sModelName = "RockSample_5_7";
		String sMethodName = "FSVI";
						
		if( args.length > 0 )
			sModelName = args[0];
		if( args.length > 1 )
			sMethodName = args[1];			
		if( args.length > 2 )
			sMethodName = args[1];			
		
		POMDP pomdp = getPOMDP( sModelName );
		
		double dTargetADR = 100.0;

		try{
			Logger.getInstance().setOutputStream( pomdp.getName() + "_" + sMethodName + ".txt" );
		}
		catch( Exception e ){
			System.err.println( e );
		}

		if( sMethodName.equals( "QMDP" ) ){
			MDPValueFunction vfQMDP = pomdp.getMDPValueFunction();
			vfQMDP.persistQValues( true );
			vfQMDP.valueIteration( 100, 0.001 );
			double dDiscountedReward = pomdp.computeAverageDiscountedReward( 1000, 100, vfQMDP );
			Logger.getInstance().log( "POMDPSolver", 0, "main", "ADR = " + dDiscountedReward );
			System.exit( 0 );
		}
		
		ValueIteration viAlgorithm = AlgorithmsFactory.getAlgorithm( sMethodName, pomdp );
		int cMaxIterations = 500;
		try{					
			viAlgorithm.valueIteration( cMaxIterations, ExecutionProperties.getEpsilon(), dTargetADR );
			double dDiscountedReward = pomdp.computeAverageDiscountedReward( 500, 150, viAlgorithm );
			Logger.getInstance().log( "POMDPSolver", 0, "main", "ADR = " + dDiscountedReward );
		}

		catch( Exception e ){
			System.out.println( e );
			e.printStackTrace();
		} 
		catch( Error err ){
			Runtime rtRuntime = Runtime.getRuntime();
			System.out.println( "POMDPSolver: " + err +
					" allocated " + ( rtRuntime.totalMemory() - rtRuntime.freeMemory() ) / 1000000 +
					" free " + rtRuntime.freeMemory() / 1000000 +
					" max " + rtRuntime.maxMemory() / 1000000 );
			System.out.print( "Stack trace: " );
			err.printStackTrace();
		}
	}
}
