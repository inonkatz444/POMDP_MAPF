package pomdp;

import pomdp.algorithms.AlgorithmsFactory;
import pomdp.algorithms.ValueIteration;
import pomdp.environments.Grid;
import pomdp.utilities.ExecutionProperties;
import pomdp.utilities.InvalidModelFileFormatException;
import pomdp.utilities.JProf;
import pomdp.utilities.Logger;

import java.io.IOException;

public class Env {

    public static void main(String[] args) {
        JProf.getCurrentThreadCpuTimeSafe();
//        String sModelName = "straight_line_side_beacon_15x23";
        String sModelName = "straight_line_side_beacon_9x15";
//        String sModelName = "short_hallway_side_beacon";
//        String sModelName = "test_grid";
        String sMethodName = "Perseus";

        try{
            Logger.getInstance().setOutputStream( sModelName + "_" + sMethodName + ".txt" );
        }
        catch( Exception e ){
            System.err.println( e );
        }

        GridAgent agent = new GridAgent();
        try {
            agent.load(ExecutionProperties.getPath() + sModelName + ".POMDP");
        } catch (InvalidModelFileFormatException | IOException e) {
            e.printStackTrace();
        }

//        agent.solve(sMethodName, 100.0, 30);

        double averageADR = 0.0;
        Grid grid = new Grid();
        try {
            grid.load(ExecutionProperties.getPath() + sModelName + ".POMDP");
        } catch (IOException e) {
            e.printStackTrace();
        } catch (InvalidModelFileFormatException e) {
            e.printStackTrace();
        }
        ValueIteration viAlgorithm;
        try{
            for (int i = 0; i < 10; i++) {
                viAlgorithm = AlgorithmsFactory.getAlgorithm( sMethodName, grid );
                assert viAlgorithm != null;
                viAlgorithm.valueIteration( 30, ExecutionProperties.getEpsilon(), 100.0 );
                double dDiscountedReward = grid.computeAverageDiscountedReward( 500, 150, viAlgorithm );
                Logger.getInstance().log( "POMDPSolver", 0, "main", "ADR = " + dDiscountedReward );
                averageADR += dDiscountedReward;
            }
            averageADR /= 10;
            System.out.println("Average ADR: " + averageADR);
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
