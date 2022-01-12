package pomdp;

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
//        String sModelName = "test_grid";
        String sMethodName = "FSVI";

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

        agent.solve(sMethodName, 100.0, 200);
    }
}
