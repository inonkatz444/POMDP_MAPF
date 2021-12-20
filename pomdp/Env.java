package pomdp;

import pomdp.environments.Grid;
import pomdp.utilities.ExecutionProperties;
import pomdp.utilities.InvalidModelFileFormatException;
import pomdp.utilities.JProf;

import java.io.IOException;

public class Env {

    public static void main(String[] args) {
        JProf.getCurrentThreadCpuTimeSafe();
        String sModelName = "test_grid";
        String sMethodName = "FSVI";

        GridAgent agent = new GridAgent();
        try {
            agent.load(ExecutionProperties.getPath() + sModelName + ".POMDP");
        } catch (InvalidModelFileFormatException | IOException e) {
            e.printStackTrace();
        }

        agent.solve(sMethodName, 100.0);
    }
}
