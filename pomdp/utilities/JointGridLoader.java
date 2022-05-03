package pomdp.utilities;

import pomdp.environments.JointBeaconDistanceGrid;
import pomdp.utilities.datastructures.Function;

import java.io.IOException;
import java.util.*;

public class JointGridLoader {
    protected JointBeaconDistanceGrid m_pPOMDP;
    protected int numOfAgents;
    protected int numOfSingleStates;
    protected int numOfSingleActions;
    protected Function singleTransitions;
    protected Map<Integer, String> idxToSSingleAction;
    protected Map<String, Integer> sActionToIdx;

    public JointGridLoader(JointBeaconDistanceGrid pomdp) {
        m_pPOMDP = pomdp;
        idxToSSingleAction = new HashMap<>();
        sActionToIdx = new HashMap<>();
    }

    public Function load( String sFileName ) throws IOException, InvalidModelFileFormatException{
        System.out.println( "Started loading model " + sFileName );
        LineReader lrInput = new LineReader( sFileName );
        String sLine = "";
        String sType = "", sValue;
        StringTokenizer stLine;
        int cLines = 0;

        try{
            readHeader( lrInput );
        }
        catch( EndOfFileException e ){
            throw new InvalidModelFileFormatException( "Missing header parameters" );
        }

        while( !lrInput.endOfFile() ){
            sLine = lrInput.readLine();
            try{
                cLines++;
                if( cLines % 1000 == 0 ){
                    System.out.print( "." );
                }
                if( ( sLine.length() > 0 ) && ( sLine.charAt( 0 ) != '#' ) ){
                    stLine = new StringTokenizer( sLine );
                    if( stLine.hasMoreTokens() ){
                        sType = stLine.nextToken();
                        switch (sType) {
                            case "T:":
                                sValue = stLine.nextToken();
                                readTransition(lrInput, sValue, stLine);
                                break;
                            case "O:":
                                sValue = stLine.nextToken();
                                readObservation(lrInput, sValue, stLine);
                                break;
                            case "holes:":
                                readHoles(lrInput);
                                break;
                            case "start_states:":
                                readStartStates(lrInput);
                                break;
                            case "end_states:":
                                readEndStates(lrInput);
                                break;
                            case "beacons:":
                                readBeacons(lrInput);
                                break;
                            case "OS:":
                                if (stLine.hasMoreTokens())
                                    readObservationStates(stLine);
                                break;
                        }
                    }
                }
            }
            catch(Exception e)
            {
                System.out.println("Error at line: " + sLine);
                System.out.println(e);
                System.exit(1);
            }
        }

        m_pPOMDP.initGrid();

        System.out.println( "Done loading model" );
        return singleTransitions;
    }

    /**
     * Supporting the format:
     * OS: <line of observation sensitive states>
     */
    private void readObservationStates( StringTokenizer stLine ){
        String sObservationState = "";
        int iObservationState = 0;

        while( stLine.hasMoreElements() ){
            sObservationState = stLine.nextToken();
            iObservationState = m_pPOMDP.getStateIndex( sObservationState );
            m_pPOMDP.addObservationSensitiveState( iObservationState );
        }
    }

    private void readStartStates(LineReader lrInput) throws IOException {
        String sLine = "";
        StringTokenizer stLine;
        char id;
        int startState;
        int num_of_agents = 0;
        sLine = lrInput.readLine();
        while (!sLine.equals( "" )) {
            stLine = new StringTokenizer( sLine );
            id = stLine.nextToken().charAt(0);
            startState = Integer.parseInt(stLine.nextToken());
            m_pPOMDP.addStartState(id, startState);
            m_pPOMDP.setAgentListPos(id);
            num_of_agents++;
            sLine = lrInput.readLine();
        }

        this.numOfAgents = num_of_agents;
        m_pPOMDP.setStateCount( (int)Math.pow(numOfSingleStates, numOfAgents) );
        System.out.print( "|S| = " + m_pPOMDP.getStateCount() );

        int[] transitionDims = {numOfSingleStates, numOfSingleActions, numOfSingleStates};
        singleTransitions = new SparseTabularFunction( transitionDims );


        int maxActionValue = numOfSingleActions-1;
        List<Integer> actionValues = new ArrayList<>();

        for (int i = 0; i < numOfAgents; i++) {
            actionValues.add(0);
        }

        int actionChecksum;
        boolean actionDone = false;
        boolean actionChange;
        int actionR;
        StringBuilder sAction;

        while (!actionDone) {
            actionChecksum = 0;
            for (int i = 0; i < numOfAgents; i++) {
                actionChecksum += actionValues.get(i);
            }
            if (actionChecksum == numOfAgents*(maxActionValue)){
                actionDone = true;
            }

            // add actions here
            sAction = new StringBuilder("(");
            for (int i = 0; i < numOfAgents; i++) {
                sAction.append(idxToSSingleAction.get(actionValues.get(i))).append(", ");
            }
            sAction.delete(sAction.length()-2, sAction.length());
            sAction.append(")");
            m_pPOMDP.addAction(sAction.toString());
            //

            actionChange = true;
            actionR = 0;
            while (actionChange && actionR < numOfAgents) {
                actionValues.set(actionR, actionValues.get(actionR)+1);
                if (actionValues.get(actionR) > maxActionValue) {
                    actionValues.set(actionR, 0);
                }
                else {
                    actionChange = false;
                }
                actionR++;
            }
        }
        System.out.print( "|A| = " + m_pPOMDP.getActionCount() );
        System.out.println();
    }

    private void readEndStates(LineReader lrInput) throws IOException {
        String sLine = "";
        StringTokenizer stLine;
        char id;
        int endState;
        sLine = lrInput.readLine();
        while (!sLine.equals( "" )) {
            stLine = new StringTokenizer( sLine );
            id = stLine.nextToken().charAt(0);
            endState = Integer.parseInt(stLine.nextToken());
            m_pPOMDP.addEndState(id, endState);
            sLine = lrInput.readLine();
        }
    }

    protected void readHoles( LineReader lrInput) throws InvalidModelFileFormatException, IOException {
        String sLine = "";
        StringTokenizer stLine;
        int hole_row, hole_col;
        sLine = lrInput.readLine();
        while (!sLine.equals( "" )) {
            stLine = new StringTokenizer( sLine );
            hole_row = Integer.parseInt(stLine.nextToken());
            hole_col = Integer.parseInt(stLine.nextToken());
            m_pPOMDP.addHole(hole_row, hole_col);
            sLine = lrInput.readLine();
        }

    }

    // syntax: row col : range
    protected void readBeacons(LineReader lrInput) throws IOException {
        String sLine = "";
        StringTokenizer stLine;
        int beacon_row, beacon_col, beacon_range;
        int max_range = 0, num_beacons = 0;
        sLine = lrInput.readLine();
        while (!sLine.equals( "" )) {
            stLine = new StringTokenizer( sLine );
            beacon_row = Integer.parseInt(stLine.nextToken());
            beacon_col = Integer.parseInt(stLine.nextToken());
            stLine.nextToken();     // :
            beacon_range = Integer.parseInt(stLine.nextToken());
            m_pPOMDP.addBeacon(beacon_row, beacon_col, beacon_range);
            max_range = Math.max(max_range, beacon_range);
            num_beacons++;
            sLine = lrInput.readLine();
        }

        int numOfSingleObservations = (int)Math.pow(max_range + 2, num_beacons);    // +1 for zero dist, +1 for inf dist
        m_pPOMDP.setNumOfSingleObservations(numOfSingleObservations);
        m_pPOMDP.setObservationCount((int)Math.pow(numOfSingleObservations, numOfAgents));
        m_pPOMDP.initDynamicsFunctions();
        m_pPOMDP.setMaxDist(max_range);
    }

    /**
     * Supporting the following formats:
     * T: <action>  followed by a transition matrix
     * T: <action> : <start state>  followed by a single line of transitions
     * T: <action> : <start state> : <end state> %f
     * TODO - add support for a wildcard asterix (*)
     */
    protected void readTransition( LineReader lrInput, String sAction, StringTokenizer stLine ) throws InvalidModelFileFormatException{
        String sStartState = "", sEndState = "", sValue = "", sLine = "";
        int iStartState = 0, iEndState = 0, iActionIdx = sActionToIdx.get( sAction ), iAction = 0;
        double dValue = 0;
        int cStates = numOfSingleStates, cActions = numOfSingleActions;

        if( stLine.hasMoreTokens() ){
            String sTemp = stLine.nextToken();
            sStartState = stLine.nextToken();
            if( sStartState.equals( "*" ) )
                iStartState = -1;
            else
                iStartState = m_pPOMDP.getStateIndex( sStartState );
            if( stLine.hasMoreTokens() ){
                stLine.nextToken();
                sEndState = stLine.nextToken();
                sValue = stLine.nextToken();
                if( sEndState.equals( "*" ) )
                    iEndState = -1;
                else
                    iEndState = m_pPOMDP.getStateIndex( sEndState );
                dValue = Double.parseDouble( sValue );

                if( dValue == 0.0 )
                    return;

                if( sAction.equals( "*" ) ){
                    for( iAction = 0 ; iAction < cActions ; iAction++ ){
                        singleTransitions.setValue( iStartState, iAction, iEndState, dValue );
                    }
                }
                else{
                    if( sStartState.equals( "*" ) ){
                        for( iStartState = 0 ; iStartState < cStates ; iStartState++ ){
                            singleTransitions.setValue( iStartState, iActionIdx, iEndState, dValue );
                        }
                    }
                    else{
                        singleTransitions.setValue( iStartState, iActionIdx, iEndState, dValue );
                    }
                }

            }
            else{
                try{
                    sLine = lrInput.readLine();
                    stLine = new StringTokenizer( sLine );
                    for( iEndState = 0 ; iEndState < cStates ; iEndState++ ){
                        sValue = stLine.nextToken();
                        dValue = Double.parseDouble( sValue );

                        if( dValue != 0.0 ){
                            if( sAction.equals( "*" ) ){
                                for( iAction = 0 ; iAction < cActions ; iAction++ ){
                                    singleTransitions.setValue( iStartState, iAction, iEndState, dValue );
                                }
                            }
                            else{
                                singleTransitions.setValue( iStartState, iActionIdx, iEndState, dValue );
                            }
                        }
                    }
                }
                catch( NoSuchElementException e ){
                    throw new InvalidModelFileFormatException( "insufficient number of transitions " + sLine );
                }
                catch( IOException e ){
                    throw new InvalidModelFileFormatException();
                }
            }
        }
        else{
            try{
                for( iStartState = 0 ; iStartState < cStates ; iStartState++ ){
                    sLine = lrInput.readLine();
                    //System.out.println( sLine );
                    stLine = new StringTokenizer( sLine );
                    for( iEndState = 0 ; iEndState < cStates ; iEndState++ ){
                        sValue = stLine.nextToken();
                        dValue = Double.parseDouble( sValue );

                        if( dValue != 0.0 ){
                            if( sAction.equals( "*" ) ){
                                for( iAction = 0 ; iAction < cActions ; iAction++ ){
                                    singleTransitions.setValue( iStartState, iAction, iEndState, dValue );
                                }
                            }
                            else{
                                singleTransitions.setValue( iStartState, iActionIdx, iEndState, dValue );
                            }
                        }
                    }
                }
            }
            catch( NoSuchElementException e ){
                throw new InvalidModelFileFormatException( "insufficient number of transitions " + sLine );
            }
            catch( IOException e ){
                throw new InvalidModelFileFormatException();
            }
        }
    }

    /**
     * Supporting the following formats:
     * O: <action> : <end state> followed by an observation matrix
     * O: <action> : <end state> followed by a single line of observation
     * O: <action> : <end state> : <observation> %f
     * TODO - add support for a wildcard asterix (*)
     */
    protected void readObservation( LineReader lrInput, String sAction, StringTokenizer stLine ) throws InvalidModelFileFormatException{
        String sObservation = "", sEndState = "", sValue = "", sLine = "";
        int iObservation = 0, iEndState = 0, iActionIdx = m_pPOMDP.getActionIndex( sAction ), iAction = 0, iState = 0;
        double dValue = 0;

        if( stLine.hasMoreTokens() ){
            stLine.nextToken(); //":"
            sEndState = stLine.nextToken();
            if( sEndState.equals( "*" ) )
                iEndState = -1;
            else
                iEndState = m_pPOMDP.getStateIndex( sEndState );

            if( stLine.hasMoreTokens() ){
                stLine.nextToken();//":"
                sObservation = stLine.nextToken();
                iObservation = m_pPOMDP.getObservationIndex( sObservation );
                sValue = stLine.nextToken();

                dValue = Double.parseDouble( sValue );

                if( sEndState.equals( "*" ) )
                    iEndState = -1;

                if( sAction.equals( "*" ) )
                    iActionIdx = -1;

                m_pPOMDP.setObservation( iActionIdx, iEndState, iObservation, dValue );
            }
            else{
                try{
                    sLine = lrInput.readLine();
                    stLine = new StringTokenizer( sLine );
                    for( iObservation = 0 ; iObservation < m_pPOMDP.getObservationCount() ; iObservation++ ){
                        sValue = stLine.nextToken();
                        dValue = Double.parseDouble( sValue );

                        if( sAction.equals( "*" ) )
                            iActionIdx = -1;
                        m_pPOMDP.setObservation( iActionIdx, iEndState, iObservation, dValue );
                    }
                }
                catch( NoSuchElementException e ){
                    throw new InvalidModelFileFormatException( "insufficient number of observations " + sLine );
                }
                catch( IOException e ){
                    throw new InvalidModelFileFormatException();
                }
            }
        }
        else{
            try{
                for( iEndState = 0 ; iEndState < m_pPOMDP.getStateCount() ; iEndState++ ){
                    sLine = lrInput.readLine();
                    stLine = new StringTokenizer( sLine );
                    for( iObservation = 0 ; iObservation < m_pPOMDP.getObservationCount() ; iObservation++ ){
                        sValue = stLine.nextToken();
                        dValue = Double.parseDouble( sValue );

                        if( sAction.equals( "*" ) )
                            iActionIdx = -1;
                        m_pPOMDP.setObservation( iActionIdx, iEndState, iObservation, dValue );

                    }
                }
            }
            catch( NoSuchElementException e ){
                throw new InvalidModelFileFormatException( "insufficient number of observations " + sLine );
            }
            catch( IOException e ){
                throw new InvalidModelFileFormatException();
            }
        }
    }

    protected void readHeader(LineReader lrInput) throws IOException, InvalidModelFileFormatException {
        String sLine = "";
        String sType = "", sValue;
        int cVars = 0, idx = 0, rows = 0, cols = 0;
        StringTokenizer stLine;

        while( cVars < 6 ){
            sLine = "";
            while( sLine.equals( "" ) ){
                sLine = lrInput.readLine();
                //System.out.println( sLine );
                sLine = sLine.trim();
            }

            //System.out.println( sLine );

            stLine = new StringTokenizer( sLine );
            sType = stLine.nextToken();
            if( !sType.equals( "#" ) ){ //not a comment
                if( sType.equals( "discount:" ) ){
                    sValue = stLine.nextToken();
                    m_pPOMDP.setDiscountFactor( Double.parseDouble( sValue ) );
                    cVars++;
                }
                else if( sType.equals("rows:") ){
                    sValue = stLine.nextToken();
                    rows = Integer.parseInt( sValue );
                    m_pPOMDP.setRows(rows);
                    cVars++;
                }
                else if( sType.equals("cols:") ){
                    sValue = stLine.nextToken();
                    cols = Integer.parseInt( sValue );
                    m_pPOMDP.setCols(cols);
                    cVars++;
                }
                else if( sType.equals( "values:" ) ){
                    cVars++;
                }
                else if( sType.equals( "states:" ) ){
                    if(stLine.hasMoreElements()){

                        sValue = stLine.nextToken();
                        numOfSingleStates = Integer.parseInt( sValue );
                        m_pPOMDP.setNumOfSingleStates(numOfSingleStates);
                    }

                    cVars++;
                }
                else if( sType.equals( "actions:" ) ){
                    if(stLine.hasMoreElements()){

                        sValue = stLine.nextToken();

                        List<String> actions = new ArrayList<>();
                        idx = 0;
                        actions.add( sValue );
                        idxToSSingleAction.put(idx, sValue);
                        sActionToIdx.put(sValue, idx);
                        idx++;
                        while( stLine.hasMoreTokens() ){
                            sValue = stLine.nextToken();
                            actions.add( sValue );
                            idxToSSingleAction.put(idx, sValue);
                            sActionToIdx.put(sValue, idx);
                            idx++;
                        }
                        numOfSingleActions = idx;
                        m_pPOMDP.setNumOfSingleActions(numOfSingleActions);
                    }

                    cVars++;
                }

            }
        }
    }

    public int getNumOfSingleStates() {
        return numOfSingleStates;
    }

    public int getNumOfSingleActions() {
        return numOfSingleActions;
    }
}