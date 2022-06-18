package pomdp.utilities;

import pomdp.environments.Grid;
import pomdp.environments.POMDP;

import java.io.IOException;
import java.util.*;

public class MultiAgentGridLoader {
    protected Grid m_pPOMDP;
    private List<Pair<Integer, Integer>> holes;

    public MultiAgentGridLoader(Grid pomdp) {
        m_pPOMDP = pomdp;
        holes = new ArrayList<>();
    }

    public void load( String sFileName ) throws IOException, InvalidModelFileFormatException{
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
        verifyFunctions();

        System.out.println( "Done loading model" );
    }

    protected void verifyFunctions(){
        int iStartState = 0, iAction = 0, iEndState = 0, iObservation = 0;
        Iterator<Map.Entry<Integer,Double>> itNonZero = null;
        Map.Entry<Integer,Double> e = null;
        double dTr = 0.0, dSumTr = 0.0, dO = 0.0, dSumO = 0.0, dPr = 0.0, dSumPr = 0.0;
        boolean bFixed = false;
        int cStates = m_pPOMDP.getStateCount();
        int cActions = m_pPOMDP.getActionCount();

        bFixed = false;
        for( iStartState = 0 ; iStartState < cStates ; iStartState++ ){
            for( iAction = 0 ; iAction < cActions ; iAction++ ){
                dSumTr = 0.0;
                itNonZero = m_pPOMDP.getNonZeroTransitions( iStartState, iAction );
                while( itNonZero.hasNext() ){
                    e = itNonZero.next();
                    iEndState = e.getKey();
                    dTr = e.getValue();
                    dSumTr += dTr;
                }

                if( dSumTr == 0.0 ){
                    m_pPOMDP.setTransition( iStartState, iAction, iStartState, 1.0 );
                    dSumTr = 1.0;
                    bFixed = true;
                }

                if( Math.abs( dSumTr - 1.0 ) > 0.0001 )
                    System.out.println( "sum tr( " + m_pPOMDP.getStateName( iStartState ) + ", " + iAction + ", * ) = " + dSumTr );
            }
        }
        if( bFixed ){
            System.out.println( "Model file corrupted - needed to fix some transition values" );
        }

        if (m_pPOMDP.getStateToLocation() == null) {
            System.out.println( "Grid model missing rows and/or cols entries");
        }

//        for( iAction = 0 ; iAction < cActions ; iAction++ ){
//            for( iEndState = 0 ; iEndState < cStates ; iEndState++ ){
//                dSumO = 0.0;
//                itNonZero = m_pPOMDP.getNonZeroObservations( iAction, iEndState );
//                while( itNonZero.hasNext() ){
//                    e = itNonZero.next();
//                    iObservation = e.getKey();
//                    dO = e.getValue();
//                    dSumO += dO;
//                }
//                if( Math.abs( dSumO - 1.0 ) > 0.0001 )
//                    System.out.println( "sum O( " + iAction + ", " + iEndState + ", * ) = " + dSumO );
//            }
//        }
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
        Beacon b;
        int beacon_row, beacon_col, beacon_range;
        sLine = lrInput.readLine();
        while (!sLine.equals( "" )) {
            stLine = new StringTokenizer( sLine );
            beacon_row = Integer.parseInt(stLine.nextToken());
            beacon_col = Integer.parseInt(stLine.nextToken());
            stLine.nextToken();     // :
            beacon_range = Integer.parseInt(stLine.nextToken());
            b = new Beacon(beacon_row, beacon_col, beacon_range);
            m_pPOMDP.addBeacon(b);
            sLine = lrInput.readLine();
        }
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
        int iStartState = 0, iEndState = 0, iActionIdx = m_pPOMDP.getActionIndex( sAction ), iAction = 0;
        double dValue = 0;
        int cStates = m_pPOMDP.getStateCount(), cActions = m_pPOMDP.getActionCount();

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
                        m_pPOMDP.setTransition( iStartState, iAction, iEndState, dValue );
                    }
                }
                else{
                    if( sStartState.equals( "*" ) ){
                        for( iStartState = 0 ; iStartState < cStates ; iStartState++ ){
                            m_pPOMDP.setTransition( iStartState, iActionIdx, iEndState, dValue );
                        }
                    }
                    else{
                        m_pPOMDP.setTransition( iStartState, iActionIdx, iEndState, dValue );
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
                                    m_pPOMDP.setTransition( iStartState, iAction, iEndState, dValue );
                                }
                            }
                            else{
                                m_pPOMDP.setTransition( iStartState, iActionIdx, iEndState, dValue );
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
                                    m_pPOMDP.setTransition( iStartState, iAction, iEndState, dValue );
                                }
                            }
                            else{
                                m_pPOMDP.setTransition( iStartState, iActionIdx, iEndState, dValue );
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

        while( cVars < 8 ){
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

                        try{
                            m_pPOMDP.setStateCount( Integer.parseInt( sValue ) );
                        }
                        catch( NumberFormatException e ){
                            idx = 0;
                            m_pPOMDP.addState( sValue );
                            idx++;
                            while( stLine.hasMoreTokens() ){
                                sValue = stLine.nextToken();
                                m_pPOMDP.addState( sValue );
                                idx++;
                            }
                        }
                    }
                    else{//assume that state list is until next empty line
                        String sStateLine = "";
                        while(sStateLine.length() == 0){
                            sStateLine = lrInput.readLine().trim();
                        }
                        while(sStateLine.length() > 0 && !lrInput.endOfFile()){
                            m_pPOMDP.addState(sStateLine);
                            sStateLine = lrInput.readLine().trim();
                        }

                    }

                    System.out.print( "|S| = " + m_pPOMDP.getStateCount() );
                    cVars++;
                }
                else if( sType.equals( "actions:" ) ){
                    if(stLine.hasMoreElements()){

                        sValue = stLine.nextToken();

                        try{
                            m_pPOMDP.setActionCount( Integer.parseInt( sValue ) );
                        }
                        catch( NumberFormatException e ){
                            idx = 0;
                            m_pPOMDP.addAction( sValue );
                            idx++;
                            while( stLine.hasMoreTokens() ){
                                sValue = stLine.nextToken();
                                m_pPOMDP.addAction( sValue );
                                idx++;
                            }
                        }
                    }
                    else{//assume that action list is until next empty line
                        String sActionLine = "";
                        while(sActionLine.length() == 0){
                            sActionLine = lrInput.readLine().trim();
                        }
                        while(sActionLine.length() > 0 && !lrInput.endOfFile()){
                            m_pPOMDP.addAction(sActionLine);
                            sActionLine = lrInput.readLine().trim();
                        }

                    }
                    System.out.print( "|A| = " + m_pPOMDP.getActionCount() );

                    cVars++;
                }
                else if( sType.equals( "observations:" ) ){
                    if(stLine.hasMoreElements()){

                        sValue = stLine.nextToken();

                        try{
                            m_pPOMDP.setObservationCount( Integer.parseInt( sValue ) );
                        }
                        catch( NumberFormatException e ){
                            idx = 0;
                            m_pPOMDP.addObservation( sValue );
                            idx++;
                            while( stLine.hasMoreTokens() ){
                                sValue = stLine.nextToken();
                                m_pPOMDP.addObservation( sValue );
                                idx++;
                            }
                        }
                    }
                    else{//assume that observation list is until next empty line
                        String sObLine = "";
                        while(sObLine.length() == 0){
                            sObLine = lrInput.readLine().trim();
                        }
                        while(sObLine.length() > 0 && !lrInput.endOfFile()){
                            m_pPOMDP.addObservation(sObLine);
                            sObLine = lrInput.readLine().trim();
                        }

                    }
                    System.out.print( "|O| = " + m_pPOMDP.getObservationCount() );

                    cVars++;
                }

                else if (sType.equals( "radius:" )) {
                    m_pPOMDP.setRadius(Float.parseFloat(stLine.nextToken()));
                    cVars++;
                }

            }
        }
        System.out.println();
        m_pPOMDP.initDynamicsFunctions();
    }
}
