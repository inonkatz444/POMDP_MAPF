package pomdp.utilities;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.util.HashMap;
import java.util.Map;

public class TrackLogger {
	private int m_iMaximalLevel;
	private static TrackLogger m_lInstance = null;
	private Runtime m_rtRuntime;
	private PrintStream m_psOutput;

	private static Map<Character, TrackLogger> agentTrackLogger = new HashMap<>();
	private Map<Character, PrintStream> agentTrackLoggerOutput;

	private TrackLogger(){
		m_iMaximalLevel = 2;
		m_rtRuntime = Runtime.getRuntime();
		m_psOutput = System.out;
	}

	public static TrackLogger getAgentInstance(char id) {
		if (agentTrackLogger.get(id) == null) {
			agentTrackLogger.put(id, new TrackLogger());
		}
		return agentTrackLogger.get(id);
	}
	
	public void finalize(){
		m_psOutput.flush();
		m_psOutput.close();
	}
	
	public void setOutputStream( String sFileName ) throws FileNotFoundException{
		if( m_psOutput != null && m_psOutput != System.out ){
			m_psOutput.flush();
			m_psOutput.close();
		}			
		m_psOutput = new PrintStream( new FileOutputStream( sFileName ) );
	}
	
	public static TrackLogger getInstance(){
		if( m_lInstance == null )
			m_lInstance = new TrackLogger();
		return m_lInstance;
	}
	
	public void log( String sClassName, int iLevel, String sMethodName, boolean printToSystem, String sMessage ){
		if( iLevel <= m_iMaximalLevel ){
			String sFullMsg = sClassName + ":" + sMethodName + ":" + sMessage;
			m_psOutput.println( sFullMsg );
			if( m_psOutput != System.out && printToSystem )
				System.out.println( sFullMsg );
			m_psOutput.flush();
		}
	}

	public void freeLog(int iLevel, String sMessage) {
		if( iLevel <= m_iMaximalLevel ){
			m_psOutput.println( sMessage );
			m_psOutput.flush();
		}
	}
	
	public void logError( String sClassName, String sMethodName, String sMessage ){
		System.err.println( sClassName + ":" + sMethodName + ":" + sMessage );
	}
	public void logFull( String sClassName, int iLevel, String sMethodName, String sMessage ){
		if( iLevel <= m_iMaximalLevel ){
			String sFullMsg = sClassName + ":" + sMethodName + ":" + sMessage +
				", memory: " + 
				" total " + m_rtRuntime.totalMemory() / 1000000 +
				" free " + m_rtRuntime.freeMemory() / 1000000 +
				" max " + m_rtRuntime.maxMemory() / 1000000;
			m_psOutput.println( sFullMsg );
			if( m_psOutput != System.out )
				System.out.println( sFullMsg );
			m_psOutput.flush();
		}
	}
}
