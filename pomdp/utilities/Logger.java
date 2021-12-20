package pomdp.utilities;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;

public class Logger {
	private int m_iMaximalLevel;
	private static Logger m_lInstance = null;
	private Runtime m_rtRuntime;
	private PrintStream m_psOutput;

	private Logger(){
		m_iMaximalLevel = 2;
		m_rtRuntime = Runtime.getRuntime();
		m_psOutput = System.out;
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
	
	public static Logger getInstance(){
		if( m_lInstance == null )
			m_lInstance = new Logger();
		return m_lInstance;
	}
	
	public void log( String sClassName, int iLevel, String sMethodName, String sMessage ){
		if( iLevel <= m_iMaximalLevel ){
			String sFullMsg = sClassName + ":" + sMethodName + ":" + sMessage;
			m_psOutput.println( sFullMsg );
			if( m_psOutput != System.out )
				System.out.println( sFullMsg );
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
