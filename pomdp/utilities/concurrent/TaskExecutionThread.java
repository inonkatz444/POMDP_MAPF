package pomdp.utilities.concurrent;


public class TaskExecutionThread extends Thread{
	protected ThreadPool m_tpPool;
	protected boolean m_bKilled;
	protected Task m_tCurrent;
	
	public TaskExecutionThread( ThreadPool tp ){
		m_tpPool = tp;
		m_bKilled = false;
		m_tCurrent = null;
	}
	public void run(){
		try{
			while( !m_bKilled ){
				m_tCurrent = m_tpPool.getNextTask();
				m_tCurrent.execute();
				m_tpPool.taskDone( m_tCurrent );
			}	
		}
		catch( Error e ){
			System.err.println( e );
			e.printStackTrace();
		}
	}
	public void kill(){
		m_bKilled = true;
		if( m_tCurrent != null )
			m_tCurrent.terminate();
	}
}
