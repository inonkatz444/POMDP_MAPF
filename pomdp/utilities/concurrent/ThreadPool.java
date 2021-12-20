package pomdp.utilities.concurrent;

import java.util.Collections;
import java.util.LinkedList;
import java.util.Map;
import java.util.TreeMap;
import java.util.Vector;

import pomdp.environments.POMDP;
import pomdp.utilities.ExecutionProperties;
import pomdp.utilities.Logger;

public class ThreadPool {
	private Vector<TaskExecutionThread> m_vThreads;
	private LinkedList<Task> m_lTasks;
	private int m_cTasks;
	private POMDP m_pPOMDP;
	private boolean m_bTerminated;
	Map<String,Vector<Long>> m_mTaskStatistics;
	Vector<ValueIterationTask> m_vAlgorithms;
	
	private Listener m_lListener;
	
	private static ThreadPool g_tpPool = null;

	public ThreadPool( int cThreads, POMDP pomdp ){
		this( cThreads, pomdp, true );
		m_bTerminated = false;
	}
	
	public ThreadPool( int cThreads, POMDP pomdp, boolean bUseRemoteHelpers ){
		int iThread = 0;
		m_pPOMDP = pomdp;
		m_vThreads = new Vector<TaskExecutionThread>();
		m_lTasks = new LinkedList<Task>();
		for( iThread = 0 ; iThread < cThreads ; iThread++ ){
			m_vThreads.add( new TaskExecutionThread( this ) );
		}
		for( Thread t : m_vThreads ){
			t.start();
		}
		m_cTasks = 0;
		if( bUseRemoteHelpers && ExecutionProperties.useRemoteHelpers() ){
			try{
				m_lListener = new Listener( this, m_pPOMDP );
				m_lListener.start();
			}
			catch( Exception e ){
				Logger.getInstance().logError( "ThreadPool", "Constructor", "Listener could not be started - " + e );
				e.printStackTrace();
			}
		}
		m_mTaskStatistics = Collections.synchronizedMap( new TreeMap<String, Vector<Long>>() );
		m_vAlgorithms = new Vector<ValueIterationTask>();
	}

	public static ThreadPool getInstance(){
		return g_tpPool;
	}
	public static void createInstance( POMDP pomdp ){
		if( g_tpPool != null ){
			g_tpPool.killAll();
		}
		g_tpPool = new ThreadPool( ExecutionProperties.getThreadCount(), pomdp );
	}
	
	public void killAll() {
		m_bTerminated = true;
		for( TaskExecutionThread t : m_vThreads ){
			t.kill();
		}
		for( TaskExecutionThread t : m_vThreads ){
			addTask( new EmptyTask() );
			addTask( new EmptyTask() );
			addTask( new EmptyTask() );
		}
		for( TaskExecutionThread t : m_vThreads ){
			try {
				t.join( 1000 );
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

	public int addTask( Task t ){
		String sTaskName = t.getName();
		if( !m_mTaskStatistics.containsKey( sTaskName ) ){
			Vector<Long> vStatistics = new Vector<Long>();
			long l = 0;
			vStatistics.add( l );
			vStatistics.add( l );
			vStatistics.add( l );
			m_mTaskStatistics.put( sTaskName, vStatistics );
		}
		t.setStartWaitTime( System.currentTimeMillis() );
		
		if( t instanceof ValueIterationTask ){
			m_vAlgorithms.add( (ValueIterationTask) t );
		}
		
		t.setId( m_cTasks++ );
		t.init();
		synchronized( g_oTasksLock ){
			m_lTasks.addFirst( t );
		}
		synchronized( m_lTasks ){
			m_lTasks.notify();
		}
		return t.getId();
	}
	
	private Object g_oTasksLock = new Object();
	
	Task getNextTask() {
		Task t = null;
		while( t == null ){
			synchronized( g_oTasksLock ){
				if( m_lTasks.size() > 0 ){
					t = m_lTasks.removeFirst();
				}
			}
			if( t == null ){
				try {
					synchronized( m_lTasks ){
						m_lTasks.wait();
					}
				} 
				catch( InterruptedException e ){
				}
			}
		}
		t.setStartExecutionTime( System.currentTimeMillis() );
		return t;
	}

	void taskDone( Task t ) {
		synchronized( t ){
			t.done();
			t.notify();
		}
		t.setEndExecutionTime( System.currentTimeMillis() );
		String sTaskName = t.getName();
		Vector<Long> vStatistics = m_mTaskStatistics.get( sTaskName );
		vStatistics.set( 0, vStatistics.elementAt( 0 ) + 1 );
		vStatistics.set( 1, vStatistics.elementAt( 1 ) + t.getStartExecutionTime() - t.getStartWaitTime() );
		vStatistics.set( 2, vStatistics.elementAt( 2 ) + t.getEndExecutionTime() - t.getStartExecutionTime() );
	}
	public void waitForTask( Task t ){
		while( !t.isDone() && !m_bTerminated ){
			try {
				synchronized( t ){
					if( !t.isDone() )
						t.wait( 1000 );
				}
			} 
			catch (InterruptedException e) {
			}
		}
	}

	public void addThread( RemoteTaskExecutionThread t ){
		m_vThreads.add( t );
		t.start();
	}

	public void clear() {
		killAll();
		m_vThreads.clear();
		m_lTasks.clear();
		m_vThreads = null;
		m_lTasks = null;
		g_tpPool = null;
	}

	public void printStatistics() {
		for( String sTaskName : m_mTaskStatistics.keySet() ){
			Vector<Long> vStatistics = m_mTaskStatistics.get( sTaskName );
			long cExecutions = vStatistics.elementAt( 0 );
			long lWaitTime = vStatistics.elementAt( 1 );
			long lExecutionTime = vStatistics.elementAt( 2 );
			if( cExecutions > 0 ){
				Logger.getInstance().log( "ThreadPool", 0, "printStatistics", sTaskName + 
						", executions " + cExecutions + ", wait time " + lWaitTime / cExecutions +
						", execution time " + lExecutionTime / cExecutions );
			}
			else{
				Logger.getInstance().log( "ThreadPool", 0, "printStatistics", sTaskName + 
						", first execution still running" );
			}
		}
		for( ValueIterationTask t : m_vAlgorithms ){
			Logger.getInstance().log( "ThreadPool", 0, "printStatistics", t.getAlgorithmName() + 
					" backups " + t.getBackupCount() );
		}
	}
}
