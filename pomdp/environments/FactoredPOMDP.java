package pomdp.environments;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Collection;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.TreeMap;
import java.util.Vector;

import java.util.Map.Entry;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

import com.sun.swing.internal.plaf.basic.resources.basic;

import pomdp.algorithms.PolicyStrategy;
import pomdp.environments.POMDP.RewardType;
import pomdp.utilities.AlphaVector;
import pomdp.utilities.ArrayComparator;
import pomdp.utilities.BeliefState;
import pomdp.utilities.BeliefStateFactory;
import pomdp.utilities.ExecutionProperties;
import pomdp.utilities.JProf;
import pomdp.utilities.Logger;
import pomdp.utilities.TabularAlphaVector;
import pomdp.utilities.factored.AffineADD;
import pomdp.utilities.factored.AlgebraicDecisionDiagram;
import pomdp.utilities.factored.CompactAlgebraicDecisionDiagram;
import pomdp.utilities.factored.FactoredAlphaVector;
import pomdp.utilities.factored.FactoredBeliefState;
import pomdp.utilities.factored.FactoredBeliefStateFactory;
import pomdp.utilities.factored.IndepandantBeliefState;
import pomdp.utilities.factored.IndependenBeliefStateFactory;
import pomdp.utilities.factored.LogisticsBeliefState;
import pomdp.utilities.factored.AlgebraicDecisionDiagram.VariableTranslator;
import sun.reflect.generics.reflectiveObjects.NotImplementedException;

public abstract class FactoredPOMDP extends POMDP{
	protected int m_cStateVariables;
	protected AlgebraicDecisionDiagram[] m_adReward;
	private AlgebraicDecisionDiagram[][] m_adObservation;	
	private AlgebraicDecisionDiagram[][] m_adIndependentComponentTransitions;
	private AlgebraicDecisionDiagram[][] m_adNegativeTransitions;
	private AlgebraicDecisionDiagram[][] m_adDualAction;
	private AlgebraicDecisionDiagram[][] m_adCompleteAction;
	private AlgebraicDecisionDiagram[][] m_adCompleteActionForG;
	private AlgebraicDecisionDiagram[][] m_adCompleteActionForBS;
	private AlgebraicDecisionDiagram[][] m_adRelevantAction;
	private AlgebraicDecisionDiagram[] m_adIrelevantAction;
	protected BeliefType m_btFactored;
	private VariableTranslator m_vReducer, m_vPreExpander, m_vPostExpander;
	public static boolean g_bUseIrrelelvant = false;
	
	public boolean m_bUseSpecialADDForG = true;
	public boolean m_bUseRelevantVariablesOnly = true;
	
	private final static double m_dEpsilon = 0.001; 	
	
	public enum BeliefType{
		Flat, Factored, Independent;
	}
	
	public FactoredPOMDP( int cStateVariables, int cActions, int cObservations, BeliefType btFactored, boolean bUseSpecialADDForG, boolean bUseRelevantVariablesOnly ){
		this();
		m_bUseSpecialADDForG = bUseSpecialADDForG;
		m_bUseRelevantVariablesOnly = bUseRelevantVariablesOnly;
		if( m_bUseRelevantVariablesOnly )
			m_bUseSpecialADDForG = false;
		m_btFactored = btFactored;
		m_cActions = cActions;
		m_cObservations = cObservations;
		m_cStates = (int)Math.pow( 2, cStateVariables );
		m_cStateVariables = cStateVariables;
		
		initBeliefStateFactory();
		if( !ExecutionProperties.useMultiThread() )
			getBeliefStateFactory().cacheBeliefStates( true );
		else
			getBeliefStateFactory().cacheBeliefStates( false );
			

	}
	
	public FactoredPOMDP(){
		m_adReward = null;
		m_adObservation = null;
		m_adIndependentComponentTransitions = null;//new AlgebraicDecisionDiagram[m_cActions][m_cStateVariables];
		m_adNegativeTransitions = null;//new AlgebraicDecisionDiagram[m_cActions][m_cStateVariables];
		m_adDualAction = null;//new AlgebraicDecisionDiagram[m_cActions][m_cStateVariables];
		m_adCompleteAction = null;
		m_adRelevantAction = null;
		m_adIrelevantAction = null;
		m_adCompleteActionForG = null;
		m_adCompleteActionForG = null;
		m_adCompleteActionForBS = null;
		
		m_bGBasedBackup = false;
		m_vReducer = m_vPreExpander = m_vPostExpander = null;
	}
	
	protected void initADDs() {
		String sFileName = ExecutionProperties.getPath() + getName() + ".pomdp";
		boolean bFileLoaded = false;

		long lTimeBefore = JProf.getCurrentThreadCpuTimeSafe();
		
		//initStoredRewards();
		
		if( m_btFactored == BeliefType.Factored || m_btFactored == BeliefType.Independent ){
			try{
				//bFileLoaded = readADDs( sFileName );
				bFileLoaded = false;
			}
			catch( Exception e ){
				if( e instanceof FileNotFoundException )
					Logger.getInstance().log( "FactoredPOMDP", 0, "initADDs", "No model file defintion - need to compute ADDS" );
				else{
					Logger.getInstance().log( "FactoredPOMDP", 0, "initADDs", "Corrupted model file defintion - " + e );
					e.printStackTrace();
				}
			}
			
			if( !bFileLoaded ){
				if( m_btFactored == BeliefType.Factored || m_btFactored == BeliefType.Independent ){
					learnRewards();
					//learnObservations();
					//learnTransitions();
					if( !m_bUseRelevantVariablesOnly )
						createCompleteActionDiagrams();
					else
						createPartialActionDiagrams();
				}
				try{
					//writeADDs( sFileName );
				}
				catch( Exception e ){
					e.printStackTrace();
					Logger.getInstance().log( "FactoredPOMDP", 0, "initADDs", "Could not create model file - " + e );
				}
			}
		}
		if( m_btFactored == BeliefType.Independent ){
			learnIndependentCompoenentDiagrams();
			learnObservations();
		}
		
		long lTimeAfter = JProf.getCurrentThreadCpuTimeSafe();
		Logger.getInstance().log( "FactoredPOMDP", 0, "initADDs", "Finished intializing. Time - " + ( lTimeAfter - lTimeBefore ) / 1000000000.0 );
	}

	protected void learnIndependentCompoenentDiagrams() {
		int iComponent = 0, iAction = 0;
		m_adIndependentComponentTransitions = new AlgebraicDecisionDiagram[m_cActions][getIndependentComponentsCount()];
		for( iAction = 0 ; iAction < m_cActions ; iAction++ ){
			for( iComponent = 0 ; iComponent < getIndependentComponentsCount() ; iComponent++ ){
				learnIndependentComponentDiagram( iAction, iComponent );
			}
		}		
	}


	private int[] unifyVariables( int[] aiStart, int[] aiEnd ){
		int[] aiUnified = new int[aiStart.length + aiEnd.length];
		int i = 0, iStart = 0, iEnd = 0;
		while( i < aiUnified.length ){
			if( iEnd == aiEnd.length ){
				aiUnified[i] = aiStart[iStart] * 2;
				iStart++;
			}
			else if( iStart == aiStart.length ){
				aiUnified[i] = aiEnd[iEnd] * 2 + 1;
				iEnd++;
			}
			else if( aiStart[iStart] <= aiEnd[iEnd] ){
				aiUnified[i] = aiStart[iStart] * 2;
				iStart++;
			}
			else{
				aiUnified[i] = aiEnd[iEnd] * 2 + 1;
				iEnd++;				
			}
			i++;
		}
		return aiUnified;
	}

	private boolean[] unifyPath( int[] aiStartVariables, int[] aiEndVariables, boolean[] abStartValues, boolean[] abEndValues ){
		boolean[] abUnified = new boolean[aiStartVariables.length + aiEndVariables.length];
		int i = 0, iStart = 0, iEnd = 0;
		while( i < abUnified.length ){
			if( iEnd == aiEndVariables.length ){
				abUnified[i] = abStartValues[iStart];
				iStart++;
			}
			else if( iStart == aiStartVariables.length ){
				abUnified[i] = abEndValues[iEnd];
				iEnd++;
			}
			else if( aiStartVariables[iStart] <= aiEndVariables[iEnd] ){
				abUnified[i] = abStartValues[iStart];
				iStart++;
			}
			else{
				abUnified[i] = abEndValues[iEnd];
				iEnd++;
				
			}
			i++;
		}
		return abUnified;
	}

	
	private void toBits( int iValue, boolean[] abValues ){
		int i = 0;
		for( i = 0 ; i < abValues.length  ; i++ ){
			if( iValue % 2 == 1 )
				abValues[i] = true;
			else
				abValues[i] = false;
			iValue /= 2;
		}
	}

	private void learnIndependentComponentDiagram( int iAction, int iComponent ) {
		int[] aiRelevantVariables = getRelevantVariablesForComponent( iAction, iComponent );
		int[] aiComponent = getIndependentComponentVariables( iComponent );
		int[] aiUnifiedVariables = unifyVariables( aiRelevantVariables, aiComponent );
		boolean[] abRelevantValues = new boolean[aiRelevantVariables.length];
		boolean[] abComponentValues = new boolean[aiComponent.length];
		boolean[] abUnifiedPath = null;
		int cRelevantValues = (int) Math.pow( 2, aiRelevantVariables.length );
		int cComponentValues = (int) Math.pow( 2, aiComponent.length );
		int iRelevantValue = 0, iComponentValue = 0;
		double dTr = 0.0;
		
		AlgebraicDecisionDiagram addTr = newAlgebraicDecisionDiagram( m_cStateVariables * 2, true );

		if( changingComponent( iComponent, iAction, -1 ) ){
			for( iRelevantValue = 0 ; iRelevantValue < cRelevantValues ; iRelevantValue++ ){
				toBits( iRelevantValue, abRelevantValues );
				for( iComponentValue = 0 ; iComponentValue < cComponentValues ; iComponentValue++ ){
					toBits( iComponentValue, abComponentValues );
					dTr = transitionGivenRelevantVariables( iAction, aiComponent, abComponentValues, aiRelevantVariables, abRelevantValues );
					if( dTr != 0.0 ){
						abUnifiedPath = unifyPath( aiRelevantVariables, aiComponent, abRelevantValues, abComponentValues );
						addTr.addPartialPath( aiUnifiedVariables, abUnifiedPath, dTr, false );
					}
				}
			}
		}
		else{ //component not changing - learn the identity diagram
			aiUnifiedVariables = unifyVariables( aiComponent, aiComponent );
			for( iComponentValue = 0 ; iComponentValue < cComponentValues ; iComponentValue++ ){
				toBits( iComponentValue, abComponentValues );
				abUnifiedPath = unifyPath( aiComponent, aiComponent, abComponentValues, abComponentValues );
				addTr.addPartialPath( aiUnifiedVariables, abUnifiedPath, 1.0, false );
			}
		}
		
		addTr.finalizePaths( 0.0 );
				
		addTr.reduce();
		m_adIndependentComponentTransitions[iAction][iComponent] = addTr;
	}

	public static int log( double d ){
		int c = 0;
		while( d > 1 ){
			d = d / 2;
			c++;
		}
		return c;
	}
	
	public boolean useSpecialADDForG(){
		return m_bUseSpecialADDForG;
	}
	
	public AlphaVector newAlphaVector(){
		if( m_btFactored == BeliefType.Factored || m_btFactored == BeliefType.Independent )
			return new FactoredAlphaVector( null, 0, m_cStateVariables, this );
		else
			return new TabularAlphaVector( null, 0, this );
	}

	public void setCompleteAction( int iAction, int iObservation ){
		int iStateVariable = 0;
		m_adCompleteAction[iAction][iObservation] = m_adDualAction[iAction][0].product( m_adDualAction[iAction][1] );
		for( iStateVariable = 2 ; iStateVariable < m_cStateVariables ; iStateVariable++ ){
			m_adCompleteAction[iAction][iObservation] = m_adCompleteAction[iAction][iObservation].product( m_adDualAction[iAction][iStateVariable] );
		}
	}
	
	private boolean[] init( int iSize ){
		boolean[] ab = new boolean[iSize];
		int i = 0;
		for( i = 0 ; i < iSize ; i++ )
			ab[i] = false;
		return ab;
	}
	
	// returns true in case of an overflow
	protected boolean increment( boolean[] ab ){
		int i = 0;
		boolean bCarry = true;
		for( i = 0 ; i < ab.length && bCarry ; i++ ){
			if( ab[i] && bCarry ){
				ab[i] = false;
			}
			else if( ab[i] || bCarry ){
				ab[i] = true;
				bCarry = false;
			}
		}
		return bCarry;
	}
	
	protected double R( int[] aiVariables, boolean[] abValues, int iAction ){
		int iState = partialToFull( aiVariables, abValues );
		return R( iState, iAction );
	}
	
	protected void learnReward( int iAction ){
		boolean[] abState = null;
		int cStates = (int)Math.pow( 2, m_cStateVariables );
		double dR = 0.0;
		AlgebraicDecisionDiagram addReward = newAlgebraicDecisionDiagram( m_cStateVariables, true );
		if( m_bUseRelevantVariablesOnly ){
			int[] aiRelevantVariables = getRewardRelevantVariables( iAction );
			int iState = 0, cRelevantVariables = aiRelevantVariables.length;
			abState = init( cRelevantVariables );
			for( iState = (int)Math.pow( 2, cRelevantVariables ) ; iState > 0 ; iState-- ){
				dR = R( aiRelevantVariables, abState, iAction );
				if( dR != 0 ){
					addReward.addPartialPath( aiRelevantVariables, abState, dR, false );
				}
				increment( abState );
			}
		}
		else{
			abState = init( m_cStateVariables );
			for( int iState : getValidStates() ){
				if( isValid( iState ) ){
					dR = R( iState, iAction );
					if( dR != 0 )
						addReward.addPath( abState, dR );
					increment( abState );
				}
			}
		}
		addReward.finalizePaths( 0.0 );
		addReward.reduce();
		m_adReward[iAction] = addReward;
		Logger.getInstance().logFull( "FactoredPOMDP", 0, "learnReward", "Done learning reward for action " + getActionName( iAction ) +
				" vertexes " + addReward.getVertexCount() );
	}
	
	public AlgebraicDecisionDiagram newAlgebraicDecisionDiagram( int cStateVariables, boolean bAlphaVectorOrBeliefState ) {
		//AffineADD.InitAADDContext( getStateVariablesCount() * 2 );
		//return new AffineADD( cStateVariables ); 
		return new CompactAlgebraicDecisionDiagram( cStateVariables, bAlphaVectorOrBeliefState ); 
		//return new VertexBasedAlgebraicDecisionDiagram( cStateVariables );
	}

	public void learnRewards(){
		int iAction = 0;
		m_adReward = new AlgebraicDecisionDiagram[m_cActions];
		for( iAction = 0 ; iAction < m_cActions ; iAction++ )
			learnReward( iAction );
		Logger.getInstance().logFull( "FactoredPOMDP", 0, "learnRewards", " done learning rewards" );
	}
	
	private AlgebraicDecisionDiagram learnObservation( int iAction, int iObservation ){
		int[] aiVariables = getObservationRelevantVariables( iAction );
		boolean[] abState = init( aiVariables.length );
		int iState = 0, cStates = (int)Math.pow( 2, aiVariables.length );
		double dO = 0.0;
		AlgebraicDecisionDiagram addObservation = newAlgebraicDecisionDiagram( m_cStateVariables, false );
		
		for( iState = 0 ; iState < cStates ; iState++ ){
			toBits( iState, abState );
			dO = observationGivenRelevantVariables( iAction, iObservation, aiVariables, abState );
			addObservation.addPartialPath( aiVariables, abState, dO, false );
		}
		addObservation.reduce();
		//System.out.println( "O( " + iAction + ", " + iObservation + " ) = \n" + addObservation.getTreeString() );
		m_adObservation[iAction][iObservation] = addObservation;
		return m_adObservation[iAction][iObservation];
	}
	
	public void learnObservations(){
		int iAction = 0, iObservation = 0;
		m_adObservation = new AlgebraicDecisionDiagram[m_cActions][m_cObservations];
		for( iAction = 0 ; iAction < m_cActions ; iAction++ )
			for( iObservation = 0 ; iObservation < m_cObservations ; iObservation++ )
				learnObservation( iAction, iObservation );
		Logger.getInstance().logFull( "FactoredPOMDP", 0, "learnObservations", " done learning observations" );
	}
 
	private String getArrayString( boolean[] abState ){
		String sResult = "[";
		int i = 0;
		for( i = 0 ; i < abState.length - 1 ; i++ ){
			if( abState[i] )
				sResult += "T,";
			else
				sResult += "F,";
		}
		if( abState[abState.length - 1] )
			sResult += "T]";
		else
			sResult += "F]";
		return sResult;
	}


	protected int partialToFull( int[] aiIndexes, boolean[] abValues ){
		int iState = 0, iStateVariable = 0, iRelevant = 0;
		for( iStateVariable = 0 ; iStateVariable < aiIndexes.length ; iStateVariable++ ){
			if( abValues[iStateVariable] ){
				iRelevant = 1 << aiIndexes[iStateVariable];
				iState += iRelevant;
			}
		}
		return iState;
	}
	
	protected boolean[] fullToPartial( int iState, int[] aiIndexes ){
		int iStateVariable = 0, iRelevant = 0;
		boolean[] abValues = new boolean[aiIndexes.length];
		for( iStateVariable = 0 ; iStateVariable < aiIndexes.length ; iStateVariable++ ){
			if( ( iState >> aiIndexes[iStateVariable] ) % 2 == 1 )
				abValues[iStateVariable] = true;
			else
				abValues[iStateVariable] = false;
		}
		return abValues;
	}
	
	public Iterator<Entry<boolean[], Double>> getNonZeroTransitions( int[] aiRelevantVariables, 
			boolean[] abStateValues, int iAction ){
		int iStartState = partialToFull( aiRelevantVariables, abStateValues ), iEndState = 0;
		boolean[] abEndState = null;
		double dProb = 0.0;
		Iterator<Entry<Integer,Double>> itNonZeroTransitions = getNonZeroTransitions( iStartState, iAction );
		HashMap<boolean[], Double> mNonZero = new HashMap<boolean[], Double>();
		Entry<Integer,Double> e = null;
		while( itNonZeroTransitions.hasNext() ){
			e = itNonZeroTransitions.next();
			iEndState = e.getKey();
			dProb = e.getValue();
			abEndState = fullToPartial( iEndState, aiRelevantVariables );
			mNonZero.put( abEndState, dProb );
		}
		return mNonZero.entrySet().iterator();
	}
	
	public double O( int iAction, int[] aiRelevantVariables, boolean[] abEndState, int iObservation ){
		int iEndState = stateToIndex( aiRelevantVariables, abEndState );
		return O( iAction, iEndState, iObservation );
	}
	
	public double O( int iAction, boolean[] abEndState, int iObservation ){
		int iEndState = stateToIndex( abEndState ); 
		return O( iAction, iEndState, iObservation );
	}
	
	public Collection<boolean[]> getRelevantStates( int[] aiRelevantVariables ){
		Vector<boolean[]> vRelevantStates = new Vector<boolean[]>();
		int cRelevantVariables = aiRelevantVariables.length;
		int iState = 0;
		boolean[] abStartState = new boolean[cRelevantVariables];
		for( iState = (int)Math.pow( 2, cRelevantVariables ) ; iState > 0 ; iState-- ){
			vRelevantStates.add( abStartState.clone() );
			increment( abStartState );			
		}
		return vRelevantStates;
	}
	
	public void createPartialActionDiagrams( int iAction ){
		int iObservation = 0, cRelevantVertexes = 0, cIrelevantVertexes = 0;
		double dTr = 0.0, dOb = 0.0;
		double[] dSum = new double[m_cObservations];
		Iterator<Entry<boolean[],Double>> itNonZeroTransitions = null;
		Entry<boolean[],Double> e = null;
		int[] aiRelevantVariables = getRelevantVariables( iAction );
		int[] aiIrelevantVariables = getIrelevantVariables( iAction );
		int cRelevantVariables = aiRelevantVariables.length;
		int cIrelevantVariables = aiIrelevantVariables.length;
		boolean[] abRelevant = new boolean[cRelevantVariables * 2],
			abIrelevant = new boolean[cIrelevantVariables * 2];
		boolean[] abEndState = null;
		
		for( iObservation = 0 ; iObservation < m_cObservations ; iObservation++ ){
			m_adRelevantAction[iAction][iObservation] = newAlgebraicDecisionDiagram( m_cStateVariables * 2, true );
			m_adIrelevantAction[iAction] = newAlgebraicDecisionDiagram( m_cStateVariables * 2, true );
		}

		if( cRelevantVariables > 0 ){
			for( boolean[] abStartState : getRelevantStates( aiRelevantVariables) ){
				itNonZeroTransitions = getNonZeroTransitions( aiRelevantVariables, abStartState, iAction );
				while( itNonZeroTransitions.hasNext() ){
					e = itNonZeroTransitions.next();
					abEndState = e.getKey();
					dTr = e.getValue();
					setPath( abStartState, abEndState, abRelevant, abIrelevant, aiRelevantVariables );
					for( iObservation = 0 ; iObservation < m_cObservations ; iObservation++ ){
						dOb = O( iAction, aiRelevantVariables, abEndState, iObservation );
						if( dOb > 0.0 ){
							m_adRelevantAction[iAction][iObservation].addPartialPath( aiRelevantVariables, abRelevant, dTr * dOb, true );
							dSum[iObservation] += dTr * dOb;
						}
					}
				}
				increment( abStartState );
			}
		}
		double dSumProbabilities = 0.0;
		for( iObservation = 0 ; iObservation < m_cObservations ; iObservation++ ){
			if( cRelevantVariables == 0 )
				m_adRelevantAction[iAction][iObservation].finalizePaths( 1.0 );
			else
				m_adRelevantAction[iAction][iObservation].finalizePaths( 0.0 );
			m_adRelevantAction[iAction][iObservation].reduce();
			cRelevantVertexes += m_adRelevantAction[iAction][iObservation].getVertexCount();			
		}
		if( !g_bUseIrrelelvant || cIrelevantVariables == 0 )
			m_adIrelevantAction[iAction].finalizePaths( 1.0 );
		else
			m_adIrelevantAction[iAction].finalizePaths( 0.0 );
		m_adIrelevantAction[iAction].reduce();
		cIrelevantVertexes += m_adIrelevantAction[iAction].getVertexCount();

		Logger.getInstance().logFull( "FactoredPOMDP", 0, "createPartialActionDiagrams", 
				" done learning partial for a " + getActionName( iAction ) +
				", relevant vertex " + cRelevantVertexes +
				", irelevant vertex " + cIrelevantVertexes + 
				", prob sum = " + dSumProbabilities 
				);
	}

/*	
	public void createPartialActionDiagrams( int iAction ){
		int iStateVariable = 0, iEndState = 0, iObservation = 0, cRelevantVertexes = 0, cIrelevantVertexes = 0;
		double dTr = 0.0, dOb = 0.0;
		Iterator<Entry<Integer,Double>> itNonZeroTransitions = null;
		Entry<Integer,Double> e = null;
		int[] aiRelevantVariables = getRelevantVariables( iAction );
		int cRelevantVariables = aiRelevantVariables.length;
		int cIrelevantVariables = m_cStateVariables - cRelevantVariables;
		int[] aiIrelevantVariables = getIrelevantVariables( iAction );
		boolean[] abRelevant = new boolean[cRelevantVariables * 2],
			abIrelevant = new boolean[cIrelevantVariables * 2];
		AlgebraicDecisionDiagram addStateVariableTransition = null, addObservation = null;;
		
		for( iObservation = 0 ; iObservation < m_cObservations ; iObservation++ ){
			m_adRelevantAction[iAction][iObservation] = newAlgebraicDecisionDiagram( m_cStateVariables * 2 );
			m_adIrelevantAction[iAction] = newAlgebraicDecisionDiagram( m_cStateVariables * 2 );
		}
		
		for( iStateVariable = 0 ; iStateVariable < aiRelevantVariables.length ; iStateVariable++ ){
			addStateVariableTransition = learnTransition( iAction, aiRelevantVariables[iStateVariable] );
			System.out.println( addStateVariableTransition.getTreeString() );
			for( iObservation = 0 ; iObservation < m_cObservations ; iObservation++ ){
				addObservation = learnObservation( iAction, iObservation );
				m_adRelevantAction[iAction][iObservation] = m_adRelevantAction[iAction][iObservation].product( addStateVariableTransition );
				m_adRelevantAction[iAction][iObservation] = m_adRelevantAction[iAction][iObservation].product( addObservation );
			}
		}
				
		for( iStateVariable = 0 ; iStateVariable < aiIrelevantVariables.length ; iStateVariable++ ){
			addStateVariableTransition = learnTransition( iAction, aiIrelevantVariables[iStateVariable] );
			m_adIrelevantAction[iAction] = m_adIrelevantAction[iAction].product( addStateVariableTransition );
		}
			
			
			
		for( iObservation = 0 ; iObservation < m_cObservations ; iObservation++ ){
			m_adRelevantAction[iAction][iObservation].finalizePaths( 0.0 );
			m_adRelevantAction[iAction][iObservation].reduce();
			cRelevantVertexes += m_adRelevantAction[iAction][iObservation].getVertexCount();
		}
		m_adIrelevantAction[iAction].finalizePaths( 0.0 );
		m_adIrelevantAction[iAction].reduce();
		cIrelevantVertexes += m_adIrelevantAction[iAction].getVertexCount();

		Logger.getInstance().logFull( "FactoredPOMDP", 0, "createPartialActionDiagrams", 
				" done learning partial for a " + getActionName( iAction ) +
				", avg relevant vertex " + cRelevantVertexes / m_cObservations +
				", irelevant vertex " + cIrelevantVertexes
				);
	}
*/	
	private String toString( boolean[] abRelevant ){
		String s = "[";
		for( boolean b : abRelevant ){
			if( b )
				s += "T";
			else
				s += "F";
		}
		return s + "]";
	}

	public void createCompleteActionDiagrams( int iAction ){
		int iEndState = 0, iObservation = 0, cVertexes = 0, cVertexesForG = 0, cVertexesForBS = 0;
		double dTr = 0.0, dOb = 0.0, dValue = 0.0;
		Iterator<Entry<Integer,Double>> itNonZeroTransitions = null;
		Entry<Integer,Double> e = null;
		boolean[] abPath = new boolean[m_cStateVariables * 2];
		boolean[] abPathForG = new boolean[m_cStateVariables * 2];
		boolean[] abPathForBS = new boolean[m_cStateVariables * 2];

		for( iObservation = 0 ; iObservation < m_cObservations ; iObservation++ ){
			m_adCompleteAction[iAction][iObservation] = newAlgebraicDecisionDiagram( m_cStateVariables * 2, false );
			if( m_bUseSpecialADDForG )
				m_adCompleteActionForG[iAction][iObservation] = newAlgebraicDecisionDiagram( m_cStateVariables * 2, false );
			//m_adCompleteActionForBS[iAction][iObservation] = newAlgebraicDecisionDiagram( m_cStateVariables * 2 );
		}
		
		for( int iStartState : getValidStates() ){
			itNonZeroTransitions = getNonZeroTransitions( iStartState, iAction );
			while( itNonZeroTransitions.hasNext() ){
				e = itNonZeroTransitions.next();
				iEndState = e.getKey();
				dTr = e.getValue();
				setPath( iStartState, iEndState, abPath, 2 );
				setPath( iStartState, iEndState, abPathForG, 0 );
				setPath( iStartState, iEndState, abPathForBS, 1 );
				for( iObservation = 0 ; iObservation < m_cObservations ; iObservation++ ){
					dOb = O( iAction, iEndState, iObservation );
					if( dOb > 0.0 ){
						//if( iAction == 7 )
							//System.out.println( toString( abPath ) + " = " + dTr * dOb );
						m_adCompleteAction[iAction][iObservation].addPath( abPath, dTr * dOb );
						if( m_bUseSpecialADDForG )
							m_adCompleteActionForG[iAction][iObservation].addPath( abPathForG, dTr * dOb );
						//m_adCompleteActionForBS[iAction][iObservation].addPath( abPathForBS, (float)( dTr * dOb ) );
					}
				}
			}
		}
		for( iObservation = 0 ; iObservation < m_cObservations ; iObservation++ ){
			m_adCompleteAction[iAction][iObservation].finalizePaths( 0 );
			m_adCompleteAction[iAction][iObservation].reduce();
			if( m_bUseSpecialADDForG ){
				m_adCompleteActionForG[iAction][iObservation].finalizePaths( 0 );
				m_adCompleteActionForG[iAction][iObservation].reduce();
			}
			//m_adCompleteActionForBS[iAction][iObservation].finalizePaths( 0 );
			//m_adCompleteActionForBS[iAction][iObservation].reduce();
			cVertexes += m_adCompleteAction[iAction][iObservation].getVertexCount();
			if( m_bUseSpecialADDForG )
				cVertexesForG += m_adCompleteActionForG[iAction][iObservation].getVertexCount();
			//cVertexesForBS += m_adCompleteActionForBS[iAction][iObservation].getVertexCount();
		}
		/*
		for( int iStartState = 0 ; iStartState < m_cStates ; iStartState++ ){
			for( iEndState = 0 ; iEndState < m_cStates ; iEndState++ ){
				setPath( iStartState, iEndState, abPath, 2 );
				setPath( iStartState, iEndState, abPathForG, 0 );
				dTr = tr( iStartState, iAction, iEndState );
				for( iObservation = 0 ; iObservation < m_cObservations ; iObservation++ ){
					dOb = O( iAction, iEndState, iObservation );
					if( m_bUseSpecialADDForG )
						dValue = m_adCompleteActionForG[iAction][iObservation].valueAt( abPathForG );
					else
						dValue = m_adCompleteAction[iAction][iObservation].valueAt( abPath );						
					if( Math.abs( dValue - dTr * dOb ) > 0.0001 ){
						System.out.println( m_adCompleteActionForG[iAction][iObservation].getTreeString() );
					}
					dValue = m_adCompleteAction[iAction][iObservation].valueAt( abPath );
					if( Math.abs( dValue - dTr * dOb ) > 0.0001 ){
						System.out.println( m_adCompleteAction[iAction][iObservation].getTreeString() );
					}
				}
			}
		}
		*/
		
		Logger.getInstance().logFull( "FactoredPOMDP", 0, "createCompleteActionDiagrams", 
				" done learning complete for a " + iAction +
				", vertexes " + cVertexes  +
				", vertexes for BS " + cVertexesForBS +
				", vertexes for G " + cVertexesForG 
				);
	}
	

	private void setPath( int iStartState, int iEndState, boolean[] abPath, int iType ){
		int iStateVariable = 0, iBit = 0;
		boolean[] abStartPath = indexToState( iStartState );
		boolean[] abEndPath = indexToState( iEndState );
		for( iStateVariable = 0 ; ( iStateVariable < m_cStateVariables ) ; iStateVariable ++ ){
			if( iType == 0 ){
				abPath[iStateVariable] = abStartPath[iStateVariable];
				abPath[m_cStateVariables + iStateVariable] = abEndPath[iStateVariable];			
			}
			else if( iType == 1 ){
				abPath[iStateVariable] = abEndPath[iStateVariable];
				abPath[m_cStateVariables + iStateVariable] = abStartPath[iStateVariable];			
			}
			else{
				abPath[iBit++] = abEndPath[iStateVariable];
				abPath[iBit++] = abStartPath[iStateVariable];
			}
		}
		
	}

	private void setPath( int iStartState, int iEndState, 
			boolean[] abRelevantPath, boolean[] abIrelevantPath, 
			int[] aiRelevant ){
		int iStateVariable = 0, iRelevant = 0, iIrelevant = 0;
		boolean[] abStartPath = indexToState( iStartState );
		boolean[] abEndPath = indexToState( iEndState );
		for( iStateVariable = 0 ; ( iStateVariable < m_cStateVariables ) ; iStateVariable ++ ){
			if( iRelevant < aiRelevant.length && aiRelevant[iRelevant] == iStateVariable ){
				abRelevantPath[2 * iRelevant] = abEndPath[iStateVariable];
				abRelevantPath[2 * iRelevant + 1] = abStartPath[iStateVariable];
				iRelevant++;
			}
			else{
				abIrelevantPath[2 * iIrelevant] = abEndPath[iStateVariable];
				abIrelevantPath[2 * iIrelevant + 1] = abStartPath[iStateVariable];
				iIrelevant++;
			}
		}		
	}
	
	private void setPath( boolean[] abStartState, boolean[] abEndState, 
			boolean[] abRelevantPath, boolean[] abIrelevantPath, 
			int[] aiRelevant ){
		int iStateVariable = 0, iRelevant = 0, iIrelevant = 0;
		for( iStateVariable = 0 ; ( iStateVariable < m_cStateVariables ) ; iStateVariable ++ ){
			if( iRelevant < aiRelevant.length && aiRelevant[iRelevant] == iStateVariable ){
				abRelevantPath[2 * iRelevant] = abEndState[iRelevant];
				abRelevantPath[2 * iRelevant + 1] = abStartState[iRelevant];
				iRelevant++;
			}
			else{
				abIrelevantPath[2 * iIrelevant] = false;
				abIrelevantPath[2 * iIrelevant + 1] = false;
				iIrelevant++;
			}
		}		
	}

	public void createCompleteActionDiagrams(){
		int iAction = 0;
		m_adCompleteAction = new AlgebraicDecisionDiagram[m_cActions][m_cObservations];
		if( m_bUseSpecialADDForG )
			m_adCompleteActionForG = new AlgebraicDecisionDiagram[m_cActions][m_cObservations];
		else
			m_adCompleteActionForG = null;
		m_adCompleteActionForBS = new AlgebraicDecisionDiagram[m_cActions][m_cObservations];
		for( iAction = 0 ; iAction < m_cActions ; iAction++ )
			createCompleteActionDiagrams( iAction );
		CompactAlgebraicDecisionDiagram.resetFactories();
		System.gc();
		Logger.getInstance().logFull( "FactoredPOMDP", 0, "createCompleteActionDiagrams", " done learning all complete action diagrams" );
	}
	
	public void createPartialActionDiagrams(){
		int iAction = 0;
		m_adRelevantAction = new AlgebraicDecisionDiagram[m_cActions][m_cObservations];
		m_adIrelevantAction = new AlgebraicDecisionDiagram[m_cActions];
		for( iAction = 0 ; iAction < m_cActions ; iAction++ )
			createPartialActionDiagrams( iAction );
		CompactAlgebraicDecisionDiagram.resetFactories();
		System.gc();
		Logger.getInstance().logFull( "FactoredPOMDP", 0, "createPartialActionDiagrams", " done learning all partial action diagrams" );
	}
	
	protected abstract double stateVariableTransition( int[] aiStateVariableIndexes, boolean[] abStateVariableValuesBefore, int iAction, boolean[] abStateVariableValuesAfter );
	protected abstract double stateVariableObservation( int[] aiStateVariableIndexes, int iAction, boolean[] abStateVariableValues, int iObservation );
	protected abstract double stateVariableTransition( int iState, int iAction, int iStateVariable );
	public abstract boolean[] indexToState( int iState );	
	protected abstract int stateToIndex( boolean[] abState );
	protected abstract int stateToIndex( int[] aiStateVariableIndexes, boolean[] abStateVariableValues );
	public abstract int[] getRelevantVariables( int iAction );
	public abstract int[] getRelevantVariablesForComponent( int iAction, int iComponent );
	public abstract int[] getRelevantComponentsForComponent( int iAction, int iComponent );
	public int[] getRelevantVariables( int iAction, int iVariable ){
		return getRelevantVariables( iAction );
	}
	protected abstract int[] getObservationRelevantVariables( int iAction );
	protected abstract int[] getRewardRelevantVariables( int iAction );
	protected int[] getIrelevantVariables( int iAction ){
		int[] aiRelevant = getRelevantVariables( iAction );
		int[] aiIrelevant = new int[m_cStateVariables - aiRelevant.length];
		int iStateVariable = 0, iRelevant = 0, iIrelevant = 0;
		for( iStateVariable = 0 ; iStateVariable < m_cStateVariables ; iStateVariable++ ){
			if( iRelevant < aiRelevant.length && iStateVariable == aiRelevant[iRelevant] )
				iRelevant++;
			else{
				aiIrelevant[iIrelevant] = iStateVariable;
				iIrelevant++;
			}
		}
		return aiIrelevant;
	}

	public AlgebraicDecisionDiagram getRelevantActionDiagram( int iAction, int iObservation ){
		return m_adRelevantAction[iAction][iObservation];
	}

	public AlgebraicDecisionDiagram getIrelevantActionDiagram( int iAction ){
		return m_adIrelevantAction[iAction];
	}

	public AlgebraicDecisionDiagram getCompleteActionDiagram( int iAction, int iObservation ){
		return m_adCompleteAction[iAction][iObservation];
	}

	public AlgebraicDecisionDiagram getCompleteActionDiagramForG( int iAction, int iObservation ){
		return m_adCompleteActionForG[iAction][iObservation];
	}

	public AlgebraicDecisionDiagram getCompleteActionDiagramForBS( int iAction, int iObservation ){
		return m_adCompleteActionForBS[iAction][iObservation];
	}

	//eliminates variables either post-action or pre-action
	public AlgebraicDecisionDiagram existentialAbstraction( AlgebraicDecisionDiagram add, boolean bPreAction ){
		AlgebraicDecisionDiagram addEliminated = null;
		AlgebraicDecisionDiagram.AbstractionFilter aFilter = null;
		if( bPreAction )
			aFilter = new PreActionFilter();
		else
			aFilter = new PostActionFilter();
		//addEliminated = add.existentialAbstractionTopDown( aFilter );
		//System.out.println( add.getTreeString() );
		addEliminated = add.existentialAbstraction( aFilter );
		addEliminated.translateVariables( getVariableReducer() );
		//System.out.println( addEliminated.getTreeString() );
		addEliminated.reduce();
		//System.out.println( addEliminated.getTreeString() );
		return addEliminated;
	}
	
	public AlgebraicDecisionDiagram existentialAbstraction( AlgebraicDecisionDiagram add, boolean bRelevant, boolean bPreAction, int iAction, boolean bTranslate ){
		AlgebraicDecisionDiagram addEliminated = null;
		AlgebraicDecisionDiagram.AbstractionFilter aFilter = null;
		int[] aiVariables = null;
		if( bRelevant )
			aiVariables = getRelevantVariables( iAction );
		else
			aiVariables = getIrelevantVariables( iAction );
		if( bPreAction )
			aFilter = new PreActionFilter( aiVariables, true );
		else
			aFilter = new PostActionFilter( aiVariables, true );
		//addEliminated = add.existentialAbstractionTopDown( aFilter );
		addEliminated = add.existentialAbstraction( aFilter );
		
		//System.out.println( addEliminated.getTreeString() );
		
		if( bTranslate )
			addEliminated.translateVariables( getVariableReducer() );
		
		addEliminated.reduce( null );
		return addEliminated;
	}

	public AlgebraicDecisionDiagram existentialAbstraction( AlgebraicDecisionDiagram add, int iIndependentComponent, boolean bTwoTimeSteps ){
		AlgebraicDecisionDiagram addEliminated = null;
		int[] aiVariables = getIndependentComponentVariables( iIndependentComponent );

		addEliminated = add.existentialAbstraction( new SimpleActionFilter( aiVariables, bTwoTimeSteps ) );
		
		if( bTwoTimeSteps ){
			addEliminated.translateVariables( getVariableReducer() );
			addEliminated.reduce( new SimpleActionFilter( aiVariables, false ) );
		}
		return addEliminated;
	}
/*
	//eliminates variables either post-action or pre-action
	public AlgebraicDecisionDiagram existentialAbstraction( AlgebraicDecisionDiagram add, boolean bPreAction ){
		AlgebraicDecisionDiagram addEliminated = add;
		int iStateVariable = 0, iStateVariableId = 0;
		for( iStateVariable = 0 ; iStateVariable < m_cStateVariables ; iStateVariable++ ){
			if( bPreAction )
				iStateVariableId = iStateVariable * 2 + 1;
			else
				iStateVariableId = iStateVariable * 2;
			addEliminated = addEliminated.existentialAbstraction( iStateVariableId, iStateVariableId );
			//System.out.println( addEliminated.getTreeString() );
		}
		addEliminated.translateVariables( getVariableReducer() );
		addEliminated.reduce();
		return addEliminated;
	}
*/

	public int getStateVariablesCount() {
		return m_cStateVariables;
	}


	public AlgebraicDecisionDiagram getReward( int iAction ) {
		return m_adReward[iAction];
	}

	public AlgebraicDecisionDiagram getDualActionDiagram( int iAction, int iStateVariable) {
		return m_adDualAction[iAction][iStateVariable];
	}

	public AlgebraicDecisionDiagram getObservationDiagram( int iAction, int iObservation ) {
		return m_adObservation[iAction][iObservation];
	}

	public VariableTranslator getPreActionVariableExpander() {
		if( m_vPreExpander == null )
			m_vPreExpander = new VariableExpander( 1 );
		return m_vPreExpander;
	}
	
	public VariableTranslator getPostActionVariableExpander() {
		if( m_vPostExpander == null )
			m_vPostExpander = new VariableExpander( 0 );
		return m_vPostExpander;
	}
	
	public VariableTranslator getRelevantPreActionVariableExpander( int iAction ){
		int[] aiRelevant = getRelevantVariables( iAction );
		return new VariableExpander( aiRelevant, 1 );
	}
		
	public VariableTranslator getRelevantPostActionVariableExpander( int iAction ){
		int[] aiRelevant = getRelevantVariables( iAction );
		return new VariableExpander( aiRelevant, 0 );
	}
		
	public VariableTranslator getIrrelevantPreActionVariableExpander( int iAction ){
		int[] aiRelevant = getIrelevantVariables( iAction );
		return new VariableExpander( aiRelevant, 1 );
	}
		
	public VariableTranslator getIrrelevantPostActionVariableExpander( int iAction ){
		int[] aiRelevant = getIrelevantVariables( iAction );
		return new VariableExpander( aiRelevant, 0 );
	}
		
	public VariableTranslator getVariableReducer() {
		if( m_vReducer == null )
			m_vReducer = new VariableReducer();
		return m_vReducer;
	}
	
	private class VariableExpander implements VariableTranslator{
		private int m_iOffset;
		private int[] m_aiRelevant;
		public VariableExpander( int iOffset ){
			this( null, iOffset );
		}
		public VariableExpander( int[] aiRelevant, int iOffset ) {
			m_aiRelevant = aiRelevant;
			m_iOffset = iOffset;
		}
		public int translate( int iVar ){
			if( expandable( iVar ) ){
				return iVar * 2 + m_iOffset;					
			}
			return iVar * 2 + 1 - m_iOffset;					
		}
		private boolean expandable( int iVar ) {
			if( m_aiRelevant == null )
				return true;
			for( int iExpandable : m_aiRelevant ){
				if( iVar == iExpandable )
					return true;
			}
			return false;
		}
		public int translateVariableCount( int cVariables ){
			return cVariables * 2;
		}		
	}
	private class VariableReducer implements VariableTranslator{
		public VariableReducer(){
		
		}
		public int translate( int iVar ){
			return iVar / 2;
		}
		public int translateVariableCount(int cVariables) {
			return cVariables / 2;
		}		
	}
	private class PreActionFilter implements AlgebraicDecisionDiagram.AbstractionFilter{
		private int[] m_aiVariables;
		boolean m_bAllVariables;
		
		public PreActionFilter(){
			m_aiVariables = null;
			m_bAllVariables = false;
		}
		
		public PreActionFilter( int[] aiVariables, boolean bAllVariables ){
			m_aiVariables = aiVariables;
			m_bAllVariables = bAllVariables;
		}
		
		public boolean abstractVariable( int iVariable ){
			if( m_aiVariables != null ){
				for( int iVar : m_aiVariables )
					if( iVar * 2 + 1 == iVariable )
						return true;
				return false;
			}
			return iVariable % 2 == 1;
		}

		public int firstVariableAfter( int iVariable ) {
			int iNext = iVariable + 1;
			boolean bFound = false;
			if( iNext % 2 == 1 ){
				if( m_aiVariables != null ){
					for( int iVar = 0 ; iVar < m_aiVariables.length ; iVar++ ){
						if( m_aiVariables[iVar] * 2 + 1 == iNext )
							bFound = true;
					}
					if( !bFound )
						iNext = iNext + 1;
				}
			}	
			return iNext;
		}
		
		public int countVariablesBetween( int iVar1, int iVar2 ){
			int cVarsBetween = 0, iVar = 0, i = 0;
			for( iVar = iVar1 ; iVar < iVar2 ; iVar++ ){
				if( iVar % 2 == 0 ){
					cVarsBetween++;
				}
				else{
					for( i = 0 ; i < m_aiVariables.length ; i++ ){
						if( m_aiVariables[i] * 2 + 1 == iVar )
							cVarsBetween++;
					}
				}
			}
			return cVarsBetween;	
		}

		public int countAbstractionVariablesBetween( int iVar1, int iVar2 ){
			int cVarsBetween = 0, iVar = 0, i = 0;
			for( iVar = iVar1 ; iVar < iVar2 ; iVar++ ){
				if( iVar % 2 == 1 ){
					if( m_aiVariables != null ){
						for( i = 0 ; i < m_aiVariables.length ; i++ ){
							if( m_aiVariables[i] * 2 + 1 == iVar )
								cVarsBetween++;
						}
					}
					else{
						cVarsBetween++;
					}
				}
			}
			return cVarsBetween;	
		}

		public int lastVariable() {
			if( m_aiVariables != null ){
				return m_aiVariables[m_aiVariables.length - 1] * 2 + 1;
			}
			return 0;
		}

		public boolean sumMissingLevels() {
			return true;
		}

		public int getFirstVariableId() {
			if( m_bAllVariables )
				return 0;
			else
				return m_aiVariables[0];
		}

		public int getLastVariableId() {
			if( m_bAllVariables )
				return m_cStateVariables;
			else
				return m_aiVariables[m_aiVariables.length - 1] + 1;
		}	
	}
	
	private class PostActionFilter implements AlgebraicDecisionDiagram.AbstractionFilter{
		private int[] m_aiVariables;
		private boolean m_bAllVariables;
		
		public PostActionFilter(){
			m_aiVariables = null;
			m_bAllVariables = false;
		}
		
		public PostActionFilter( int[] aiVariables, boolean bAllVariables ){
			m_aiVariables = aiVariables;
			m_bAllVariables = bAllVariables;
		}

		public boolean abstractVariable( int iVariable ){
			if( m_aiVariables != null ){
				for( int iVar : m_aiVariables )
					if( iVar * 2 == iVariable )
						return true;
				return false;
			}
			return iVariable % 2 == 0;
		}

		public int firstVariableAfter( int iVariable ) {
			/*
			if( m_aiVariables != null ){
				for( int iVar = 0 ; iVar < m_aiVariables.length - 1 ; iVar++ ){
					if( m_aiVariables[iVar] * 2 == iVariable )
						return m_aiVariables[iVar + 1] * 2;
				}
			}
			if( iVariable % 2 == 0 )
				return iVariable + 2;
			return iVariable + 1;
			*/
			int iNext = iVariable + 1;
			boolean bFound = false;
			if( iNext % 2 == 0 ){
				if( m_aiVariables != null ){
					for( int iVar = 0 ; iVar < m_aiVariables.length ; iVar++ ){
						if( m_aiVariables[iVar] * 2 == iNext )
							bFound = true;
					}
					if( !bFound )
						iNext = iNext + 1;
				}
			}	
			return iNext;

		}

		public int lastVariable() {
			if( m_aiVariables != null ){
				return m_aiVariables[m_aiVariables.length - 1] * 2;
			}
			return 0;
		}

		public boolean sumMissingLevels() {
			return false;
		}
		
		public int countVariablesBetween( int iVar1, int iVar2 ){
			int cVarsBetween = 0, iVar = 0, i = 0;
			for( iVar = iVar1 ; iVar < iVar2 ; iVar++ ){
				if( iVar % 2 == 1 ){
					cVarsBetween++;
				}
				else{
					for( i = 0 ; i < m_aiVariables.length ; i++ ){
						if( m_aiVariables[i] * 2 + 1 == iVar )
							cVarsBetween++;
					}
				}
			}
			return cVarsBetween;	
		}

		public int countAbstractionVariablesBetween( int iVar1, int iVar2 ){
			int cVarsBetween = 0, iVar = 0, i = 0;
			for( iVar = iVar1 ; iVar < iVar2 ; iVar++ ){
				if( iVar % 2 == 0 ){
					if( m_aiVariables != null ){
						for( i = 0 ; i < m_aiVariables.length ; i++ ){
							if( m_aiVariables[i] * 2 + 1 == iVar )
								cVarsBetween++;
						}
					}
					else{
						cVarsBetween++;
					}
				}
			}
			return cVarsBetween;	
		}

		public int getFirstVariableId() {
			if( m_bAllVariables )
				return 0;
			else
				return m_aiVariables[0];
		}

		public int getLastVariableId() {
			if( m_bAllVariables )
				return m_cStateVariables;
			else
				return m_aiVariables[m_aiVariables.length - 1] + 1;
		}	

	}

	private class SimpleActionFilter implements AlgebraicDecisionDiagram.AbstractionFilter{
		private int[] m_aiVariables;
		private boolean m_bTwoTimeSteps;
				
		public SimpleActionFilter( int[] aiVariables, boolean bTwoTimeSteps ){
			m_aiVariables = aiVariables;
			m_bTwoTimeSteps = bTwoTimeSteps;
		}

		public boolean abstractVariable( int iVariable ){
			/*
			int i = 0;
			for( i = 0 ; i < m_aiVariables.length ; i++ ){
				if( m_aiVariables[i] * 2 + 1 == iVariable )
					return false;
			}
			return true;
			*/
			int i = 0;
			for( i = 0 ; i < m_aiVariables.length ; i++ ){
				if( m_bTwoTimeSteps ){
					if( m_aiVariables[i] * 2 + 1 == iVariable )
						return false;
				}
				else{
					if( m_aiVariables[i] == iVariable )
						return false;
				}
			}
			return true;				
		}

		public int firstVariableAfter( int iVariable ){
			/*
			if( m_bTwoTimeSteps ){
				int i = 0;
				for( i = 0 ; i < m_aiVariables.length ; i++ ){
					if( m_aiVariables[i] == iVariable / 2 )
						return iVariable + 1;
				}
				return iVariable + 2;				
			}				
			return iVariable + 1;
			*/
			return iVariable + 1;
		}

		public int lastVariable(){
			/*
			if( m_bTwoTimeSteps )
				return m_aiVariables[m_aiVariables.length - 1] * 2;			
			return m_aiVariables[m_aiVariables.length - 1];
			*/
			if( m_bTwoTimeSteps )
				return m_cStateVariables * 2;
			else
				return m_cStateVariables;				
		}

		public boolean sumMissingLevels() {
			throw new NotImplementedException();
		}
		
		public int countVariablesBetween( int iVar1, int iVar2 ){
			return iVar2 - iVar1;	
		}

		public int countAbstractionVariablesBetween( int iVar1, int iVar2 ){
			int cVarsBetween = 0, i = 0;
			for( i = iVar1 ; i < iVar2 ; i++ ){
				if( abstractVariable( i ) )
					cVarsBetween++;
			}
			return cVarsBetween;	
		}

		public int getFirstVariableId() {
			/*
			if( m_bTwoTimeSteps )
				return m_aiVariables[0] * 2;
			return m_aiVariables[0];
			*/
			return 0;
		}

		public int getLastVariableId() {
			/*
			if( m_bTwoTimeSteps )
				return ( m_aiVariables[m_aiVariables.length - 1] + 1 ) * 2;
			return m_aiVariables[m_aiVariables.length - 1] + 1;
			*/
			if( m_bTwoTimeSteps )
				return m_cStateVariables * 2;
			else
				return m_cStateVariables;				
		}
	}

	
	public int countVertexes() {
		int iAction = 0, iObservation = 0;
		int cVertexes = 0;
		for( iAction = 0 ; iAction < m_cActions ; iAction++ ){
			for( iObservation = 0 ; iObservation < m_cObservations ; iObservation++ ){
				if( m_adCompleteAction[iAction][iObservation] != null ){
					//m_adCompleteAction[iAction][iObservation].approve();
					cVertexes += m_adCompleteAction[iAction][iObservation].getVertexCount();
				}
			}
			if( m_adReward[iAction] != null ){
				//m_adReward[iAction].approve();
				cVertexes += m_adReward[iAction].getVertexCount();
			}
		}
		return cVertexes;
	}
	
	public AlgebraicDecisionDiagram relevantActionDiagramProduct( AlgebraicDecisionDiagram add, int iAction, int iObservation, boolean bPreAction ){
		AlgebraicDecisionDiagram.AbstractionFilter aFilter = null;
		AlgebraicDecisionDiagram addRelevant = getRelevantActionDiagram( iAction, iObservation );
		int[] aiVariables = getRelevantVariables( iAction );
		
		//System.out.println( add.getTreeString() );
		//System.out.println( add );

		if( bPreAction )
			add.translateVariables( getRelevantPreActionVariableExpander( iAction ) );
		else
			add.translateVariables( getRelevantPostActionVariableExpander( iAction ) );
			
		if( bPreAction )
			aFilter = new PreActionFilter( aiVariables, true );
		else
			aFilter = new PostActionFilter( aiVariables, true );
		
		//System.out.println( add.getTreeString() );
		//System.out.println( add );
		//System.out.println( addRelevant.getTreeString() );
		//System.out.println( addRelevant );
		
		AlgebraicDecisionDiagram addProduct = null;
		addProduct = add.product( addRelevant );
		
		//System.out.println( addProduct.getTreeString() );
		//System.out.println( addProduct );

		addProduct.reduce( aFilter );
		
		//System.out.println( addProduct.getTreeString() );
		
		add.translateVariables( getVariableReducer() );

		//System.out.println( add.getTreeString() );
		
		return addProduct;
	}
	
	public AlgebraicDecisionDiagram irrelevantActionDiagramProduct( AlgebraicDecisionDiagram add, int iAction, boolean bPreAction ){
		AlgebraicDecisionDiagram.AbstractionFilter aFilter = null;
		AlgebraicDecisionDiagram addIrelevant = getIrelevantActionDiagram( iAction );
		int[] aiVariables = getRelevantVariables( iAction );
		
		//System.out.println( add.getTreeString() );

		if( bPreAction )
			add.translateVariables( getIrrelevantPreActionVariableExpander( iAction ) );
		else
			add.translateVariables( getIrrelevantPostActionVariableExpander( iAction ) );
			
		if( bPreAction )
			aFilter = new PreActionFilter( aiVariables, true );
		else
			aFilter = new PostActionFilter( aiVariables, true );
		
		//System.out.println( add.getTreeString() );
		//System.out.println( addRelevant.getTreeString() );
		
		AlgebraicDecisionDiagram addProduct = add.product( addIrelevant );
		addProduct.reduce( aFilter );
		add.translateVariables( getVariableReducer() );
		//AlgebraicDecisionDiagram addAbstracted = m_pPOMDP.existentialAbstraction( addProduct, true );
		return addProduct;
	}
	
	public boolean isFactored() {
		return m_btFactored == BeliefType.Factored || m_btFactored == BeliefType.Independent;
	}
	
	protected double computeImmediateReward( BeliefState bs, int iAction ){
		if( bs instanceof FactoredBeliefState ){
			FactoredBeliefState fbs = (FactoredBeliefState)bs;
			double dReward = m_adReward[iAction].innerProduct( fbs.getProbabilitiesADD() );
			return dReward;
		}
		if( bs instanceof IndepandantBeliefState ){
			IndepandantBeliefState ibs = (IndepandantBeliefState)bs;
			AlgebraicDecisionDiagram[] addProbs = ibs.getIndependentComponentProbabilities();
			AlgebraicDecisionDiagram addProd = m_adReward[iAction].product( addProbs[0] );
			double dReward = 0.0;
			int iComponent = 0, cComponents = getIndependentComponentsCount();
			
			for( iComponent = 1 ; iComponent < cComponents ; iComponent++ ){
				addProd = addProd.product( addProbs[iComponent] );
			}
			dReward = addProd.getValueSum();
			return dReward;
		}
		return super.computeImmediateReward( bs, iAction );
	}
	
	public void writePOMDP( String sFileName ) throws IOException{
		Logger.getInstance().log( "FactoredPOMDP", 0, "writePOMDP", "Writing " + getName() + " to file" );
		FileWriter writer = new FileWriter( sFileName );
		writer.write( "// " + getName() + "\n" );
		writeVariables( writer );
		writeObservations( writer );
		writeInitialState( writer );
		writeActions( writer );
		//writer.write( "reward\t(Rock0 (v0 (Rock1 (v0 (Rock2 (v0 (1.0)))))))\n" );
		writer.write( "discount " + m_dGamma + "\n" );
		writer.write( "tolerance 0.001\n" );
		writer.close();
		Logger.getInstance().log( "FactoredPOMDP", 0, "writePOMDP", "Finished writing " + getName() + " to file" );
	}

	public void writePOMDPMultiValue( String sFileName ) throws IOException{
		Logger.getInstance().log( "FactoredPOMDP", 0, "writePOMDPMultiValue", "Writing " + getName() + " to file" );
		FileWriter writer = new FileWriter( sFileName );
		writer.write( "// " + getName() + "\n" );
		writeVariablesMultiValue( writer );
		writeObservations( writer );
		writeInitialStateMultiValue( writer );
		writeActionsMultiValue( writer );
		writer.write( "discount " + m_dGamma + "\n" );
		writer.write( "tolerance 0.001\n" );
		writer.close();
		Logger.getInstance().log( "FactoredPOMDP", 0, "writePOMDP", "Finished writing " + getName() + " to file" );
	}

	protected abstract int getRealStateVariableCount();
	protected abstract int getValueCount( int iVariable );
	protected abstract String getRealVariableName( int iVariable );
	protected abstract String getValueName( int iVariable, int iValue );
	protected abstract double getInitialVariableValueProbability( int iVariable, int iValue );
	protected abstract boolean relevantTransitionRealVariable( int iAction, int iVariable );
	
	protected abstract int[] getRelevantVariablesMultiValue( int iAction, int iVariable );	
	protected abstract int[] getObservationRelevantVariablesMultiValue( int iAction );
	protected abstract int[] getRewardRelevantVariablesMultiValue( int iAction );
	protected abstract double transitionGivenRelevantVariablesMultiValue( int iAction, int iVariable, int iValue, int[] aiRelevantVariables, int[] atValues );
	protected abstract double observationGivenRelevantVariablesMultiValue( int iAction, int iObservation, int[] aiRelevantVariables, int[] aiValues );
	protected abstract double rewardGivenRelevantVariablesMultiValue( int iAction, int[] aiRelevantVariables, int[] aiValues );
	
	private void writeInitialStateMultiValue( FileWriter writer ) throws IOException{
		int iVariable = 0, iValue = 0, cValues = 0, cStateVariables = 0;
		writer.write( "// Initial belief over state variable values \n" );
		writer.write( "init [* \n" );
		cStateVariables = getRealStateVariableCount();
		for( iVariable = 0 ; iVariable < cStateVariables ; iVariable++ ){
			writer.write( "\t(" + getRealVariableName( iVariable ) );
			cValues = getValueCount( iVariable );
			for( iValue = 0 ; iValue < cValues ; iValue++ ){
				writer.write( " (" + getValueName( iVariable, iValue ) + "(" + getInitialVariableValueProbability( iVariable, iValue ) + "))" );
			}
			writer.write( ")\n" );
		}
		writer.write( "]\n" );
	}
	
	private void writeInitialState( FileWriter writer ) throws IOException{
		int iVariable = 0;
		writer.write( "// Initial belief over state variable values \n" );
		writer.write( "init [* \n" );
		for( iVariable = 0 ; iVariable < m_cStateVariables ; iVariable++ ){
			writer.write( "\t(" + getVariableName( iVariable ) + 
					" (v0 (" + getInitialVariableValueProbability( iVariable, false ) + "))" +
					" (v1 (" + getInitialVariableValueProbability( iVariable, true ) + ")))" + "\n" );
		}
		writer.write( "]\n" );
	}
	
	protected String getTransitionCPT( int iAction, int iVariable, 
			int[] aiRelevantVariables, int iCurrentVariable, boolean[] abValues ){
		String sResult = "";
		double dTr = 0.0;
		if( iCurrentVariable == aiRelevantVariables.length ){
			dTr = transitionGivenRelevantVariables( iAction, iVariable, true, aiRelevantVariables, abValues );
			sResult = "(" + getVariableName( iVariable ) + "' ";
			sResult += "(v0 (" + ( 1 - dTr ) + ")) (v1 (" + dTr + ")))";
		}
		else{
			sResult = "(" + getVariableName( aiRelevantVariables[iCurrentVariable] );
			abValues[iCurrentVariable] = false;
			sResult += "\t(v0 " + getTransitionCPT( iAction, iVariable, aiRelevantVariables, iCurrentVariable + 1, abValues ) + ") ";
			abValues[iCurrentVariable] = true;
			sResult += "\t(v1 " + getTransitionCPT( iAction, iVariable, aiRelevantVariables, iCurrentVariable + 1, abValues ) + "))";
		}
		return sResult;
	}
	
	protected String getTransitionCPTMultiValue( int iAction, int iVariable, 
			int[] aiRelevantVariables, int iCurrentVariable, int[] aiValues ){
		String sResult = "";
		int cValues = 0, iValue = 0;
		double dTr = 0.0;
		if( iCurrentVariable == aiRelevantVariables.length ){
			sResult = "(" + getRealVariableName( iVariable ) + "'";
			cValues = getValueCount( iVariable );
			for( iValue = 0 ; iValue < cValues ; iValue++ ){
				dTr = transitionGivenRelevantVariablesMultiValue( iAction, iVariable, iValue, aiRelevantVariables, aiValues );
				sResult += " (" + getValueName( iVariable, iValue ) + " (" + dTr + "))";
			}
			sResult += ")";
		}
		else{
			sResult = "(" + getRealVariableName( aiRelevantVariables[iCurrentVariable] );
			cValues = getValueCount( aiRelevantVariables[iCurrentVariable] );
			for( iValue = 0 ; iValue < cValues ; iValue++ ){
				aiValues[iCurrentVariable] = iValue;
				sResult += " (" + getValueName( aiRelevantVariables[iCurrentVariable], iValue ) + " " + 
					getTransitionCPTMultiValue( iAction, iVariable, aiRelevantVariables, iCurrentVariable + 1, aiValues ) + ")";
			}
			sResult += ")";
		}
		return sResult;
	}
	
	protected String getTransitionCPT( int iAction, int iVariable ){
		int[] aiRelevantVariables = getRelevantVariables( iAction, iVariable );
		int cRelevantVariables = aiRelevantVariables.length;
		boolean[] abValues = new boolean[cRelevantVariables];
		
		return getTransitionCPT( iAction, iVariable, aiRelevantVariables, 0, abValues );
	}

	protected String getTransitionCPTMultiValue( int iAction, int iVariable ){
		int[] aiRelevantVariables = getRelevantVariablesMultiValue( iAction, iVariable );
		int cRelevantVariables = aiRelevantVariables.length;
		int[] aiValues = new int[cRelevantVariables];
		
		return getTransitionCPTMultiValue( iAction, iVariable, aiRelevantVariables, 0, aiValues );
	}

	protected String getObservationCPTMultiValue( int iAction, int[] aiRelevantVariables, int iCurrentVariable, int[] aiValues ){
		String sResult = "";
		int iValue = 0, cValues = 0;
		double dPr = 0.0;
		int iObservation = 0;
		if( iCurrentVariable == aiRelevantVariables.length ){
			sResult = "(" + getObservationName() + "' ";
			for( iObservation = 0 ; iObservation < m_cObservations ; iObservation++ ){
				dPr = observationGivenRelevantVariablesMultiValue( iAction, iObservation, aiRelevantVariables, aiValues );
				sResult += "(" + getObservationName( iObservation ) + " (" + dPr + ")) ";
			}
			sResult += ")";
		}
		else{
			cValues = getValueCount( aiRelevantVariables[iCurrentVariable] );
			sResult = "(" + getRealVariableName( aiRelevantVariables[iCurrentVariable] ) + "'";
			for( iValue = 0 ; iValue < cValues ; iValue++ ){
				aiValues[iCurrentVariable] = iValue;
				sResult += " (" + getValueName( aiRelevantVariables[iCurrentVariable], iValue ) + " " + 
					getObservationCPTMultiValue( iAction, aiRelevantVariables, iCurrentVariable + 1, aiValues ) + ")";
			}
			sResult += ")";
		}
		return sResult;
	}
	
	protected String getObservationCPT( int iAction, int[] aiRelevantVariables, int iCurrentVariable, boolean[] abValues ){
		String sResult = "";
		double dPr = 0.0;
		int iObservation = 0;
		if( iCurrentVariable == aiRelevantVariables.length ){
			sResult = "(" + getObservationName() + "' ";
			for( iObservation = 0 ; iObservation < m_cObservations ; iObservation++ ){
				dPr = observationGivenRelevantVariables( iAction, iObservation, aiRelevantVariables, abValues );
				sResult += "(" + getObservationName( iObservation ) + " (" + dPr + ")) ";
			}
			sResult += ")";
		}
		else{
			sResult = "(" + getVariableName( aiRelevantVariables[iCurrentVariable] ) + "'";
			abValues[iCurrentVariable] = false;
			sResult += "\t(v0 " + getObservationCPT( iAction, aiRelevantVariables, iCurrentVariable + 1, abValues ) + ") ";
			abValues[iCurrentVariable] = true;
			sResult += "\t(v1 " + getObservationCPT( iAction, aiRelevantVariables, iCurrentVariable + 1, abValues ) + "))";
		}
		return sResult;
	}
	
	protected String getObservationCPT( int iAction ){
		int[] aiRelevantVariables = getObservationRelevantVariables( iAction );
		int cRelevantVariables = aiRelevantVariables.length;
		boolean[] abValues = new boolean[cRelevantVariables];
		
		return getObservationCPT( iAction, aiRelevantVariables, 0, abValues );
	}
	protected String getObservationCPTMultiValue( int iAction ){
		int[] aiRelevantVariables = getObservationRelevantVariablesMultiValue( iAction );
		int cRelevantVariables = aiRelevantVariables.length;
		int[] aiValues = new int[cRelevantVariables];
		
		return getObservationCPTMultiValue( iAction, aiRelevantVariables, 0, aiValues );
	}
	protected String getCostTable( int iAction, int[] aiRelevantVariables, int iCurrentVariable, boolean[] abValues ){
		String sResult = "";
		double dCost = 0.0;
		if( iCurrentVariable == aiRelevantVariables.length ){
			dCost = -1.0 * rewardGivenRelevantVariables( iAction, aiRelevantVariables, abValues );
			sResult = "(" + dCost + ")";
		}
		else{
			sResult = "(" + getVariableName( aiRelevantVariables[iCurrentVariable] );// + "'";
			abValues[iCurrentVariable] = false;
			sResult += "\t(v0 " + getCostTable( iAction, aiRelevantVariables, iCurrentVariable + 1, abValues ) + ") ";
			abValues[iCurrentVariable] = true;
			sResult += "\t(v1 " + getCostTable( iAction, aiRelevantVariables, iCurrentVariable + 1, abValues ) + "))";
		}
		return sResult;
	}
	
	protected String getCostTableMultiValue( int iAction, int[] aiRelevantVariables, int iCurrentVariable, int[] aiValues ){
		String sResult = "";
		int iValue = 0 , cValues = 0;
		double dCost = 0.0;
		if( iCurrentVariable == aiRelevantVariables.length ){
			dCost = -1.0 * rewardGivenRelevantVariablesMultiValue( iAction, aiRelevantVariables, aiValues );
			sResult = "(" + dCost + ")";
		}
		else{
			cValues = getValueCount( aiRelevantVariables[iCurrentVariable] );
			sResult = "(" + getRealVariableName( aiRelevantVariables[iCurrentVariable] );// + "'";
			for( iValue = 0 ; iValue < cValues ; iValue++ ){
				aiValues[iCurrentVariable] = iValue;
				sResult += " (" + getValueName( aiRelevantVariables[iCurrentVariable], iValue ) + " " + 
					 getCostTableMultiValue( iAction, aiRelevantVariables, iCurrentVariable + 1, aiValues ) + ") ";
			}
			sResult += ")";
		}
		return sResult;
	}
	
	protected String getCostTable( int iAction ){
		int[] aiRelevantVariables = getRewardRelevantVariables( iAction );
		int cRelevantVariables = aiRelevantVariables.length;
		boolean[] abValues = new boolean[cRelevantVariables];
		
		return getCostTable( iAction, aiRelevantVariables, 0, abValues );
	}

	protected String getCostTableMultiValue( int iAction ){
		int[] aiRelevantVariables = getRewardRelevantVariablesMultiValue( iAction );
		int cRelevantVariables = aiRelevantVariables.length;
		int[] aiValues = new int[cRelevantVariables];
		
		return getCostTableMultiValue( iAction, aiRelevantVariables, 0, aiValues );
	}

	protected boolean getVariableValue( int[] aiVariables, boolean[] abValues, int iVariable ){
		for( int i = 0 ; i < aiVariables.length ; i++ )
			if( aiVariables[i] == iVariable )
				return abValues[i];
		return false;
	}
	
	protected int getVariableValue( int[] aiVariables, int[] aiValues, int iVariable ){
		for( int i = 0 ; i < aiVariables.length ; i++ )
			if( aiVariables[i] == iVariable )
				return aiValues[i];
		return -1;
	}
	
	public abstract double rewardGivenRelevantVariables( int iAction, int[] aiRelevantVariables, boolean[] abValues );
	public abstract double transitionGivenRelevantVariables( int iAction, int iVariable, boolean bValue, int[] aiRelevantVariables, boolean[] abValues );
	public abstract double observationGivenRelevantVariables( int iAction, int iObservation, int[] aiRelevantVariables, boolean[] abValues );
	
	private void writeActionMultiValue( FileWriter writer, int iAction ) throws IOException{
		int iVariable = 0, cStateVariables = 0;
		writer.write( "// Action - " + getActionName( iAction ) + " \n" );
		writer.write( "action " + getActionName( iAction ) + " \n" );
		cStateVariables = getRealStateVariableCount();
		for( iVariable = 0 ; iVariable < cStateVariables ; iVariable++ ){
			writer.write( "\t" + getRealVariableName( iVariable ) + " " );
			if( !relevantTransitionRealVariable( iAction, iVariable) )
				writer.write( "(SAME" + getRealVariableName( iVariable ) + ")" );
			else
				writer.write( getTransitionCPTMultiValue( iAction, iVariable ) );
			writer.write( "\n" );
		}
		writer.write( "\tobserve \n\t\t" + getObservationName() + " " + getObservationCPTMultiValue( iAction ) + "\n" );
		writer.write( "\tendobserve \n" );		
		writer.write( "\tcost \t" + getCostTableMultiValue( iAction ) + "\n" );
		writer.write( "endaction \n" );
	}
	
	private void writeAction( FileWriter writer, int iAction ) throws IOException{
		int iVariable = 0;
		writer.write( "// Action - " + getActionName( iAction ) + " \n" );
		writer.write( "action " + getActionName( iAction ) + " \n" );
		for( iVariable = 0 ; iVariable < m_cStateVariables ; iVariable++ ){
			writer.write( "\t" + getVariableName( iVariable ) + " " );
			if( !relevantTransitionVariable( iAction, iVariable) )
				writer.write( "(SAME" + getVariableName( iVariable ) + ")" );
			else
				writer.write( getTransitionCPT( iAction, iVariable ) );
			writer.write( "\n" );
		}
		writer.write( "\tobserve \n\t\t" + getObservationName() + " " + getObservationCPT( iAction ) + "\n" );
		writer.write( "\tendobserve \n" );		
		writer.write( "\tcost \t" + getCostTable( iAction ) + "\n" );
		writer.write( "endaction \n" );
	}
	protected abstract boolean relevantTransitionVariable( int iAction, int iVariable );
	
	private void writeActionsMultiValue( FileWriter writer ) throws IOException{
		int iAction = 0;
		writer.write( "// Actions \n" );
		
		for( iAction = 0 ; iAction < m_cActions ; iAction++ ){
			writeActionMultiValue( writer, iAction );
		}
		writer.write( "\n" );
	}
	private void writeActions( FileWriter writer ) throws IOException{
		int iAction = 0;
		writer.write( "// Actions \n" );
		
		for( iAction = 0 ; iAction < m_cActions ; iAction++ ){
			writeAction( writer, iAction );
		}
		writer.write( "\n" );
	}
	private void writeVariablesMultiValue( FileWriter writer ) throws IOException{
		int iVariable = 0, iValue = 0, cValues = 0, cStateVariables = 0;
		writer.write( "// Variables \n" );
		writer.write( "(variables \n" );
		cStateVariables = getRealStateVariableCount();
		for( iVariable = 0 ; iVariable < cStateVariables ; iVariable++ ){
			writer.write( "\t(" + getRealVariableName( iVariable ) );
			cValues = getValueCount( iVariable );
			for( iValue = 0 ; iValue < cValues ; iValue++ ){
				writer.write( " " + getValueName( iVariable, iValue ) );
			}
			writer.write( ")\n" );
		}
		writer.write( ")\n" );
	}
	private void writeVariables( FileWriter writer ) throws IOException{
		int iVariable = 0;
		writer.write( "// Variables \n" );
		writer.write( "(variables \n" );
		for( iVariable = 0 ; iVariable < m_cStateVariables ; iVariable++ ){
			writer.write( "\t(" + getVariableName( iVariable ) + " v0 v1)\n" );
		}
		writer.write( ")\n" );
	}
	private void writeObservations( FileWriter writer ) throws IOException{
		int iObservation = 0;
		writer.write( "// Observations \n" );
		writer.write( "(observations (" + getObservationName() );
		for( iObservation = 0 ; iObservation < m_cObservations ; iObservation++ ){
			writer.write( " " + getObservationName( iObservation ) );
		}
		writer.write( "))\n" );
	}

	protected abstract String getVariableName( int iVariable );
	public abstract double getInitialVariableValueProbability( int iVariable, boolean bValue );
	protected abstract String getObservationName();
	
	public void writeADDs( String sFileName ) throws IOException{
		FileWriter fw = new FileWriter( sFileName );
		int iAction = 0, iObservation = 0;
		fw.write( "<?xml version=\"1.0\" encoding=\"UTF-8\" ?>" );
		fw.write( "<POMDP name = \"" + getName() + 
				"\" Discount = \"" + m_dGamma + 
				"\" StateVariableCount = \"" + m_cStateVariables + 
				"\" StateCount = \"" + m_cStates + 
				"\" ActionCount = \"" + m_cActions +
				"\" ObservationCount = \"" + m_cObservations + "\">" );
		for( iAction = 0 ; iAction < m_cActions ; iAction++ ){
			fw.write( "<Action index = \"" + iAction + "\">" );
			fw.write( "<Reward>" );
			getReward( iAction ).save( fw );
			fw.write( "</Reward>" );
			for( iObservation = 0 ; iObservation < m_cObservations ; iObservation++ ){
				fw.write( "<Observation index = \"" + iObservation + "\">" );
				getRelevantActionDiagram( iAction, iObservation ).save( fw );
				fw.write( "</Observation>" );
			}
			fw.write( "</Action>" );
		}
		fw.write( "</POMDP>" );
		fw.close();
	}
	
	/**
	 * Loads the model description from an XML file.
	 */
	public boolean readADDs( String sFileName ) throws Exception{
		DocumentBuilder builder = DocumentBuilderFactory.newInstance().newDocumentBuilder();
		Document docValueFunction = builder.parse( new FileInputStream( sFileName ) );
		Element eModel = null, eAction = null;
		NodeList nlActions = null;
		int cStates = 0, cActions = 0, cObservations = 0, iActionItem = 0, iAction = 0, cStateVariables = 0;
		
		Logger.getInstance().log( "FactoredPOMDP", 0, "readADDs", "Started reading ADDs from file " + sFileName );
		
		
		eModel = (Element)docValueFunction.getFirstChild();
		cStateVariables = Integer.parseInt( eModel.getAttribute( "StateVariableCount" ) );
		cStates = Integer.parseInt( eModel.getAttribute( "StateCount" ) );
		cActions = Integer.parseInt( eModel.getAttribute( "ActionCount" ) );
		cObservations = Integer.parseInt( eModel.getAttribute( "ObservationCount" ) );
		
		Logger.getInstance().log( "FactoredPOMDP", 0, "readADDs", "Model definitions - state variables " + cStateVariables + ", states = " + cStates + ", actions = " + cActions + ", observations = " + cObservations );

		m_adReward = new AlgebraicDecisionDiagram[cActions];
		m_adRelevantAction = new AlgebraicDecisionDiagram[cActions][cObservations];
		
		nlActions = eModel.getChildNodes();
		
		for( iActionItem = 0 ; iActionItem < nlActions.getLength() ; iActionItem++ ){
			eAction = (Element)nlActions.item( iActionItem );
			iAction = Integer.parseInt( eAction.getAttribute( "index" ) );
			readActionADDs( eAction, iAction );
		}
		
		return true;
	}

	private void readActionADDs( Element eAction, int iAction ){
		NodeList nlChildren = eAction.getChildNodes();
		Element eChild = null;
		int iChild = 0, iObservation = 0;
		for( iChild = 0 ; iChild < nlChildren.getLength() ; iChild++ ){
			eChild = (Element) nlChildren.item( iChild );
			if( eChild.getNodeName().equals( "Reward" ) ){
				m_adReward[iAction] = newAlgebraicDecisionDiagram( m_cStateVariables, true ); 
				m_adReward[iAction].parseXML( (Element) eChild.getFirstChild() );
				Logger.getInstance().logFull( "FactoredPOMDP", 0, "readActionADDs", "Read Reward ADD for action " + getActionName( iAction ) );
			}
			else if( eChild.getNodeName().equals( "Observation" ) ){
				iObservation = Integer.parseInt( eChild.getAttribute( "index" ) );
				m_adRelevantAction[iAction][iObservation] = newAlgebraicDecisionDiagram( m_cStateVariables, true ); 
				m_adRelevantAction[iAction][iObservation].parseXML( (Element) eChild.getFirstChild() );
				//m_adRelevantAction[iAction][iObservation].reduceToMin( m_dEpsilon );	
				Logger.getInstance().logFull( "FactoredPOMDP", 0, "readActionADDs", "Read partial ADD for action " + getActionName( iAction ) + ", observation " + iObservation + " size = " + m_adRelevantAction[iAction][iObservation].getVertexCount() );
			}
		}
	}
	
	public BeliefType getBeliefType(){
		return m_btFactored;
	}

	public abstract int getIndependentComponentsCount();
	public abstract int[] getIndependentComponentVariables( int iComponent );
	//public abstract double rewardGivenRelevantVariables( int iAction, int[] aiRelevantVariables, boolean[] abValues );
	public abstract double transitionGivenRelevantVariables( int iAction, int[] aiComponent, boolean[] abComponentValues, int[] aiRelevantVariables, boolean[] abRelevantValues );
	public abstract double getInitialComponenetValueProbability( int iComponent, int iValue );
	public abstract boolean changingComponent( int iComponent, int iAction, int iObservation );
	public abstract int[] getRelevantComponents( int iAction, int iObservation );

	public AlgebraicDecisionDiagram getIndependentComponentTransitionDiagram( int iAction, int iComponent ){
		return m_adIndependentComponentTransitions[iAction][iComponent];
	}
	public void initBeliefStateFactory() {
		if( m_btFactored == BeliefType.Factored ){
			System.out.println( "Using factored representaiton" );
			m_bsFactory = new FactoredBeliefStateFactory( this );
		}
		else if( m_btFactored == BeliefType.Flat ){
			System.out.println( "Using flat representaiton" );
			m_bsFactory = new BeliefStateFactory( this );
		}
		else if( m_btFactored == BeliefType.Independent ){
			System.out.println( "Using factored independent representaiton" );
			m_bsFactory = new IndependenBeliefStateFactory( this );
		}
	}
	public double getMaxMinR(){
		return 0.0;
	}

	public Vector<int[][]> getConsistentStates( int[][] aiState ){
		return null;
	}
	
	//Methods for the restricted POMDP implementation - allow scaling up beyond MAX_INT
	protected boolean[] toBool( int[] aiValues ){
		boolean[] abValues = new boolean[aiValues.length];
		int iVar = 0;
		for( iVar = 0 ; iVar < aiValues.length ; iVar++ ){
			abValues[iVar] = aiValues[iVar] == 1;
		}
		return abValues;
	}

	protected int[] toInt( boolean[] abValues ){
		int[] aiValues = new int[abValues.length];
		int iVar = 0;
		for( iVar = 0 ; iVar < abValues.length ; iVar++ ){
			if( abValues[iVar] )
				aiValues[iVar] = 1;
			else
				aiValues[iVar] = 0;
		}
		return aiValues;
	}

	protected boolean equals( int[] a1, int[] a2 ){
		if( a1.length != a2.length )
			return false;
		int i = 0;
		for( i = 0 ; i < a1.length ; i++ )
			if( a1[i] != a2[i] )
				return false;
		return true;
	}
	
	protected boolean equals( boolean[] a1, boolean[] a2 ){
		if( a1.length != a2.length )
			return false;
		int i = 0;
		for( i = 0 ; i < a1.length ; i++ )
			if( a1[i] != a2[i] )
				return false;
		return true;
	}
	
	public int[] indexToState2( int iState ) {
		int[] abValues = new int[m_cStateVariables];
		int iVar = 0;
		for( iVar = 0 ; iVar < m_cStateVariables ; iVar++ ){
			abValues[iVar ] = iState % 2;
			iState /= 2;
		}
		return abValues;
	}
	
	public int[][] getState2( int[] aiVariables, boolean[] abValues ){
		return new int[][]{ aiVariables, toInt( abValues ) };
	}
	
	protected class State2Comparator implements Comparator<int[][]>{
		
		public int compare( int[] a1, int[] a2 ){
			if( a1.length != a2.length )
				return a1.length - a2.length;
			int i = 0;
			for( i = 0 ; i < a1.length ; i++ )
				if( a1[i] != a2[i] )
					return a1[i] - a2[i];
			return 0;
		}
		
		@Override
		public int compare(int[][] a1, int[][] a2) {
			int iCompare = compare( a1[0], a2[0] );
			if( iCompare == 0 )
				iCompare = compare( a1[1], a2[1] );
			return iCompare;
		}
	}
	
	public abstract int[][] execute(int iAction, int[][] aiState);
	public abstract int observe(int iAction, int[][] aiState);
	public abstract double probStartState( int[][] aiState );
	public abstract double probStartState( int[][] aiState, int iStartOption );
	public abstract double R(int[][] aiState, int iAction);
	public abstract double O(int iAction, int[][] aiState, int observation);
	public abstract double tr(int[][] aiStartState, int iAction, int[][] aiEndState);
	public abstract boolean isTerminalState(int[][] aiState);
	public abstract String getStateName(int[][] aiState);
	public abstract Iterator<Entry<int[][], Double>> getNonZeroTransitions( int[][] aiState, int iAction );
	public abstract int[][] chooseStartState2();
	
	
	
	public double computeDiscountedReward( int cMaxStepsToGoal, PolicyStrategy policy, Vector<BeliefState> vObservedBeliefPoints, boolean bExplore, int[] aiActionCount ){
		double dDiscountedReward = 0.0, dCurrentReward = 0.0, dDiscountFactor = 1.0;
		int iStep = 0, iAction = 0, iObservation = 0;
		int[][] aiState = chooseStartState2(), aiNextState = null;
		BeliefState bsCurrentBelief = getBeliefStateFactory().getInitialBeliefState(), bsNext = null;
		//BeliefState bsCurrentBelief = new LogisticsBeliefState( (Logistics)this, null ), bsNext = null;
		
		boolean bDone = false;
		int cRewards = 0;
		int cSameStates = 0;
		
		Vector<String> vPath = new Vector<String>();
		
		for( iStep = 0 ; ( iStep < cMaxStepsToGoal ) && !bDone && bsCurrentBelief != null ; iStep++ ){
			
			
			iAction = policy.getAction( bsCurrentBelief );
			
			//String s = getActionName( iAction );
			//if( s.startsWith( "CheckRock" ) )
			//	System.out.println( s + " - " + bsCurrentBelief );
			
			if( iAction == -1 )
				throw new Error( "Could not find optimal action for bs " + bsCurrentBelief );
		
			if( iAction == -1 )
				return Double.NEGATIVE_INFINITY;
			
			if( aiActionCount != null )
				aiActionCount[iAction]++;
			
			if( vObservedBeliefPoints != null ){
				bsCurrentBelief.setFactoryPersistence( true );
				vObservedBeliefPoints.add( bsCurrentBelief );
			}
			
			aiNextState = execute( iAction, aiState );
			iObservation = observe( iAction, aiNextState );
			
			
			if( m_rtReward == RewardType.StateAction )
				dCurrentReward = R( aiState, iAction ); //R(s,a)
			/* - not implemented currently
			else if( m_rtReward == RewardType.StateActionState )
				dCurrentReward = R( aiState, iAction, aiNextState ); //R(s,a,s')
			else if( m_rtReward == RewardType.State )
				dCurrentReward = R( iState );
				*/
			dDiscountedReward += dCurrentReward * dDiscountFactor;
			dDiscountFactor *= m_dGamma;
			
			if( dCurrentReward != 0 )
				cRewards++;

			bDone = isTerminalState( aiNextState );
			if( bDone )
				dDiscountFactor = 0.0;
			
			bsNext = bsCurrentBelief.nextBeliefState( iAction, iObservation );
			 
			bsCurrentBelief.release();
			bsCurrentBelief = bsNext;	
			
			aiState = aiNextState;
			
			
			String sInfo = getStateName( aiState ) + ", " + getActionName( iAction ) + ", " + iObservation + ", " + bsCurrentBelief;
			vPath.add( sInfo );

		}	
	/*
		System.out.println( "****************************" );
		for( String s : vPath ){
			System.out.println( s );
		}
		*/
		return dDiscountedReward;// + m_dMinReward * ( 1 / ( 1 - dDiscountFactor ) );
	}

	
}
