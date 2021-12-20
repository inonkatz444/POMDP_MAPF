package pomdp.utilities.factored;

import java.io.FileWriter;
import java.io.IOException;
import java.util.*;

import org.w3c.dom.Document;
import org.w3c.dom.Element;

import pomdp.utilities.factored.affine.AADD;
import pomdp.utilities.factored.affine.DD;
import sun.reflect.generics.reflectiveObjects.NotImplementedException;

public class AffineADD implements AlgebraicDecisionDiagram {

	public final static ArrayList EMPTY_ARRAY = new ArrayList();
	
	// An AADD is given by its node ID
	public int _nodeID = -1;
	
	// The context maintains the caches and must be initialized
	public int  MAX_VARS = -1;
	public static AADD _context = null;
	public static int  _zero = -1;
	
	
	// Must pre-specify the maximum number of variables when 
	// initializing a context
	public static void InitAADDContext(int max_vars) {
		if (_context != null) {
			return;
		} 
		//MAX_VARS = max_vars;
		ArrayList order = new ArrayList();
		for (int i = 0; i < max_vars; i++) {
			order.add(i);
		}
		_context = new AADD(order);
		_zero = _context.getConstantNode(0d);
	}
/*	
	// Default AADD is constant 0 function
	public AffineADD() {
		if (_context == null) {
			System.err.println("AADD Error - Constructor: no context initialized");
			System.exit(1);
		}
		
		// This AADD is really a node within larger AADD
		_nodeID = _zero;
	}
*/
	
	public AffineADD( int cVars ){
		_nodeID = _zero;
		MAX_VARS = cVars;
	}
	
	// Copy constructor
	public AffineADD(AffineADD src) {
		
		// This AADD is really a node within larger AADD
		this._nodeID = src._nodeID;
	}

	
	@Override
	public void addPartialPath(int[] aiVariables, boolean[] abValues,
			double value, boolean twoTimeSteps) {
		int temp_aadd = buildPartialPath(0, aiVariables, abValues, value);
		_nodeID = _context.applyInt(_nodeID, temp_aadd, DD.ARITH_SUM);
	}

	protected int buildPartialPath(int index, int[] aiVariables, boolean[] abValues, double value) {
		if (index == aiVariables.length) 
			return _context.getConstantNode(value);
		else 
			return _context.applyInt(
					_context.getVarNode(aiVariables[index], abValues[index] ? 0d : 1d, abValues[index] ? 1d : 0d), 
					buildPartialPath(index+1, aiVariables, abValues, value), 
					DD.ARITH_PROD);
	}

	@Override
	public void addPath(boolean[] abPath, double value) {
		int temp_aadd = buildPath(0, abPath, value);
		_nodeID = _context.applyInt(_nodeID, temp_aadd, DD.ARITH_SUM);
	}

	protected int buildPath(int var, boolean[] abPath, double value) {
		if (var == abPath.length) 
			return _context.getConstantNode(value);
		else 
			return _context.applyInt(
					_context.getVarNode(var, abPath[var] ? 0d : 1d, abPath[var] ? 1d : 0d), 
					buildPath(var+1, abPath, value), 
					DD.ARITH_PROD);
	}
	
	@Override
	public AlgebraicDecisionDiagram copy() {
		return new AffineADD(this);
	}

	@Override
	public boolean dominates(AlgebraicDecisionDiagram addOther) {
		System.err.println("AADD Error - dominates: not implemented");
		System.exit(1);
		return false;
	}

	@Override
	public boolean equals(AlgebraicDecisionDiagram addOther) {
		if (!(addOther instanceof AffineADD)) {
			System.err.println("AADD Error - addOther: not implemented");
			System.exit(1);
		}
		return _nodeID == ((AffineADD)addOther)._nodeID;
	}

	@Override
	public AlgebraicDecisionDiagram existentialAbstraction(
			AbstractionFilter filter) {

		AffineADD new_dd = new AffineADD(this);
		int id = -1;
		for( id = 0 ; id < MAX_VARS ; id++ ){
			if( filter.abstractVariable( id ) )
				new_dd._nodeID = _context.opOut(new_dd._nodeID, id, DD.ARITH_SUM);
		}
	/*	
		int last_id = filter.getLastVariableId();
		do {
			id = (id == -1) ? filter.getFirstVariableId() : filter.firstVariableAfter(id);
			new_dd._nodeID = _context.opOut(new_dd._nodeID, id, DD.ARITH_SUM);
		} while (id != last_id);
		*/
		return new_dd;
	}

	@Override
	public void finalizePaths(double defaultValue) {
		// Do nothing since we started with the constant 0 diagram
	}

	@Override
	public Element getDOM(Document doc) {
		System.err.println("AADD Error - getDOM: not implemented");
		System.exit(1);
		return null;
	}

	@Override
	public long getId() {
		return (long)_nodeID;
	}

	@Override
	public double getMaxValue() {
		return _context.getMaxValue(_nodeID);
	}

	@Override
	public String getTreeString() {
		return _context.printNode(_nodeID);
	}

	public String toString(){
		String s = "[";
		boolean[] abPath = new boolean[MAX_VARS];
		int iState = (int)Math.pow( 2, MAX_VARS);
		double dValue = 0.0;
		while( iState > 0 ){
			increment( abPath );
			dValue = valueAt( abPath );
			if( dValue != 0.0 )
				s += dValue + ",";
			iState--;
		}
		return s + "]";
	}
	
	private void increment( boolean[] abPath ) {
		int i = 0;
		for( i = 0 ; i < abPath.length ; i++ ){
			if( abPath[i] == false ){
				abPath[i] = true;
				return;
			}
			else{
				abPath[i] = false;
			}
		}
	}

	@Override
	public double getValueSum() {
		int new_dd = _nodeID;
		for (int v = 0; v < MAX_VARS; v++) {
			new_dd = _context.opOut(new_dd, v, DD.ARITH_SUM);
		}
		return _context.evaluate(new_dd, EMPTY_ARRAY);
	}

	@Override
	public long getVertexCount() {
		return _context.countExactNodes(_nodeID);
	}

	@Override
	public double innerProduct(AlgebraicDecisionDiagram addOther) {
		if (!(addOther instanceof AffineADD)) {
			System.err.println("AADD Error - getDOM: not implemented");
			System.exit(1);
		}
		AffineADD aaddOther = (AffineADD)addOther;
		AffineADD aaddResult = new AffineADD( MAX_VARS );
		aaddResult._nodeID = _context.applyInt(_nodeID, aaddOther._nodeID, DD.ARITH_PROD);
		
		return aaddResult.getValueSum();
	}

	@Override
	public void parseXML(Element eadd) {
		//System.err.println("AADD Error - parseXML: not implemented");
		//System.exit(1);
		throw new NotImplementedException();
	}

	@Override
	public AlgebraicDecisionDiagram product(AlgebraicDecisionDiagram addOther) {
		if (!(addOther instanceof AffineADD)) {
			System.err.println("AADD Error - getDOM: not implemented");
			System.exit(1);
		}
		AffineADD aaddOther = (AffineADD)addOther;
		AffineADD aaddResult = new AffineADD( MAX_VARS );
		aaddResult._nodeID = _context.applyInt(_nodeID, aaddOther._nodeID, DD.ARITH_PROD);
		
		return aaddResult;
	}

	@Override
	public void product(double factor) {
		_nodeID = _context.scalarMultiply(_nodeID, factor);
	}

	@Override
	public void reduce() {
		_nodeID = _context.reduce(_nodeID);
	}

	@Override
	public void reduce(AbstractionFilter filter) {
		_nodeID = _context.reduce(_nodeID);
	}

	@Override
	public void reduceToMin(double span) {
		_nodeID = _context.reduce(_nodeID);
	}

	@Override
	public void save(FileWriter fw) throws IOException {
		throw new NotImplementedException();
	}

	@Override
	public AlgebraicDecisionDiagram sum(AlgebraicDecisionDiagram addOther) {
		if (!(addOther instanceof AffineADD)) {
			System.err.println("AADD Error - getDOM: not implemented");
			System.exit(1);
		}
		AffineADD aaddOther = (AffineADD)addOther;
		AffineADD aaddResult = new AffineADD( MAX_VARS );
		aaddResult._nodeID = _context.applyInt(_nodeID, aaddOther._nodeID, DD.ARITH_SUM);
		
		return aaddResult;
	}

	@Override
	public void translateVariables(VariableTranslator vt) {
		HashMap gid_map = new HashMap();
		for (int i = 0; i < MAX_VARS; i++) {
			int trans = vt.translate(i);
			if (trans != i)
				gid_map.put(i, trans);
		}
		_context.remapGIDs(_nodeID, gid_map);
	}

	@Override
	public double valueAt(boolean[] abPath) {
		ArrayList assign = new ArrayList();
		for (int i = 0; i < MAX_VARS; i++) 
			assign.add(abPath[i]);
		return _context.evaluate(_nodeID, assign);
	}

	@Override
	public double valueAt(int[] aiVariables, boolean[] abValues) {
		ArrayList assign = new ArrayList();
		for (int i = 0; i < MAX_VARS; i++) 
			assign.add(null);
		for (int i = 0; i < aiVariables.length; i++)
			assign.set(aiVariables[i], abValues[i]);
		return _context.evaluate(_nodeID, assign);
	}

	@Override
	public void release() {
		System.err.println("AADD Error - release: not implemented at this time");
	}

	@Override
	public int compareTo(AlgebraicDecisionDiagram arg0) {
		if (!(arg0 instanceof AffineADD)) {
			System.err.println("AADD Error - compareTo: not implemented");
			System.exit(1);
		}
		return _nodeID - ((AffineADD)arg0)._nodeID;
	}

	/**
	 * @param args
	 */
	/*
	public static void main(String[] args) {
		
		// Initialize AADD caches with max number of variables 
		AffineADD.InitAADDContext(3);
		
		// Build an AADD and add in two paths (one full, one partial)
		AlgebraicDecisionDiagram a1 = new AffineADD();
		
		// Path 1
		boolean[] abPath = new boolean[] {true, false, true};
		a1.addPath(abPath, 1d);
		AlgebraicDecisionDiagram a2 = (AffineADD)a1.copy();
		
		// Path 2
		int[] aiVariables = new int[] { 1, 2 };
		boolean[] abValues = new boolean[] { true, true };
		a1.addPartialPath(aiVariables, abValues, 2d, false);
		
		// Some dianostic output
		System.out.println("AADD Repr: " + a1.getTreeString());
		System.out.println("Num nodes [Expected:5]: " + a1.getVertexCount()); // should be 9
		System.out.println("ValueSum  [Expected:5]: " + a1.getValueSum());    // shuold be 5
		System.out.println("InnerProd [Expected:9]: " + a1.innerProduct(a1)); // should be 9
		
		// Print out truth table values 
		System.out.println("\nOriginal - should be two 2's, one 1:");
		for (int i = 0; i < 8; i++) {
			abPath[0] = ((i >> 0) % 2) != 0;
			abPath[1] = ((i >> 1) % 2) != 0;
			abPath[2] = ((i >> 2) % 2) != 0;
			System.out.println("a1 val [" + abPath[0] + "," + abPath[1] + "," + abPath[2] + "] = " + a1.valueAt(abPath));
		}

		// Print out truth table values for a product
		System.out.println("\nAfter a product - should be one 1");
		AlgebraicDecisionDiagram a3 = a1.product(a2);
		for (int i = 0; i < 8; i++) {
			abPath[0] = ((i >> 0) % 2) != 0;
			abPath[1] = ((i >> 1) % 2) != 0;
			abPath[2] = ((i >> 2) % 2) != 0;
			System.out.println("a3 val [" + abPath[0] + "," + abPath[1] + "," + abPath[2] + "] = " + a3.valueAt(abPath));
		}

		// Print out truth table values for a sum & scalar multiply
		System.out.println("\nAfter a sum & scalar multiply - should be three 3 4's");
		AlgebraicDecisionDiagram a4 = a1.sum(a2);
		a4.product(2d);
		for (int i = 0; i < 8; i++) {
			abPath[0] = ((i >> 0) % 2) != 0;
			abPath[1] = ((i >> 1) % 2) != 0;
			abPath[2] = ((i >> 2) % 2) != 0;
			System.out.println("a4 val [" + abPath[0] + "," + abPath[1] + "," + abPath[2] + "] = " + a4.valueAt(abPath));
		}
	}
*/
	@Override
	public AlgebraicDecisionDiagram max(AlgebraicDecisionDiagram addOther) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public double innerProduct(double[] adVariableProbabilities) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double innerProduct(PathProbabilityEstimator pbe) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public void setUnspecifiedVariablesToWorstCase(
			Vector<Integer> unspecifiedVariables) {
		// TODO Auto-generated method stub
		
	}
}
