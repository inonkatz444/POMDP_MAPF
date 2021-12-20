package pomdp.utilities.factored.affine;

import java.util.*;

public class ADDINode 
    extends ADDNode {

    public int _nHigh; // Reference to an ADDNode (true branch)
    public int _nLow;  // Reference to an ADDNode (false branch)
    
    // This is assigned by an outer struct and gives
    // access to formula assoc with global-ID, is
    // analogy of prop var ID for ADDs.
    public int _nGlobalID;
    public double _dMinLower, _dMinUpper;
    public double _dMaxLower, _dMaxUpper;
    
    public ADDINode(int lid, int gid) {
	_nLocalID  = lid;
	_nGlobalID = gid;
	_dMinLower = _dMinUpper = Double.NaN;
	_dMaxLower = _dMaxUpper = Double.NaN;
    }
    
    public ADDINode(int lid, int gid, int low, int high) {
	_nLocalID  = lid;
	_nGlobalID = gid;
	_nLow      = low;
	_nHigh     = high;
	_dMinLower = _dMinUpper = Double.NaN;
	_dMaxLower = _dMaxUpper = Double.NaN;
    }
    
    public String toString(Object context, int depth) {
	StringBuffer sb = new StringBuffer();
	sb.append("[ #" + _nLocalID /* + "/" + _nRefCount*/ + " v" + _nGlobalID + " ");
	
	// Internal bounds
	//sb.append("<" +  ADD._df.format(_dMinLower) + "..." +  ADD._df.format(_dMaxLower) + " ; " + 
	//	         ADD._df.format(_dMinUpper) + "..." +  ADD._df.format(_dMaxUpper) + "> ");
	
	// Node level cache
	ADDNode n1 = ((ADD)context).getNode(_nLow);
	if (n1 != null) {
	    sb.append("\n" + indent(depth) + "l:[ " + 
		      n1.toString(((ADD)context), depth+1) + "] ");
	} else {
	    sb.append("l:[null] ");
	}
	ADDNode n2 = ((ADD)context).getNode(_nHigh);
	if (n2 != null) {
	    sb.append("\n" + indent(depth) + 
		      "h:[ " + n2.toString(((ADD)context), depth+1) + "] ");
	} else {
	    sb.append("h:[null] ");
	}
	sb.append("] ");
	
	return sb.toString();
    }
}

