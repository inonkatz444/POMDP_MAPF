package pomdp.utilities.factored.affine;

import java.util.*;

public class AADDINode extends ADDNode {
    
    // This is assigned by an outer struct and gives
    // access to formula assoc with global-ID, is
    // analogy of prop var ID for ADDs.
    public int _nGlobalID;
    public int   _nLow, _nHigh;
    public double _dLowOffset,  _dLowMult;
    public double _dHighOffset, _dHighMult;
    //public double _dMinLower, _dMinUpper;
    //public double _dMaxLower, _dMaxUpper;
    
    public AADDINode(int lid, int gid) {
	_nLocalID  = lid;
	_nGlobalID = gid;
	//_dMinLower = _dMinUpper = Double.NaN;
	//_dMaxLower = _dMaxUpper = Double.NaN;
    }
    
    public AADDINode(int lid, int gid, int low, int high, 
		     double o_l, double m_l, double o_h, double m_h) {
	_nLocalID    = lid;
	_nGlobalID   = gid;
	_nLow        = low;
	_nHigh       = high;
	_dLowOffset  = o_l;
	_dLowMult    = m_l;
	_dHighOffset = o_h;
	_dHighMult   = m_h;
	//_dMinLower = _dMinUpper = Double.NaN;
	//_dMaxLower = _dMaxUpper = Double.NaN;
    }

    public String toString(Object context, int depth) {
	StringBuffer sb = new StringBuffer();
	sb.append("[ #" + _nLocalID + " v" + _nGlobalID + " ");
	
	// Internal bounds
	//sb.append("<" +  ADD._df.format(_dMinLower) + "..." +  
        //                 ADD._df.format(_dMaxLower) + " ; " + 
	//	           ADD._df.format(_dMinUpper) + "..." +  
	//                 ADD._df.format(_dMaxUpper) + "> ");

	if (context instanceof AADD) {

	    // Node level cache
	    ADDNode n1 = ((AADD)context).getNode(_nHigh);
	    if (n1 != null) {
		sb.append("\n" + indent(depth) + "<" + AADD._df.format(_dHighOffset) + 
			  "," + AADD._df.format(_dHighMult) + ">");
		if (_nHigh == _nLow) {
		    sb.append("\n" + indent(depth) + "<" + AADD._df.format(_dLowOffset) + 
			      "," + AADD._df.format(_dLowMult) + "> LIN:");
		    sb.append(" [ " + n1.toString(((AADD)context), depth+1) + "] ");
		} else {
		    sb.append(" h:[ " + n1.toString(((AADD)context), depth+1) + "] ");	
		}
	    } else {
		sb.append("h:[null] ");
	    }
	    
	    if (_nHigh != _nLow) {
		
		ADDNode n2 = ((AADD)context).getNode(_nLow);
		if (n2 != null) {
		    sb.append("\n" + indent(depth) + "<" + AADD._df.format(_dLowOffset) + 
			      "," + AADD._df.format(_dLowMult) + ">" + " l:[ " + 
			      n2.toString(((AADD)context), depth+1) + "] ");
		} else {
		    sb.append("l:[null] ");
		}
		sb.append("] ");
	    }	

	} 

	return sb.toString();
    }
}

