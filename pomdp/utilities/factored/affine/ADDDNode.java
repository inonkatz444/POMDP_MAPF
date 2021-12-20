package pomdp.utilities.factored.affine;

import java.util.*;

public class ADDDNode extends ADDNode {
    
    public double _dLower;
    public double _dUpper;

    public String _sLowerLabel; // Optional labels, i.e. which max contributed?
    public String _sUpperLabel; 
    
    // For a single value
    public ADDDNode(int lid, double val) {
	_nLocalID  = lid;
	_dLower    = val;
	_dUpper    = val;
	_sLowerLabel = null;
	_sUpperLabel = null;
    }
    
    // For a range value
    public ADDDNode(int lid, double min, double max) {
	_nLocalID = lid;
	_dLower   = min;
	_dUpper   = max;
	_sLowerLabel = null;
	_sUpperLabel = null;
    }
    
    // For a range value
    public ADDDNode(int lid, double min, double max, 
		    String lower_label, String upper_label) {
	_nLocalID = lid;
	_dLower   = min;
	_dUpper   = max;
	_sLowerLabel = lower_label;
	_sUpperLabel = upper_label;
    }
    
    public String toString() {
	return "*" + DD._df.format(_dLower) + "*";
    }

    public String toString(Object context, int depth) {
	
	if (_dUpper == _dLower) {
	    String label = "";
	    if (_sUpperLabel != null) {
		label = ": <" + _sUpperLabel + "> ";
	    }
	    return "[ #" + _nLocalID + " <" + ADD._df.format(_dLower) + "> ] " + label;
	} else {
	    String label = "";
	    if (_sLowerLabel != null ||_sUpperLabel != null) {
		if (_sLowerLabel == null) {
		    label = ": <" + _sUpperLabel + "> ";
		} else if (_sUpperLabel == null) {
		    label = ": <" + _sLowerLabel + "> ";
		} else if (_sUpperLabel.equals(_sLowerLabel)) {
		    label = ": <" + _sUpperLabel + "> ";
		} else {
		    label = ": <" + _sLowerLabel + "," + _sUpperLabel + "> ";
		}
		label = ": " + _sUpperLabel;
	    }
	    return "[ #" + _nLocalID + " <" + ADD._df.format(_dLower) 
		                      + "," + ADD._df.format(_dUpper) + "> ] " + label;
	}
    }
}
