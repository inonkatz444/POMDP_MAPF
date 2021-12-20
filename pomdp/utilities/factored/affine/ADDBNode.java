package pomdp.utilities.factored.affine;

import java.util.*;

public class ADDBNode extends ADDNode {
    
    public boolean _bVal;
    
    public ADDBNode(int lid, boolean bval) {
	_nLocalID = lid;
	_bVal     = bval;
    }
    
    public String toString(Object context, int depth) {
	    return "[ #" + _nLocalID + " <" + ((_bVal) ? "T" : "F" ) + "> ] ";
    }
}
