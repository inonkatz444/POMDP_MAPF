package pomdp.utilities;

import java.util.Comparator;

public class ArrayComparator implements Comparator<long[]>{
	
	private static ArrayComparator g_sInstance = new ArrayComparator();
	
	private ArrayComparator(){	
	}
	
	public static ArrayComparator getInstance(){
		return g_sInstance;
	}
	
	public int compare( long[] a1, long[] a2 ){
		if( a1.length != a2.length )
			return a1.length - a2.length;
		/*
		if( a1[0] * a2[0] < 0 )
			 return (int) (a1[0] - a2[0]);
		if( a1[0] * a2[0] == 0 ){
			if( a1[0] != 0 )
				return (int) a1[0];
			if( a2[0] != 0 )
				return (int) -a2[0];
		}
		*/
		int i = 0; 
		for( i = a1.length - 1 ; i >= 0 ; i-- )
			if( a1[i] != a2[i] )
				return (int)( a1[i] - a2[i] );
		return 0;
	}
}
