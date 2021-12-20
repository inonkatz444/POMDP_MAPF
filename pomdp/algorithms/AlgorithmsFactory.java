package pomdp.algorithms;

import pomdp.algorithms.gridbased.FixedResolutionGrid;
import pomdp.algorithms.gridbased.FixedSetGrid;
import pomdp.algorithms.gridbased.VariableResolutionGrid;
import pomdp.algorithms.online.RealTimeBeliefSpaceSearch;
import pomdp.algorithms.online.RealTimeDynamicProgramming;
import pomdp.algorithms.pointbased.ForwardSearchValueIteration;
import pomdp.algorithms.pointbased.HeuristicSearchValueIteration;
import pomdp.algorithms.pointbased.PerfectInformationValueIteration;
import pomdp.algorithms.pointbased.PerseusValueIteration;
import pomdp.algorithms.pointbased.PointBasedValueIteration;
import pomdp.algorithms.pointbased.PrioritizedPBVI;
import pomdp.algorithms.pointbased.PrioritizedPerseus;
import pomdp.algorithms.pointbased.PrioritizedValueIteration;
import pomdp.environments.POMDP;
import pomdp.utilities.MDPValueFunction;

public class AlgorithmsFactory {
	public static ValueIteration getAlgorithm( String sName, POMDP pomdp ){
		if( sName.equals( "FSVI" ) )
			return new ForwardSearchValueIteration( pomdp );
		if( sName.equals( "PBVI" ) )
			return new PointBasedValueIteration( pomdp );
		if( sName.equals( "HSVI" ) )
			return new HeuristicSearchValueIteration( pomdp );
		if( sName.equals( "VPI" ) )
			return new PerfectInformationValueIteration( pomdp );
		if( sName.equals( "Perseus" ) )
			return new PerseusValueIteration( pomdp );
		if( sName.equals( "PVI" ) )
			return new PrioritizedValueIteration( pomdp );
		if( sName.equals( "PPBVI" ) )
			return new PrioritizedPBVI( pomdp );
		if( sName.equals( "PPerseus" ) )
			return new PrioritizedPerseus( pomdp );
		if( sName.equals( "PBVI" ) )
			return new PointBasedValueIteration( pomdp );
		if( sName.equals( "RTDP" ) )
			return new RealTimeDynamicProgramming( pomdp );
		if( sName.equals( "RTBSS" ) )
			return new RealTimeBeliefSpaceSearch( pomdp );
		if( sName.equals( "FRG" ) )
			return new FixedResolutionGrid( pomdp );
		if( sName.equals( "FSG" ) )
			return new FixedSetGrid( pomdp );
		if( sName.equals( "VRG" ) )
			return new VariableResolutionGrid( pomdp );
		if( sName.equals( "IP" ) )
			return new IncrementalPruningValueIteration( pomdp );
		return null;
	}
}
