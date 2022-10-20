# POMDP_MAPF

TODO: Add a nice description

main():
	for agent a:
		a.timer = 0
	PI = initial policy for all agents
	step = 0
		
	while (not all agents are done AND step < maxSteps):
		for agent a:
			if (a.timer == 0)
				replan for a without forbidden states
				a.timer = null
				
		inConflict = findNextSafeStep(PI, d, t)
		
//		for agent a:
//			if (a is in inConflict)
//				localize a
//			else
//				execute step with a, decrease a.timer if exist
				
		if (inConflict is empty)
			for agent a:
				execute step with a, decrease a.timer if exist
		else
			localize
			
findNextSafeStep(PI, d, t):
	inConflict = mightCollide(PI, d)
	Rtry = empty set
	while (inConflict is not empty AND inConflict \ Rtry is not empty):
		r = getNextNonDominant(inConflict, Rtry)
		add r to Rtry
		pi_R = findSafePlan(r, PI^-r, d)
		if (pi_r is a good enough policy for r)
			update PI with pi_r
			r.timer = t
			inConflict = mayCollide(PI, d)
	return inConflict