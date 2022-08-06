# POMDP_MAPF

TODO: Add a nice description

TODO List:

TODO: increase reward to motivate localization
TOOD: Fix conflict detection to prevent following conflicts **done
TODO: Full localization baseline	*later
TODO: Constraint propagation	*Later
TODO: Develop smart localization algorithm	*Later
TODO: Create more small problems
TODO: With n>2 agents, plan for each non dominant agent based on the forbidden states of the more dominant ones.

Suggestion: If none of the agent succeded to find a policy to the goal, try to localize both agents before expanding their
			beliefs (would reduce each forbidden states).

for agent a:
	a.timer = 0
	
while (exceeds number of steps OR all agents are done):
	for agent a:
		if (a is done):
			continue
		if (a.timer == 0)
			replan for a without forbidden states
			a.timer = null
			
	if (there is a potential collision):
		do:
			do:
				detect the next most non-dominant agent c
				forbid c to reach certain states
				replan policy for c
			while c is failing
			if c succeded:
				break
			else:
				foreach agent a, localize a
		while not TIMEOUT
		c.timer = k
		
	for agent a:
		execute step with a, decrease a.timer if exist