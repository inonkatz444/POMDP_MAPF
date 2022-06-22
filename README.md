# POMDP_MAPF

TODO: Add a nice description

TODO List:
TODO: Run the forbidden states algorithm with the change
TODO: Change the algorithm so the non-dominant agent will replan after k steps using the escape policy


Questions:
- What to do in case that the joint agent can collide with other agent (possible answer: build another joint agent with this agent)
- Theory: if I check collision for every move of one agent, the potential collision will be between exactly 2 agents. ?
- For room: very inaccurate location -> large minigrid -> stuck
- We need to suggest new method to enclose the minigrid.


for agent a:
	a.timer = 0
	
while (exceeds number of steps OR all agents are done):
	for agent a:
		if (a is done):
			continue
			
		if (a.timer == 0)
			replan for a without forbidden states
			a.timer = null
			
		if (a potentially collide with agents A):
			detect the most non-dominant agent c
			forbid c to reach certain states
			replan policy for c
			c.timer = k
			
		execute step with a using the most constrainted policy, decrease a.timer if exist