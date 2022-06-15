# POMDP_MAPF

TODO: Add a nice description

TODO List:
TODO: apply mini joint problem on possibly-collided agents
	TOOD: Change the observation model to ping beacons
TODO: check if the agents can collide in <= k steps (instead of k steps)
TODO: Run the forbidden states algorithm

Questions:
- What to do in case that the joint agent can collide with other agent (possible answer: build another joint agent with this agent)
- Theory: if I check collision for every move of one agent, the potential collision will be between exactly 2 agents. ?
- For room: very inaccurate location -> large minigrid -> stuck
- We need to suggest new method to enclose the minigrid.