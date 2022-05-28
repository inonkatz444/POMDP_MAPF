package pomdp.utilities;

import pomdp.GridAgent;

import java.util.List;
import java.util.Set;

public class PotentialCollisionData {
    List<GridAgent> agents;
    Set<Integer> collisionStates;

    public PotentialCollisionData() {

    }

    public void addAgent(GridAgent a) {
        agents.add(a);
    }

    public void addCollisionStates(Set<Integer> states) {
        collisionStates.addAll(states);
    }

    public List<GridAgent> getAgents() {
        return agents;
    }

    public Set<Integer> getCollisionStates() {
        return collisionStates;
    }

    public boolean isEmpty() {
        return agents.isEmpty();
    }
}
