package pomdp.utilities;

import pomdp.GridAgent;

import java.util.*;

public class PotentialCollisionData {
    private List<GridAgent> agents;
    private Set<Integer> collisionStates;
    private int collisionStep;
    private int nonDominantIndex;
    private boolean isReset;

    public PotentialCollisionData() {
        agents = new ArrayList<>();
        collisionStates = new HashSet<>();
        nonDominantIndex = 0;
        isReset = false;
    }

    public void addAgent(GridAgent a) {
        agents.add(a);
    }

    public void addAgents(GridAgent[] toAddAgents) {
        agents.addAll(Arrays.asList(toAddAgents));
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

    // removes agent a from the list of agents
    public void removeAgent(GridAgent a) {
        agents.remove(a);
    }

    public void setCollisionStep(int collisionStep) {
        this.collisionStep = collisionStep;
    }

    public int getCollisionStep() {
        return collisionStep;
    }

    private void initSelection() {
        Collections.sort(agents);
    }

    public GridAgent getMostNonDominant() {
        if (!isReset) {
            initSelection();
            isReset = true;
        }
        return agents.get(0);
    }

    public void reset() {
        nonDominantIndex = 0;
        initSelection();
    }

    public boolean hasNextNonDominant() {
        return nonDominantIndex < agents.size();
    }

    public GridAgent getNextNonDominant() {
        if (!isReset) {
            initSelection();
            isReset = true;
        }
        return agents.get(nonDominantIndex++);
    }
}
