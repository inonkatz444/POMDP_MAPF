package pomdp.utilities.datastructures;

import pomdp.utilities.Pair;

import java.util.*;

public class CartesianIterator implements Iterator<Map.Entry<Integer, Double>> {

    private final List<Iterable<Map.Entry<Integer, Double>>> iterables;
    private final List<Iterator<Map.Entry<Integer, Double>>> iterators;
    private List<Map.Entry<Integer, Double>> values;
    private int numOfSingleStates;
    private int size;
    private boolean empty;

    private static Map<Integer, Pair<Integer, Double>> cachedPairs = new HashMap<>();

    /**
     * Constructor
     *
     * @param iterables array of Iterables being the source for the Cartesian product.
     */
    public CartesianIterator(List<Iterable<Map.Entry<Integer, Double>>> iterables, int numOfSingleStates) {
        this.size = iterables.size();
        this.iterables = iterables;
        this.iterators = new ArrayList<>(size);
        this.numOfSingleStates = numOfSingleStates;

        // Initialize iterators
        for (int i = 0; i < size; i++) {
            iterators.add(iterables.get(i).iterator());
            // If one of the iterators is empty then the whole Cartesian product is empty
            if (!iterators.get(i).hasNext()) {
                empty = true;
                break;
            }
        }

        // Initialize the tuple of the iteration values except the last one
        if (!empty) {
            values = new ArrayList<>(size);
            for (int i = 0; i < size - 1; i++) setNextValue(i);
        }
    }

    public void initialize() {
        this.iterators.clear();

        // Initialize iterators
        for (int i = 0; i < size; i++) {
            iterators.add(iterables.get(i).iterator());
            // If one of the iterators is empty then the whole Cartesian product is empty
            if (!iterators.get(i).hasNext()) {
                empty = true;
                break;
            }
        }

        // Initialize the tuple of the iteration values except the last one
        if (!empty) {
            values = new ArrayList<>();
            for (int i = 0; i < size - 1; i++) setNextValue(i);
        }
    }

    @Override
    public boolean hasNext() {
        if (empty) return false;
        for (int i = 0; i < size; i++)
            if (iterators.get(i).hasNext())
                return true;
        return false;
    }

    @Override
    public Map.Entry<Integer, Double> next() {
        // Find first in reverse order iterator the has a next element
        int cursor;
        for (cursor = size - 1; cursor >= 0; cursor--)
            if (iterators.get(cursor).hasNext()) break;

        // Initialize iterators next from the current one
        for (int i = cursor + 1; i < size; i++) iterators.set(i, iterables.get(i).iterator());

        // Get the next value from the current iterator and all the next ones
        for (int i = cursor; i < size; i++) setNextValue(i);

        int state = 0;
        double prob = 1.0;
        for (int i = 0, power = 1; i < size; i++, power *= numOfSingleStates) {
            state += power * values.get(i).getKey();
            prob *= values.get(i).getValue();
        }

        if (cachedPairs.containsKey(state)) {
            cachedPairs.get(state).setSecond(prob);
        }
        else {
            cachedPairs.put(state, new Pair<>(state, prob));
        }
        return cachedPairs.get(state);
    }

    /**
     * Gets the next value provided there is one from the iterator at the given index.
     *
     * @param index
     */
    private void setNextValue(int index) {
        Iterator<Map.Entry<Integer, Double>> it = iterators.get(index);
        if (it.hasNext()) {
            if (values.size() <= index) {
                values.add(it.next());
            }
            else {
                values.set(index, it.next());
            }
        }
    }
}
