package pomdp.utilities;

public class Beacon {
    private Pair<Integer, Integer> loc;
    private int range;
    private int ID;

    public Beacon(int id, int row, int col, int range) {
        this.loc = new Pair<>(row, col);
        this.range = range;
        this.ID = id;
    }

    public int getRange() {
        return range;
    }

    public int distTo(int state_row, int state_col) {
        return Math.abs(this.loc.first() - state_row) + Math.abs(this.loc.second() - state_col);
    }

    public int distTo(Pair<Integer, Integer> loc) {
        return distTo(loc.first(), loc.second());
    }

    @Override
    public String toString() {
        return "Beacon{" +
                "loc=" + loc +
                '}';
    }
}
