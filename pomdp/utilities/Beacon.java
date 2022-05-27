package pomdp.utilities;

import java.util.Objects;

public class Beacon {
    private Point loc;
    private int range;
    private int ID;
    private static int beaconID = 0;

    public Beacon(int row, int col, int range) {
        this.loc = new Point(row, col);
        this.range = range;
        this.ID = beaconID++;
    }

    public Beacon(Point loc, int range) {
        this.loc = loc;
        this.range = range;
        this.ID = beaconID++;
    }

    public int getRange() {
        return range;
    }

    public Point getLoc() {
        return loc;
    }

    public Beacon relativeTo(Point anchor) {
        return new Beacon(loc.relativeTo(anchor), range);
    }

    public int distTo(int state_row, int state_col) {
        return Math.abs(this.loc.first() - state_row) + Math.abs(this.loc.second() - state_col);
    }

    public int distTo(Pair<Integer, Integer> loc) {
        return distTo(loc.first(), loc.second());
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Beacon beacon = (Beacon) o;
        return loc.equals(beacon.loc);
    }

    @Override
    public int hashCode() {
        return Objects.hash(loc);
    }

    @Override
    public String toString() {
        return "Beacon{" +
                "loc=" + loc +
                '}';
    }
}
