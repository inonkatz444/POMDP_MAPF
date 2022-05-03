package pomdp.utilities;

public class Point extends Pair<Integer, Integer>{
    public Point(int first, int second) {
        super(first, second);
    }

    public int distance(Point other) {
        return Math.abs(this.m_first - other.m_first) + Math.abs(this.m_second - other.m_second);
    }

    @Override
    public String toString() {
        return "<" + m_first + ", " + m_second + ">";
    }
}
