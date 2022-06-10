package pomdp.utilities;

import pomdp.environments.Grid;

public class Point extends Pair<Integer, Integer>{
    public static Point[][] points;
    public Point(int first, int second) {
        super(first, second);
    }

    public static Point getPoint(int row, int col) {
        if (points[row][col] == null) {
            points[row][col] = new Point(row, col);
        }
        return points[row][col];
    }

    public int distance(Point other) {
        return Math.abs(this.m_first - other.m_first) + Math.abs(this.m_second - other.m_second);
    }

    public Point add(Point other) {
        return Point.getPoint(m_first + other.first(), m_second + other.second());
    }

    public Point subtract(Point other) {
        return Point.getPoint(m_first - other.first(), m_second - other.second());
    }

    public Point relativeTo(Point anchor) {
        return subtract(anchor);
    }

    public boolean inBound(Grid grid) {
        return m_first >= grid.getOrigin().first() &&
                m_second >= grid.getOrigin().second() &&
                m_first < grid.getOrigin().first() + grid.getRows() &&
                m_second < grid.getOrigin().second() + grid.getCols();
    }

    @Override
    public String toString() {
        return "<" + m_first + ", " + m_second + ">";
    }
}
