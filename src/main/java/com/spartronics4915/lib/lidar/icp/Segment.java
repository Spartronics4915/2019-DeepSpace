package com.spartronics4915.lib.lidar.icp;

public class Segment
{

    public Line line;
    public double tMin, tMax;
    public Point pMin, pMax;

    public Segment(Line line, double tMin, double tMax)
    {
        this.line = line;
        this.tMin = tMin;
        this.tMax = tMax;
        this.pMin = line.getPoint(tMin);
        this.pMax = line.getPoint(tMax);
    }

    public Segment(Point p0, Point p1)
    {
        this(new Line(p0, p1), 0, 1);
        normalize();
    }

    private void normalize()
    {
        double mSq = line.vx * line.vx + line.vy * line.vy;
        double m = Math.sqrt(mSq);
        line.vx /= m;
        line.vy /= m;
        line.r /= m;
        tMax *= m;
    }

    public double getDistance(Point p)
    {
        double t = line.getT(p);
        if (t <= tMin)
            return pMin.getDistance(p);
        if (t >= tMax)
            return pMax.getDistance(p);
        return line.getDistance(p);
    }

    public double getDistanceSq(Point p)
    {
        double t = line.getT(p);
        if (t <= tMin)
            return pMin.getDistanceSq(p);
        if (t >= tMax)
            return pMax.getDistanceSq(p);
        double d = line.getDistance(p);
        return d * d;
    }

    public Point getClosestPoint(Point p)
    {
        double t = line.getT(p);
        if (t <= tMin)
            return pMin;
        if (t >= tMax)
            return pMax;
        return line.getPoint(t);
    }

    public Point getMidpoint()
    {
        return line.getPoint((tMin + tMax) / 2);
    }

    public String toString()
    {
        return "Segment(" + line + ", [" + tMin + ", " + tMax + "])";
    }

    public static Segment[] makeInRectangle(Point cornerOne, Point cornerTwo)
    {
        // These may not be top or right if cornerTwo is left or higher than
        // cornerOne but it's easier to reason about this by naming things this way.
        Point topRight = new Point(cornerTwo.x, cornerOne.y);
        Point bottomLeft = new Point(cornerOne.x, cornerTwo.y);
        return new Segment[] {
                new Segment(cornerOne, topRight),
                new Segment(topRight, cornerTwo),
                new Segment(cornerTwo, bottomLeft),
                new Segment(bottomLeft, topRight),
        };
    }
}
