package com.spartronics4915.lib.util;

import java.util.ArrayList;

import com.spartronics4915.lib.lidar.icp.Point;
import com.spartronics4915.lib.lidar.icp.Segment;

public class SegmentBuilder {
    private ArrayList<Point> mPoints;

    public SegmentBuilder(Point firstPoint, Point secondPoint) {
        mPoints = new ArrayList<Point>();
        mPoints.add(firstPoint);
        mPoints.add(secondPoint);
    }

    public SegmentBuilder(Point secondPoint) {
        this(new Point(0, 0), secondPoint);
    }

    public SegmentBuilder horizontalBy(double x) {
        mPoints.add(getLastPointAdded(x, 0));
        return this; // Method chaining support
    }

    public SegmentBuilder verticalBy(double y) {
        mPoints.add(getLastPointAdded(0, y));
        return this; // Method chaining support
    }

    public Segment[] getSegments() {
        Segment[] segments = new Segment[mPoints.size() - 1];
        for (int i = 0; i < segments.length; i++) {
            segments[i] = new Segment(mPoints.get(i), mPoints.get(i + 1));
        }
        return segments;
    }

    private Point getLastPointAdded(double dx, double dy) {
        Point p = mPoints.get(mPoints.size() - 1);
        return new Point(p.x + dx, p.y + dy);
    }
}