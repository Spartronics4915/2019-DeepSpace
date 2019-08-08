package com.spartronics4915.lib.lidar.icp;

import com.spartronics4915.lib.geometry.Pose2d;
import com.spartronics4915.lib.geometry.Rotation2d;
import com.spartronics4915.lib.geometry.Translation2d;

public class Transform
{

    // rotate by theta about the origin, then translate by <tx, ty>
    public final double theta, tx, ty;
    protected final double sin, cos; // cache these

    public Transform()
    {
        theta = 0;
        tx = ty = 0;
        sin = 0.0;
        cos = 1.0;
    }

    public Transform(double theta, double tx, double ty)
    {
        this(theta, tx, ty, Math.sin(theta), Math.cos(theta));
    }

    public Transform(double theta, double tx, double ty, double sin, double cos)
    {
        this.theta = theta;
        this.tx = tx;
        this.ty = ty;
        this.sin = sin;
        this.cos = cos;
    }

    public Transform(Pose2d pose)
    {
        Translation2d t = pose.getTranslation();
        Rotation2d r = pose.getRotation();
        theta = r.getRadians();
        sin = r.sin();
        cos = r.cos();
        tx = t.x();
        ty = t.y();
    }

    public Pose2d toPose2d()
    {
        return new Pose2d(new Translation2d(tx, ty), new Rotation2d(cos, sin, false));
    }

    public Point apply(Point p)
    {
        return new Point(p.x * cos - p.y * sin + tx,
                p.x * sin + p.y * cos + ty);
    }

    public Line apply(Line l)
    {
        return new Line(l.vx * cos - l.vy * sin,
                l.vx * sin + l.vy * cos,
                l.x0 * cos - l.y0 * sin + tx,
                l.x0 * sin + l.y0 * cos + ty);
    }

    public Segment apply(Segment s)
    {
        return new Segment(apply(s.line), s.tMin, s.tMax);
    }

    public Transform inverse()
    {
        return new Transform(-theta,
                -tx * cos - ty * sin,
                tx * sin - ty * cos,
                -sin, cos);
    }

    public String toString()
    {
        return "[" + Math.toDegrees(theta) + "° <" + tx + ", " + ty + ">]";
    }

}
