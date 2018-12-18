package com.spartronics4915.lib.geometry;

public interface IRobotStateMap
{
    public class State
    {
        public Pose2d position;
        public Twist2d velocity;
        public State(Pose2d p)
        {
            this.position = p;
            this.velocity = new Twist2d(0, 0, 0);
        }
        public State(Pose2d p, Twist2d v)
        {
            this.position = p;
            this.velocity = v;
        }
    }
    public State get(double timestamp);
}
