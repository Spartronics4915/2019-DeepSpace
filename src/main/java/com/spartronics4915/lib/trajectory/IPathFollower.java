package com.spartronics4915.lib.trajectory;

import com.spartronics4915.lib.geometry.Pose2d;
import com.spartronics4915.lib.geometry.Twist2d;

public interface IPathFollower
{

    public Twist2d steer(Pose2d current_pose);

    public boolean isDone();
}
