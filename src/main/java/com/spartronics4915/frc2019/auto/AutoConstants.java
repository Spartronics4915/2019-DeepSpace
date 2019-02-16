package com.spartronics4915.frc2019.auto;

import com.spartronics4915.frc2019.Constants;

public class AutoConstants
{

    public static final double kDriveOffHabVelocity = 24;
    // Feedforward is sort of wrong for the right side
    public static final double kDriveOffHabFeedforward =
            kDriveOffHabVelocity * Constants.kDriveLeftKv + Math.copySign(Constants.kDriveLeftVIntercept, kDriveOffHabVelocity);
}
