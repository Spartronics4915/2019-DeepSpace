package com.spartronics4915.frc2019.auto.modes;

import com.spartronics4915.frc2019.auto.AutoModeBase;
import com.spartronics4915.frc2019.auto.AutoModeEndedException;
import com.spartronics4915.frc2019.paths.TrajectoryGenerator;
import com.spartronics4915.frc2019.paths.TrajectoryGenerator.TrajectorySet;
import com.spartronics4915.frc2019.auto.actions.DriveTrajectory;
import com.spartronics4915.frc2019.auto.actions.ZeroOdometryOnHAB;
import com.spartronics4915.frc2019.auto.actions.ZeroOdometryOnHAB.StartPosition;

public class PlaceCargoFromSideMode extends AutoModeBase
{
    private final boolean mIsLeft;

    public PlaceCargoFromSideMode(boolean isLeft)
    {
        mIsLeft = isLeft;
    }

    @Override
    protected void routine() throws AutoModeEndedException
    {
        TrajectorySet tSet = TrajectoryGenerator.getInstance().getTrajectorySet();
        if (mIsLeft)
        {
            runAction(new ZeroOdometryOnHAB(StartPosition.LEFT_PLATFORM));
        }
        else
        {
            runAction(new ZeroOdometryOnHAB(StartPosition.RIGHT_PLATFORM));
        }
        runAction(new DriveTrajectory(tSet.driveToClosestCargoShipBayFromSide.get(mIsLeft)));
    }

}