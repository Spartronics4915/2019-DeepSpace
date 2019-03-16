package com.spartronics4915.frc2019.auto.modes;

import com.spartronics4915.frc2019.auto.AutoConstants;
import com.spartronics4915.frc2019.auto.AutoModeBase;
import com.spartronics4915.frc2019.auto.AutoModeEndedException;
import com.spartronics4915.frc2019.auto.actions.DriveOffHAB;
import com.spartronics4915.frc2019.auto.actions.DriveTrajectory;
import com.spartronics4915.frc2019.auto.actions.ZeroOdometryOffHAB;
import com.spartronics4915.frc2019.auto.actions.DriveOffHAB.HABLevel;
import com.spartronics4915.frc2019.auto.actions.ZeroOdometryOffHAB.StartPosition;
import com.spartronics4915.frc2019.paths.TrajectoryGenerator;
import com.spartronics4915.frc2019.paths.TrajectoryGenerator.TrajectorySet;

public class PlaceHatchFromMiddleMode extends AutoModeBase 
{

    private final boolean mIsLeft;

    public PlaceHatchFromMiddleMode(boolean isLeft)
    {
        mIsLeft = isLeft;
    }
    @Override
    protected void routine() throws AutoModeEndedException {
        TrajectorySet tSet = TrajectoryGenerator.getInstance().getTrajectorySet();
        runAction(new DriveTrajectory(tSet.driveOffHabReverse.get(mIsLeft)));

        if (mIsLeft)
        {
            runAction(new ZeroOdometryOffHAB(StartPosition.LEFT_PLATFORM));
        }
        else
        {
            runAction(new ZeroOdometryOffHAB(StartPosition.RIGHT_PLATFORM));
        }

        // runAction(new DriveTrajectory(tSet.driveToClosestCargoShipBayFromSide.get(mIsLeft)));
     // runAction(new RunFunctionOnceUntilAction(() -> Superstructure.getInstance().setWantedState(Superstructure.WantedState.EJECT_PANEL),
     //  () -> Superstructure.getInstance().isDriverControlled()));

    }

}