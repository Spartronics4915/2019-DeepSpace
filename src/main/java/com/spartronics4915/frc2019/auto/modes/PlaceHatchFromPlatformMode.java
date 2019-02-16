package com.spartronics4915.frc2019.auto.modes;

import com.spartronics4915.frc2019.auto.AutoConstants;
import com.spartronics4915.frc2019.auto.AutoModeBase;
import com.spartronics4915.frc2019.auto.AutoModeEndedException;
import com.spartronics4915.frc2019.paths.TrajectoryGenerator;
import com.spartronics4915.frc2019.auto.actions.DriveOffHAB;
import com.spartronics4915.frc2019.auto.actions.DriveTrajectory;
import com.spartronics4915.frc2019.auto.actions.RunFunctionOnceAction;
import com.spartronics4915.frc2019.auto.actions.ZeroOdometryOffHAB;
import com.spartronics4915.frc2019.auto.actions.DriveOffHAB.HABLevel;
import com.spartronics4915.frc2019.subsystems.Superstructure;

public class PlaceHatchFromPlatformMode extends AutoModeBase
{

    private final boolean mIsLeft;

    public PlaceHatchFromPlatformMode(boolean isLeft)
    {
        mIsLeft = isLeft;
    }

    @Override
    protected void routine() throws AutoModeEndedException
    {
        runAction(new DriveOffHAB(AutoConstants.kDriveOffHabVelocity, AutoConstants.kDriveOffHabFeedforward, HABLevel.PLATFORM));
        runAction(new ZeroOdometryOffHAB(mIsLeft));
        runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().driveToDriverStationParallelHatch.get(mIsLeft)));
        runAction(new RunFunctionOnceAction(() -> Superstructure.getInstance().setWantedState(Superstructure.WantedState.EJECT_PANEL)));
    }

}
