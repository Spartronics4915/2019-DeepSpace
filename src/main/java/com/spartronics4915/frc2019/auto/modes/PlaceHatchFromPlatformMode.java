package com.spartronics4915.frc2019.auto.modes;

import com.spartronics4915.frc2019.auto.AutoConstants;
import com.spartronics4915.frc2019.auto.AutoModeBase;
import com.spartronics4915.frc2019.auto.AutoModeEndedException;
import com.spartronics4915.frc2019.paths.TrajectoryGenerator;
import com.spartronics4915.frc2019.auto.actions.DriveOffHAB;
import com.spartronics4915.frc2019.auto.actions.DriveTrajectory;
import com.spartronics4915.frc2019.auto.actions.ParallelAction;
import com.spartronics4915.frc2019.auto.actions.RunFunctionOnceUntilAction;
import com.spartronics4915.frc2019.auto.actions.ZeroOdometryOffHAB;
import com.spartronics4915.frc2019.auto.actions.DriveOffHAB.HABLevel;
import com.spartronics4915.frc2019.auto.actions.ZeroOdometryOffHAB.StartPosition;
import com.spartronics4915.frc2019.subsystems.Superstructure;
import com.spartronics4915.lib.trajectory.Trajectory;

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
        // if (mIsLeft)
        // {
        //     runAction(new ZeroOdometryOffHAB(StartPosition.LEFT_PLATFORM));
        // }
        // else
        // {
        //     runAction(new ZeroOdometryOffHAB(StartPosition.RIGHT_PLATFORM));
        // }

        runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().driveToDriverStationParallelHatch.get(mIsLeft)));
        // runAction(new RunFunctionOnceUntilAction(() -> Superstructure.getInstance().setWantedState(Superstructure.WantedState.EJECT_PANEL),
        //  () -> Superstructure.getInstance().isDriverControlled()));
        // runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().driveToDepot.get(mIsLeft)));
        // runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().driveToClosestCargoShipBay.get(mIsLeft)));

    }

}
