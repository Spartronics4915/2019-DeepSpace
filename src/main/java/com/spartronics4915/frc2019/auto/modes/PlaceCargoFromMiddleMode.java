package com.spartronics4915.frc2019.auto.modes;

import com.spartronics4915.frc2019.auto.AutoModeBase;
import com.spartronics4915.frc2019.auto.AutoModeEndedException;
import com.spartronics4915.frc2019.paths.TrajectoryGenerator;
import com.spartronics4915.frc2019.paths.TrajectoryGenerator.TrajectorySet;
import com.spartronics4915.frc2019.auto.actions.DriveTrajectory;
import com.spartronics4915.frc2019.auto.actions.ZeroOdometryOffHAB;
import com.spartronics4915.frc2019.auto.actions.ZeroOdometryOffHAB.StartPosition;

public class PlaceCargoFromMiddleMode extends AutoModeBase
{
    private final boolean mIsLeft;

    public PlaceCargoFromMiddleMode(boolean isLeft)
    {
        mIsLeft = isLeft;
    }

    @Override
    protected void routine() throws AutoModeEndedException
    {
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

        runAction(new DriveTrajectory(tSet.driveToDriverStationParallelHatch.get(mIsLeft)));
        // runAction(new RunFunctionOnceUntilAction(() -> Superstructure.getInstance().setWantedState(Superstructure.WantedState.EJECT_PANEL),
        //  () -> Superstructure.getInstance().isDriverControlled()));
        // runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().driveToDepot.get(mIsLeft)));
        // runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().driveToClosestCargoShipBay.get(mIsLeft)));

    }

}