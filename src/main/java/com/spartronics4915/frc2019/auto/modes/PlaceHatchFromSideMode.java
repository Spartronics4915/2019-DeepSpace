package com.spartronics4915.frc2019.auto.modes;

import com.spartronics4915.frc2019.auto.AutoModeBase;
import com.spartronics4915.frc2019.auto.AutoModeEndedException;
import com.spartronics4915.frc2019.paths.TrajectoryGenerator;
import com.spartronics4915.frc2019.paths.TrajectoryGenerator.TrajectorySet;
import com.spartronics4915.frc2019.auto.actions.DriveTrajectory;
import com.spartronics4915.frc2019.auto.actions.ZeroOdometryOnHAB;
import com.spartronics4915.frc2019.auto.actions.ZeroOdometryOnHAB.StartPosition;

public class PlaceHatchFromSideMode extends AutoModeBase
{

    private final boolean mIsLeft;

    public PlaceHatchFromSideMode(boolean isLeft)
    {
        mIsLeft = isLeft;
    }

    @Override
    protected void routine() throws AutoModeEndedException
    {
        TrajectorySet tSet = TrajectoryGenerator.getInstance().getTrajectorySet();
        runAction(new DriveTrajectory(tSet.driveToParallelCargoBayFromSide.get(mIsLeft), true));
        // runAction(new RunFunctionOnceUntilAction(() -> Superstructure.getInstance().setWantedState(Superstructure.WantedState.EJECT_PANEL),
        //  () -> Superstructure.getInstance().isDriverControlled()));
        // runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().driveToDepot.get(mIsLeft)));
        // runAction(new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().driveToClosestCargoShipBay.get(mIsLeft)));

    }

}
