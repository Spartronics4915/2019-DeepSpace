package com.spartronics4915.frc2019.auto.modes;

import java.util.Arrays;

import com.spartronics4915.frc2019.auto.AutoModeBase;
import com.spartronics4915.frc2019.auto.AutoModeEndedException;
import com.spartronics4915.frc2019.paths.TrajectoryGenerator;
import com.spartronics4915.frc2019.paths.TrajectoryGenerator.TrajectorySet;
import com.spartronics4915.frc2019.subsystems.CargoChute;
import com.spartronics4915.frc2019.subsystems.Drive;
import com.spartronics4915.frc2019.subsystems.Superstructure;
import com.spartronics4915.frc2019.auto.actions.DriveTrajectory;
import com.spartronics4915.frc2019.auto.actions.ParallelAction;
import com.spartronics4915.frc2019.auto.actions.RunFunctionOnceUntilAction;

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
        Superstructure superstructure = Superstructure.getInstance();
        CargoChute cargoChute = CargoChute.getInstance();

        runAction(new DriveTrajectory(tSet.driveToClosestCargoBayFromSide.get(mIsLeft), true));
        runAction(new RunFunctionOnceUntilAction(
                () -> superstructure.setWantedState(Superstructure.WantedState.SHOOT_CARGO_BAY),
                () -> superstructure.isDriverControlled()));
        runAction(
                new ParallelAction(Arrays.asList(
                    new DriveTrajectory(tSet.driveToDepotFromClosestCargoBay.get(mIsLeft)),
                    new RunFunctionOnceUntilAction(
                        () -> superstructure.setWantedState(Superstructure.WantedState.INTAKE_CARGO), () -> {
                            return cargoChute.atTarget() && Drive.getInstance().isDoneWithTrajectory();
                        })
            )));
        runAction(new DriveTrajectory(tSet.driveToMiddleCargoBayFromDepot.get(mIsLeft)));
    }

}
