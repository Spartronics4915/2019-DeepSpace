package com.spartronics4915.frc2019.auto;

import java.util.Arrays;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.frc2019.auto.actions.Action;
import com.spartronics4915.frc2019.auto.actions.DriveOffHAB;
import com.spartronics4915.frc2019.auto.actions.DriveTrajectory;
import com.spartronics4915.frc2019.auto.actions.RunFunctionOnceAction;
import com.spartronics4915.frc2019.auto.actions.SeriesAction;
import com.spartronics4915.frc2019.auto.actions.ZeroOdometryOffHAB;
import com.spartronics4915.frc2019.auto.actions.DriveOffHAB.HABLevel;
import com.spartronics4915.frc2019.subsystems.Superstructure;
import com.spartronics4915.lib.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.trajectory.Trajectory;
import com.spartronics4915.lib.trajectory.timing.TimedState;

public class AutoConstants
{

    public static final double kDriveOffHabVelocity = 24;
    // Feedforward is sort of wrong for the right side
    public static final double kDriveOffHabFeedforward =
            kDriveOffHabVelocity * Constants.kDriveLeftKv + Math.copySign(Constants.kDriveLeftVIntercept, kDriveOffHabVelocity);

    // Code duplication bad
    public static final Action getPlaceHatchAction(Trajectory<TimedState<Pose2dWithCurvature>> traj)
    {
        return new SeriesAction(Arrays.asList(new Action[] {
            new DriveOffHAB(AutoConstants.kDriveOffHabVelocity, AutoConstants.kDriveOffHabFeedforward, HABLevel.PLATFORM),
            new ZeroOdometryOffHAB(true),
            new DriveTrajectory(traj),
            new RunFunctionOnceAction(() -> Superstructure.getInstance().setWantedState(Superstructure.WantedState.EJECT_PANEL)),
        }));
    }
}
