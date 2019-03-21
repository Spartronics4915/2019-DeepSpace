package com.spartronics4915.frc2019.auto.modes;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.frc2019.auto.AutoModeBase;
import com.spartronics4915.frc2019.auto.AutoModeEndedException;
import com.spartronics4915.frc2019.auto.actions.DriveTrajectory;
import com.spartronics4915.frc2019.paths.TrajectoryGenerator;
import com.spartronics4915.frc2019.paths.TrajectoryGenerator.TrajectorySet;
import com.spartronics4915.frc2019.subsystems.RobotStateEstimator;

public class DriveToDepotTestMode extends AutoModeBase
{
    
    private final boolean mIsLeft;

    public DriveToDepotTestMode(boolean isLeft)
    {
        mIsLeft = isLeft;
    }

    @Override
    protected void routine() throws AutoModeEndedException
    {
        TrajectorySet tSet = TrajectoryGenerator.getInstance().getTrajectorySet();
        
        runAction(new DriveTrajectory(tSet.driveToDepotFromClosestCargoBay.get(mIsLeft), true));
        runAction(new DriveTrajectory(tSet.driveToMiddleCargoBayFromDepot.get(mIsLeft)));

    }

}