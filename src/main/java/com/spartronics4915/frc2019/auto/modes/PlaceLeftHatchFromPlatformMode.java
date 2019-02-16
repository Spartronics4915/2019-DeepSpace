package com.spartronics4915.frc2019.auto.modes;

import com.spartronics4915.frc2019.auto.AutoConstants;
import com.spartronics4915.frc2019.auto.AutoModeBase;
import com.spartronics4915.frc2019.auto.AutoModeEndedException;
import com.spartronics4915.frc2019.paths.TrajectoryGenerator;

public class PlaceLeftHatchFromPlatformMode extends AutoModeBase
{

    @Override
    protected void routine() throws AutoModeEndedException
    {
        runAction(AutoConstants.getPlaceHatchAction(TrajectoryGenerator.getInstance().getTrajectorySet().driveToDriverStationParallelHatch.left));
    }

}
