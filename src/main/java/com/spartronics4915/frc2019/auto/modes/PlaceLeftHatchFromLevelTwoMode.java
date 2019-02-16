package com.spartronics4915.frc2019.auto.modes;

import com.spartronics4915.frc2019.auto.AutoConstants;
import com.spartronics4915.frc2019.auto.AutoModeBase;
import com.spartronics4915.frc2019.auto.AutoModeEndedException;
import com.spartronics4915.frc2019.auto.actions.DriveOffHAB;
import com.spartronics4915.frc2019.auto.actions.DriveVelocity;
import com.spartronics4915.frc2019.auto.actions.DriveOffHAB.HABLevel;
import com.spartronics4915.lib.util.DriveSignal;

public class PlaceLeftHatchFromLevelTwoMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new DriveOffHAB(AutoConstants.kDriveOffHabVelocity, AutoConstants.kDriveOffHabFeedforward, HABLevel.PLATFORM));
        runAction(new DriveVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE));
    }

}