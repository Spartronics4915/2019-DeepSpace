package com.spartronics4915.frc2019.auto.modes;

import com.spartronics4915.frc2019.auto.AutoModeBase;
import com.spartronics4915.frc2019.auto.AutoModeEndedException;
import com.spartronics4915.frc2019.auto.actions.DriveOffHAB;
import com.spartronics4915.frc2019.auto.actions.DriveOffHAB.HABLevel;

public class DriveOffHABTestMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new DriveOffHAB(0, 0, HABLevel.PLATFORM));
    }

}