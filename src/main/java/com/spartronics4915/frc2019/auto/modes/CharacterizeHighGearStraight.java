package com.spartronics4915.frc2019.auto.modes;

import com.spartronics4915.frc2019.auto.AutoModeBase;
import com.spartronics4915.frc2019.auto.AutoModeEndedException;
import com.spartronics4915.frc2019.auto.actions.CollectAccelerationData;
import com.spartronics4915.frc2019.auto.actions.CollectVelocityData;
import com.spartronics4915.frc2019.auto.actions.WaitAction;
import com.spartronics4915.lib.physics.DriveCharacterization;
import com.spartronics4915.lib.util.Logger;

import java.util.ArrayList;
import java.util.List;

public class CharacterizeHighGearStraight extends AutoModeBase
{

    @Override
    protected void routine() throws AutoModeEndedException
    {
        List<DriveCharacterization.VelocityDataPoint> velocityData = new ArrayList<>();
        List<DriveCharacterization.AccelerationDataPoint> accelerationData = new ArrayList<>();

        runAction(new CollectVelocityData(velocityData, false, true));
        runAction(new WaitAction(10));
        runAction(new CollectAccelerationData(accelerationData, false, true));

        DriveCharacterization.CharacterizationConstants constants = DriveCharacterization.characterizeDrive(velocityData, accelerationData);

        Logger.notice("ks: " + constants.ks);
        Logger.notice("kv: " + constants.kv);
        Logger.notice("ka: " + constants.ka);
    }
}
