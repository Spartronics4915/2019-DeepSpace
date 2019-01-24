package com.spartronics4915.frc2019.auto.modes;

import com.spartronics4915.frc2019.auto.AutoModeBase;
import com.spartronics4915.frc2019.auto.AutoModeEndedException;
import com.spartronics4915.frc2019.auto.actions.CollectAccelerationData;
import com.spartronics4915.frc2019.auto.actions.CollectVelocityData;
import com.spartronics4915.frc2019.auto.actions.WaitAction;
import com.spartronics4915.frc2019.subsystems.Drive;
import com.spartronics4915.lib.physics.DriveCharacterization;
import com.spartronics4915.lib.util.Logger;

import java.util.ArrayList;
import java.util.List;

public class CharacterizeDriveMode extends AutoModeBase
{

    private static final boolean kAlwaysRunBothMotors = true;

    public enum SideToCharacterize
    {
        BOTH, LEFT, RIGHT;

        public double getVelocityTicksPer100ms(Drive drive)
        {
            switch (this)
            {
                case BOTH:
                    return (
                        Math.abs(drive.getLeftVelocityTicksPer100ms()) +
                        Math.abs(drive.getRightVelocityTicksPer100ms())
                    ) / 2;
                case LEFT:
                    return Math.abs(drive.getLeftVelocityTicksPer100ms());
                case RIGHT:
                    return Math.abs(drive.getRightVelocityTicksPer100ms());
                default:
                    Logger.warning("Invalid side to characterize " + this);
                    return 0.0;
            }
        }

        public boolean shouldRunRight()
        {
            return this == RIGHT || kAlwaysRunBothMotors || this == BOTH;
        }

        public boolean shouldRunLeft()
        {
            return this == LEFT || kAlwaysRunBothMotors || this == BOTH;
        }
    }
    
    private final SideToCharacterize mSide;

    public CharacterizeDriveMode(SideToCharacterize side)
    {
        mSide = side;
    }

    @Override
    protected void routine() throws AutoModeEndedException
    {
        Logger.debug("Characterizing drivetrain: " + mSide);

        List<DriveCharacterization.VelocityDataPoint> velocityData = new ArrayList<>();
        List<DriveCharacterization.AccelerationDataPoint> accelerationData = new ArrayList<>();

        runAction(new CollectVelocityData(velocityData, false, true, mSide));
        runAction(new WaitAction(10));
        runAction(new CollectAccelerationData(accelerationData, false, true, mSide));

        DriveCharacterization.CharacterizationConstants constants = DriveCharacterization.characterizeDrive(velocityData, accelerationData);

        Logger.notice("ks: " + constants.ks);
        Logger.notice("kv: " + constants.kv);
        Logger.notice("ka: " + constants.ka);
    }
}
