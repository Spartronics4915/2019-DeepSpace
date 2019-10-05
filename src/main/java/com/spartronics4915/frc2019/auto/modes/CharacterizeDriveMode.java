package com.spartronics4915.frc2019.auto.modes;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.frc2019.auto.AutoModeBase;
import com.spartronics4915.frc2019.auto.AutoModeEndedException;
import com.spartronics4915.frc2019.auto.actions.CollectAccelerationData;
import com.spartronics4915.frc2019.auto.actions.CollectVelocityData;
import com.spartronics4915.frc2019.auto.actions.WaitAction;
import com.spartronics4915.frc2019.subsystems.Drive;
import com.spartronics4915.lib.physics.DriveCharacterization;
import com.spartronics4915.lib.physics.DriveCharacterization.CharacterizationConstants;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.util.Units;

import java.util.ArrayList;
import java.util.List;

public class CharacterizeDriveMode extends AutoModeBase
{

    public enum SideToCharacterize
    {
        BOTH, LEFT, RIGHT;

        public double getVelocityTicksPer100ms(Drive drive)
        {
            switch (this)
            {
                case BOTH:
                    return (Math.abs(drive.getLeftVelocityTicksPer100ms()) +
                            Math.abs(drive.getRightVelocityTicksPer100ms())) / 2;
                case LEFT:
                    return Math.abs(drive.getLeftVelocityTicksPer100ms());
                case RIGHT:
                    return Math.abs(drive.getRightVelocityTicksPer100ms());
                default:
                    Logger.warning("Invalid side to characterize " + this);
                    return 0.0;
            }
        }

        public double getVoltage(Drive drive)
        {
            switch (this)
            {
                case BOTH:
                    return (drive.getLeftOutputVoltage() + drive.getRightOutputVoltage()) / 2;
                case LEFT:
                    return drive.getLeftOutputVoltage();
                case RIGHT:
                    return drive.getRightOutputVoltage();
                default:
                    Logger.error("Unrecognized side to characterize " + this.toString());
                    return -1;
            }
        }
    }

    private final static boolean kReverse = true;
    private final SideToCharacterize mSide;

    public CharacterizeDriveMode(SideToCharacterize side)
    {
        mSide = side;
    }

    @Override
    protected void routine() throws AutoModeEndedException
    {
        Logger.debug("Characterizing drivetrain: " + mSide);

        List<DriveCharacterization.VelocityDataPoint> linearVelData = new ArrayList<>();
        List<DriveCharacterization.AccelerationDataPoint> linearAccData = new ArrayList<>();
        List<DriveCharacterization.AccelerationDataPoint> angularAccData = new ArrayList<>();

        boolean turnInPlace;

        // First collect linear data
        Logger.debug("Collecting linear data");
        turnInPlace = false;
        runAction(new CollectVelocityData(linearVelData, kReverse, turnInPlace, mSide));
        runAction(new WaitAction(10));
        runAction(new CollectAccelerationData(linearAccData, kReverse, turnInPlace, mSide));
        runAction(new WaitAction(10));

        // Next collect angular data... It doesn't make sense to do this unless we're characterizing both sides.
        if (mSide == SideToCharacterize.BOTH)
        {
            Logger.debug("Collecting angular data");
            turnInPlace = true;
            runAction(new CollectAccelerationData(angularAccData, kReverse, turnInPlace, mSide));
            double moi = DriveCharacterization.calculateAngularInertia(linearAccData, angularAccData,
                    Units.inches_to_meters(Constants.kDriveWheelRadiusInches), Constants.kRobotLinearInertia);
            Logger.notice("J: " + moi);
        }

        CharacterizationConstants constants = DriveCharacterization.characterizeDrive(linearVelData, linearAccData);
        Logger.notice("Ks: " + constants.ks);
        Logger.notice("Kv: " + constants.kv);
        Logger.notice("Ka: " + constants.ka);
    }
}
