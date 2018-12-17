package com.spartronics4915.frc2019.auto.modes;

import com.spartronics4915.frc2019.auto.AutoModeBase;
import com.spartronics4915.frc2019.auto.AutoModeEndedException;
import com.spartronics4915.frc2019.auto.actions.DriveVelocity;
import com.spartronics4915.frc2019.subsystems.Drive;
import com.spartronics4915.lib.util.DriveSignal;

public class VelocityTestMode extends AutoModeBase
{

    private static final double kTestVelocity = 3.0; // inches/sec
    private static final double kTestFeedforward = 0.0; // volts/sec

    @Override
    protected void routine() throws AutoModeEndedException
    {
            Drive.getInstance().startLogging();

            runAction(new DriveVelocity(new DriveSignal(kTestVelocity, kTestVelocity), new DriveSignal(kTestFeedforward, kTestFeedforward)));

            Drive.getInstance().stopLogging();
    }
}
