package com.spartronics4915.frc2019.controlboard;

import com.spartronics4915.frc2019.Constants;
import edu.wpi.first.wpilibj.Joystick;

public class MainDriveControlBoard implements IDriveControlBoard
{

    private static MainDriveControlBoard mInstance = null;

    public static MainDriveControlBoard getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new MainDriveControlBoard();
        }
        return mInstance;
    }

    private final Joystick mDriveJoystick;

    private MainDriveControlBoard()
    {
        mDriveJoystick = new Joystick(Constants.kDriveJoystickPort);
    }

    @Override
    public double getThrottle()
    {
        return mDriveJoystick.getY();
    }

    @Override
    public double getTurn()
    {
        return mDriveJoystick.getX();
    }

    @Override
    public boolean getQuickTurn()
    {
        return mDriveJoystick.getRawButtonPressed(1);
    }
}
