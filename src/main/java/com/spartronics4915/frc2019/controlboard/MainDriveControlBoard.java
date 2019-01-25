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

    @Override
    public boolean getReturnToDriverControl()
    {
        return mDriveJoystick.getRawButtonReleased(2);
    }

    @Override
    public boolean getDriveToSelectedTarget()
    {
        return mDriveJoystick.getRawButtonReleased(7);
    }

    @Override
    public boolean getReverseDirection()
    {
        return mDriveJoystick.getRawButtonReleased(6);
    }

    @Override
    public boolean getTestButtonOne() {
        return mDriveJoystick.getRawButtonReleased(3);
    }

    @Override
    public boolean getTestButtonTwo() {
        return mDriveJoystick.getRawButtonReleased(4);
    }

    @Override
    public boolean getTestButtonThree() {
        return mDriveJoystick.getRawButtonReleased(5);
    }
}
