package com.spartronics4915.frc2019.controlboard;

import com.spartronics4915.frc2019.Constants;

import edu.wpi.first.wpilibj.Joystick;

public class OneJoystickControlBoard implements IDriveControlBoard
{

    private final Joystick mJoystick;

    public OneJoystickControlBoard()
    {
        mJoystick = new Joystick(Constants.kDriveJoystickPort);
    }

    @Override
    public double getThrottle()
    {
        return mJoystick.getY();
    }

    @Override
    public double getTurn()
    {
        return mJoystick.getX();
    }

    @Override
    public boolean getQuickTurn()
    {
        return mJoystick.getRawButtonPressed(1);
    }

    @Override
    public boolean getReturnToDriverControl()
    {
        return mJoystick.getRawButtonReleased(2);
    }

    @Override
    public boolean getDriveToSelectedTarget()
    {
        return mJoystick.getRawButtonReleased(7);
    }

    @Override
    public boolean getReverseDirection()
    {
        return mJoystick.getRawButtonReleased(6);
    }

    @Override
    public boolean getTestButtonOne() {
        return mJoystick.getRawButtonReleased(3);
    }

    @Override
    public boolean getTestButtonTwo() {
        return mJoystick.getRawButtonReleased(4);
    }

    @Override
    public boolean getTestButtonThree() {
        return mJoystick.getRawButtonReleased(5);
    }
}
