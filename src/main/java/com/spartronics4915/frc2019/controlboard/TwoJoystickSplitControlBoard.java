package com.spartronics4915.frc2019.controlboard;

import com.spartronics4915.frc2019.Constants;

import edu.wpi.first.wpilibj.Joystick;

public class TwoJoystickSplitControlBoard implements IDriveControlBoard
{
    private final Joystick mThrottleJoystick;
    private final Joystick mTurnJoystick;

    public TwoJoystickSplitControlBoard()
    {
        mThrottleJoystick = new Joystick(Constants.kDriveJoystickPort);
        mTurnJoystick = new Joystick(3); // TODO: Make this a Constant?
    }

    @Override
    public double getThrottle()
    {
        return mThrottleJoystick.getY() * -1;//this means pushing forwards moves the robot forwards
    }

    @Override
    public double getTurn()
    {
        return mTurnJoystick.getX();
    }

    // TODO: Below methods are unimplemented

    @Override
    public boolean getQuickTurn()
    {
        return false;
    }

    @Override
    public boolean getReturnToDriverControl()
    {
        return false;
    }

    @Override
    public boolean getReverseDirection()
    {
        return false;
    }

    @Override
    public boolean getDriveToSelectedTarget()
    {
        return false;
    }

    @Override
    public boolean getTestButtonOne()
    {
        return false;
    }

    @Override
    public boolean getTestButtonTwo()
    {
        return false;
    }

    @Override
    public boolean getTestButtonThree()
    {
        return false;
    }

    //Vision buttons
    @Override
    public boolean getAssistedIntakeCargo()
    {
        return false;
    }

    @Override
    public boolean getAssistedShootRocket()
    {
        return false;
    }

    @Override
    public boolean getAssistedShootBay()
    {
        return false;
    }

    @Override
    public boolean getAssistedIntakePanel()
    {
        return false;
    }

    @Override
    public boolean getAssistedEjectPanel()
    {
        return false;
    }
}