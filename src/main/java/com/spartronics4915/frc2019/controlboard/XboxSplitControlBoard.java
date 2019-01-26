package com.spartronics4915.frc2019.controlboard;

import com.spartronics4915.frc2019.Constants;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class XboxSplitControlBoard implements IDriveControlBoard
{
    private final XboxController mController;

    public XboxSplitControlBoard()
    {
        mController = new XboxController(Constants.kDriveJoystickPort);
    }

    @Override
    public double getThrottle()
    {
        // TODO: Is this the right way to do this?
        return mController.getX(Hand.kLeft);
    }

    @Override
    public double getTurn()
    {
        return mController.getX(Hand.kRight);
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
}