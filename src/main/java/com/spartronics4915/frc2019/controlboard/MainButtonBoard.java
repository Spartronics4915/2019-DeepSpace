package com.spartronics4915.frc2019.controlboard;

import com.spartronics4915.frc2019.Constants;
import edu.wpi.first.wpilibj.Joystick;

public class MainButtonBoard implements IButtonControlBoard
{
    private final Joystick mButtonBoard;

    public MainButtonBoard()
    {
        mButtonBoard = new Joystick(Constants.kMainButtonBoardPort);
    }


    //TODO: assign these buttons

    @Override
    public boolean getClimb()
    {
        return mButtonBoard.getRawButtonPressed(1);
    }

    @Override
    public boolean getManualExtendAllClimbPneumatics()
    {
        return mButtonBoard.getRawButtonPressed(2);
    }

    @Override
    public boolean getAssistedIntakeCargo()
    {
        return mButtonBoard.getRawButtonPressed(3);
    }

    @Override
    public boolean getGroundEjectCargo()
    {
        return mButtonBoard.getRawButtonPressed(4);
    }

    @Override
    public boolean getManualRamp()
    {
        return mButtonBoard.getRawButtonPressed(5);
    }

    @Override
    public boolean getAssistedShootRocket()
    {
        return mButtonBoard.getRawButtonPressed(6);
    }

    @Override
    public boolean getAssistedShootBay()
    {
        return mButtonBoard.getRawButtonPressed(7);
    }

    @Override
    public boolean getManualShootCargo()
    {
        return mButtonBoard.getRawButtonPressed(8);
    }

    @Override
    public boolean getManualChuteUp()
    {
        return mButtonBoard.getRawButtonPressed(9);
    }

    @Override
    public boolean getManualChuteDown()
    {
        return mButtonBoard.getRawButtonPressed(10);
    }

    @Override
    public boolean getAssistedIntakePanel()
    {
        return mButtonBoard.getRawButtonPressed(11);
    }

    @Override
    public boolean getAssistedEjectPanel()
    {
        return mButtonBoard.getRawButtonPressed(12);
    }

    @Override
    public boolean getManualEjectPanel()
    {
        return mButtonBoard.getRawButtonPressed(13);
    }

    @Override
    public boolean getInsideFramePerimeter()
    {
        return mButtonBoard.getRawButtonPressed(14);
    }

    
}
