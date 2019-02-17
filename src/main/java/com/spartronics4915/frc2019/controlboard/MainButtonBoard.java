package com.spartronics4915.frc2019.controlboard;

import com.spartronics4915.frc2019.Constants;
import edu.wpi.first.wpilibj.Joystick;

public class MainButtonBoard implements IButtonControlBoard
{
    private final Joystick mButtonBoard;
    private final Joystick mTestButtonBoard;
    
    private double mPreviousAxis0;
    private double mPreviousAxis1;
    private double mPreviousAxis2;
    private double mPreviousAxis3;

    private double current;
    private boolean result;

    public MainButtonBoard()
    {
        mButtonBoard = new Joystick(Constants.kMainButtonBoardPort);
        mTestButtonBoard = new Joystick(3);

        mPreviousAxis0 = 0.0;
        mPreviousAxis1 = 0.0;
        mPreviousAxis2 = 0.0;
        mPreviousAxis3 = 0.0;

        current = 0.0;
        result = false;
    }


    //TODO: assign these buttons

    // public boolean REFERENCEAXISMETHOD()
    // {
        // current = mButtonBoard.getRawAxis(1);
        // result = (mPreviousAxis1 != current) && (current == 1.0);
        // mPreviousAxis1 = current;
        // return result;
    // }

    @Override
    public boolean getClimb()
    {
        // return mButtonBoard.getRawButtonPressed(1);
        return false;
    }

    @Override
    public boolean getManualExtendAllClimbPneumatics()
    {
        return mButtonBoard.getRawButtonPressed(15);
    }

    @Override
    public boolean getAssistedIntakeCargo()
    {
        return mButtonBoard.getRawButtonPressed(2);
    }

    @Override
    public boolean getGroundEjectCargo()
    {
        return mButtonBoard.getRawButtonPressed(3);
    }

    @Override
    public boolean getManualRamp()
    {
        return mButtonBoard.getRawButtonPressed(4);
    }

    @Override
    public boolean getAssistedShootRocket()
    {
        return mButtonBoard.getRawButtonPressed(5);
    }

    @Override
    public boolean getAssistedShootBay()
    {
        return mButtonBoard.getRawButtonPressed(6);
    }

    @Override
    public boolean getSelectLeftVisionTarget()
    {
        return mButtonBoard.getRawButtonPressed(7);
    }

    @Override
    public boolean getSelectRightVisionTarget()
    {
        return mButtonBoard.getRawButtonPressed(8);
    }

    @Override
    public boolean getManualShootCargoBay()
    {
        return mButtonBoard.getRawButtonPressed(9);
    }

    @Override
    public boolean getManualShootCargoRocket()
    {
        return mButtonBoard.getRawButtonPressed(10);
    }

    @Override
    public boolean getManualChuteUp()
    {
        current = mButtonBoard.getRawAxis(2);
        result = (mPreviousAxis2 != current) && (current == 1.0);
        mPreviousAxis2 = current;
        return result;
    }

    @Override
    public boolean getManualChuteDown()
    {
        current = mButtonBoard.getRawAxis(3);
        result = (mPreviousAxis3 != current) && (current == 1.0);
        mPreviousAxis3 = current;
        return result;
    }

    @Override
    public boolean getAssistedIntakePanel()
    {
        current = mButtonBoard.getRawAxis(0);
        result = (mPreviousAxis0 != current) && (current == 1.0);
        mPreviousAxis0 = current;
        return result;
    }

    @Override
    public boolean getAssistedEjectPanel()
    {
        current = mButtonBoard.getRawAxis(0);
        result = (mPreviousAxis0 != current) && (current == -1.0);
        mPreviousAxis0 = current;
        return result;
    }

    @Override
    public boolean getManualEjectPanel()
    {
        current = mButtonBoard.getRawAxis(1);
        result = (mPreviousAxis1 != current) && (current == 1.0);
        mPreviousAxis1 = current;
        return result;
    }

    @Override
    public boolean getInsideFramePerimeter()
    {
        current = mButtonBoard.getRawAxis(1);
        result = (mPreviousAxis1 != current) && (current == -1.0);
        mPreviousAxis1 = current;
        return result;
    }



    //TEST BUTTON BOARD
    @Override
    public boolean getTESTClimbExtendAllPneumatics()
    {
        return mButtonBoard.getRawButtonPressed(5);
    }

    @Override
    public boolean getTESTClimbIntake()
    {
        return mButtonBoard.getRawButtonPressed(3);
    }

    @Override
    public boolean getTESTClimbRetractFrontPneumatics()
    {
        return mButtonBoard.getRawButtonPressed(4);
    }

    @Override
    public boolean getTESTClimbRetractBackPneumatics()
    {
        return mButtonBoard.getRawButtonPressed(6);
    }
}
