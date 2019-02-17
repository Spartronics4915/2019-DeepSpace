package com.spartronics4915.frc2019.controlboard;

import com.spartronics4915.frc2019.Constants;
import edu.wpi.first.wpilibj.Joystick;

public class MainButtonBoard implements IButtonControlBoard
{
    private final Joystick mButtonBoard;
    private final Joystick mTestButtonBoard;
    
    private double mPreviousAxis1;
    private double mPreviousAxis2;
    private double mPreviousAxis3;
    private double mPreviousAxis4;

    private double current;
    private boolean result;

    public MainButtonBoard()
    {
        mButtonBoard = new Joystick(Constants.kMainButtonBoardPort);
        mTestButtonBoard = new Joystick(3);

        mPreviousAxis1 = 0.0;
        mPreviousAxis2 = 0.0;
        mPreviousAxis3 = 0.0;
        mPreviousAxis4 = 0.0;

        current = 0.0;
        result = false;
    }


    //TODO: assign these buttons

    // public boolean REFERENCEAXISMETHOD()
    // {
    //     current = mButtonBoard.getRawAxis(1);
    //     result = (mPreviousAxis1 != current) && (current == 1.0);
    //     mPreviousAxis1 = current;
    //     return result;
    // }

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
    public boolean getSelectLeftVisionTarget()
    {
        return mButtonBoard.getRawButtonPressed(16);
    }

    @Override
    public boolean getSelectRightVisionTarget()
    {
        return mButtonBoard.getRawButtonPressed(17);
    }

    @Override
    public boolean getManualShootCargoBay()
    {
        return mButtonBoard.getRawButtonPressed(8);
    }

    @Override
    public boolean getManualShootCargoRocket()
    {
        return mButtonBoard.getRawButtonPressed(9);
    }

    @Override
    public boolean getManualChuteUp()
    {
        return mButtonBoard.getRawButtonPressed(10);
    }

    @Override
    public boolean getManualChuteDown()
    {
        return mButtonBoard.getRawButtonPressed(11);
    }

    @Override
    public boolean getAssistedIntakePanel()
    {
        return mButtonBoard.getRawButtonPressed(12);
    }

    @Override
    public boolean getAssistedEjectPanel()
    {
        return mButtonBoard.getRawButtonPressed(13);
    }

    @Override
    public boolean getManualEjectPanel()
    {
        return mButtonBoard.getRawButtonPressed(14);
    }

    @Override
    public boolean getInsideFramePerimeter()
    {
        return mButtonBoard.getRawButtonPressed(15);
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
