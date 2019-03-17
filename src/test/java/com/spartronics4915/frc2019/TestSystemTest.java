package com.spartronics4915.frc2019;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class TestSystemTest
{

    @Test
    public void aTest()
    {
        // assert statements
        assertEquals(0, 0, "0 must be 0");

        System.out.println(Constants.kRightRobotLocationOnPlatform + " " + Constants.ScorableLandmark.RIGHT_CLOSE_CARGO_BAY.robotLengthCorrectedPose);
    }
}
