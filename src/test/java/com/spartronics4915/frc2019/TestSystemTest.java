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

        // The below is corrected for the robot's length so that it is moved towards the center of the
        // field (i.e. +x). See the bottom of Constants.java for more
        System.out.println(Constants.kRightRobotLocationOnPlatform);

        // The below will get the pose of LEFT_CLOSE_CARGO_BAY, corrected for half of the robot's length
        // This will move the pose up (+y) because the angle of the original pose of LEFT_CLOSE_CARGO_BAY is 90
        System.out.println(Constants.ScorableLandmark.LEFT_CLOSE_CARGO_BAY.robotLengthCorrectedPose);
    }
}
