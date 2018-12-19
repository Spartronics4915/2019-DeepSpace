package com.spartronics4915.lib;

import java.nio.file.Paths;

/* for non-robot-specific constants/settings --   */
public class LibConstants
{
	/* Lidar non-game settings ----*/
	public static final int kLidarScanSize = 400;
    public static final int kLidarNumScansToStore = 10;
    public static final double kLidarRestartTime = 2.5;
    public static final String kLidarDriverPath = Paths.get(
      System.getProperty("user.home"), "/chezy_lidar/").toString();
    public static final String kLidarLogDir = Paths.get(
      System.getProperty("user.home"), "/lidarlogs/").toString();
    public static final int kNumLidarLogsToKeep = 10;
    public static final double kLidarICPTranslationEpsilon = 0.01; // convergence threshold for tx,ty
    public static final double kLidarICPAngleEpsilon = 0.01;       // convergence threshold for theta

};
