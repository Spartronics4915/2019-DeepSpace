package com.spartronics4915.lib.lidar;

import com.spartronics4915.lib.LibConstants;

import com.spartronics4915.lib.lidar.icp.ICP;
import com.spartronics4915.lib.lidar.icp.Point;

import com.spartronics4915.lib.lidar.icp.IReferenceModel;
import com.spartronics4915.lib.lidar.icp.RelativeICPProcessor;
import com.spartronics4915.lib.lidar.icp.Transform;

import com.spartronics4915.lib.util.ILoop;

import com.spartronics4915.lib.geometry.Translation2d;
import com.spartronics4915.lib.geometry.Twist2d;
import com.spartronics4915.lib.geometry.Pose2d;
import com.spartronics4915.lib.util.Logger;
import com.spartronics4915.lib.util.RobotStateMap;

import java.io.DataOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.HashSet;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.DoubleSupplier;
import java.util.concurrent.LinkedBlockingQueue;

import java.util.LinkedHashMap;
import java.util.Map;
import java.net.URI;
import java.net.URISyntaxException;

import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;

/**
 * Receives LIDAR points from the {@link LidarServer}, stores a set number of
 * scans/revolutions, and provides methods for processing the data.
 * <p>
 * All interfacing with the LIDAR should be done through this class.
 *
 * @see Constants.kLidarNumScansToStore
 * @see doICP()
 * @see getTowerPosition()
 */

class WSClient extends WebSocketClient
{
    private boolean isOpen;
    public WSClient() throws URISyntaxException
    {
        //super(new URI("ws://192.168.1.10:5080/webapi/_publish_"));
        super(new URI("ws://localhost:5080/webapi/_publish_"));
        isOpen = false;
    }

    @Override
    public void onOpen(ServerHandshake handshakedata)
    {
        Logger.info("WebSocket open");
        isOpen = true;
    }

    @Override
    public void onClose(int code,  String reason, boolean remote)
    {
        Logger.info("WebSocket close");
        isOpen = false;
    }

    @Override
    public void onError(Exception ex)
    {
        Logger.exception(ex);
    }

    @Override
    public void onMessage(String message)
    { // Logger.info("WebSocket message "  + message);
    }

    public void send(LidarScan scan)
    {
        if(isOpen)
        {
            this.send(scan.toJsonString());
        }
    }
}

public class LidarProcessor implements ILoop 
{
    public enum RunMode
    {
        kRunInRobot,
        kRunAsTest
    };

    enum OperatingMode
    {
        kRelative,
        kAbsolute
    };

    private static boolean sDebugPoints = false;
    private final Pose2d kVehicleToLidar;

    private LidarServer mLidarServer;
    private double mScanTime;
    private double mLastScanTime;
    private double mScanTimeAccum;
    private int mScanCount;
    private ICP mICP; 
    private RelativeICPProcessor mRelativeICP; 
    private DataOutputStream mDataLogFile;
    private final ReadWriteLock mRWLock; 
    private LinkedBlockingQueue<LidarScan> mScanQueue;
    private LidarScan mActiveScan;
    private final OperatingMode mMode = OperatingMode.kRelative;
    private WSClient mWSClient;
    private IReferenceModel mReferenceModel;
    private RobotStateMap mEncoderStateMap;
    private RobotStateMap mLidarStateMap;
    private DoubleSupplier mTimeSupplier;

    // A scan is a collection of lidar points.  The scan, itself,
    // has a timestamp as does each point.  Currently, the timestamp
    // of each point is the same as scan, but this needn't be the case
    // since a full scan requires 1/(5-10hz) seconds. The robot pose
    // is tracked by the application/main and the position of the robot
    // time a point is acquired by interpolating known robot poses to
    // the requested time.  Since this operation is non-trivial and since
    // we assume that multiple LidarPoints are acquired at the "same time",
    // we employ a local cache that maps timestamp to robot pose.
    private final static int MAX_ENTRIES = 10;
    private final static LinkedHashMap<Double, Pose2d> sLidarPoseCache = 
        new LinkedHashMap<Double, Pose2d>() 
    {
        private static final long serialVersionUID = 1L;

        @Override
        protected boolean removeEldestEntry(Map.Entry<Double, Pose2d> eldest)
        {
            return this.size() > MAX_ENTRIES;
        }
    };

    public LidarProcessor(RunMode runMode, IReferenceModel refmodel,
        RobotStateMap encoderStateMap, RobotStateMap lidarStateMap, Pose2d vehicleToLidar, DoubleSupplier timeSupplier) 
    {
        Logger.debug("LidarProcessor starting...");
        mICP = new ICP(LibConstants.kICPTimeoutMs);
        mScanQueue = new LinkedBlockingQueue<LidarScan>();
        mRelativeICP = new RelativeICPProcessor(mICP);
        mRWLock = new ReentrantReadWriteLock();
        mLidarServer = new LidarServer(this, timeSupplier);
        mScanTime = Double.NEGATIVE_INFINITY;
        mLastScanTime = Double.NEGATIVE_INFINITY;
        mScanTimeAccum = 0;
        mScanCount = 0;
        mActiveScan = null;
        mReferenceModel = refmodel; // may be null
        mEncoderStateMap = encoderStateMap;
        mLidarStateMap = lidarStateMap; // This could be the same object as above
        mTimeSupplier = timeSupplier;
        kVehicleToLidar = vehicleToLidar;

        try 
        {
            if(runMode == RunMode.kRunAsTest)
            {
                mWSClient = new WSClient();
                mWSClient.connect();
            }
            if(sDebugPoints)
            {
                mDataLogFile = new DataOutputStream(newLogFile());
                // mDataLogFile = new DataOutputStream(new GZIPOutputStream(newLogFile()));
            }
        } 
        catch (IOException e) 
        {
            Logger.exception(e);
        }
        catch(URISyntaxException e)
        {
            Logger.exception(e);
        }
    }

    public boolean isConnected()
    {
        return mLidarServer.isLidarConnected();
    }

    @Override
    public void onStart(double timestamp) 
    {
    }

    @Override
    public void onLoop(double timestamp) 
    {
        // we're called regularly (100hz) from the looper. 
        if (timestamp - getScanStart() > LibConstants.kLidarRestartTime) 
        {
            if (!mLidarServer.isEnding() && !mLidarServer.isRunning() &&
                mLidarServer.isLidarConnected())  
            {
                if(!mLidarServer.start())   
                {
                    // If server fails to start we update mScanTime to 
                    // ensure we don't restart each loop.
                    startNewScan(timestamp);
                }
            }
        }
        if(mLidarServer.isRunning())
        {
            try
            {
                LidarScan scan = mScanQueue.take(); // consumer blocks
                double scanTime = scan.getTimestamp();
                if(mScanCount > 0)
                    mScanTimeAccum += scanTime - mLastScanTime;
                if(mScanCount%10 == 1)
                {
                    double scansPerSec = mScanCount/mScanTimeAccum;
                    // we might want to log this to SmartDashboard
                    Logger.notice("scan " + mScanCount + 
                                  " npts:" + scan.getPoints().size() +
                                  " scansPerSec:"+ scansPerSec);
                }
                mScanCount++;
                mLastScanTime = scanTime;
                this.processLidarScan(scan);
                if(mWSClient != null)
                    mWSClient.send(scan);
            }
            catch(InterruptedException ie)
            {
            }
        }
    }

    @Override
    public void onStop(double timestamp)
    {
        mLidarServer.stop();
    }

    private void processLidarScan(LidarScan scan)
    {
        try
        {
            Pose2d pose;
            Twist2d vel;
            if(mMode == OperatingMode.kRelative)
            {
                Transform xform = mRelativeICP.doRelativeICP(scan.getPoints());
                if(xform != null)
                {
                    pose = xform.inverse().toPose2d();

                    // in (or radians) / time between scans -> in (or radians) / seconds
                    // Assumes that timestamps are in seconds
                    double conversion = mLastScanTime - mScanTime;
                    vel = new Twist2d(
                        (pose.getTranslation().x() / conversion + pose.getTranslation().y() / conversion) / 2,
                        0.0,
                        pose.getRotation().getRadians() / conversion
                        );

                    pose = pose.transformBy(mLidarStateMap.getLatestFieldToVehicle().getValue());
                }
                else
                {
                    Logger.warning("Relative ICP returned a null transform!");
                    return;
                }
            } 
            else
            {
                Pose2d estimate = mEncoderStateMap.getFieldToVehicle(scan.getTimestamp());
                // need to invoke getFieldToLidar, not getFieldToVehicle
                Transform xform = mICP.doICP(getCulledPoints(scan), 
                                new Transform(estimate).inverse(),  // ie: LidarToField
                                mReferenceModel);
                pose  = xform.inverse().toPose2d();

                Pose2d pose1SecBeforeScan = mLidarStateMap.getFieldToVehicle(scan.getTimestamp() - 1);
                vel = new Twist2d(
                    pose.distance(pose1SecBeforeScan),
                    0,
                    pose.getRotation().distance(pose1SecBeforeScan.getRotation()));
            }

            mLidarStateMap.addObservations(scan.getTimestamp(), pose, vel,
                    Twist2d.identity() /*
                        Predicted velocity is acutually just encoder velocity sampled over
                        a longer time period, and there isn't really an analogue for LIDAR.
                        */);
        }
        catch(Exception e)
        {
            Logger.exception(e);
        }
    }

    private static FileOutputStream newLogFile() throws IOException 
    {
        // delete old files if we're over the limit
        File logDir = new File(LibConstants.kLidarLogDir);
        File[] logFiles = logDir.listFiles();
        if (logFiles == null)
            throw new IOException("List files in " + LibConstants.kLidarLogDir);
        Arrays.sort(logFiles, (f1, f2) -> {
            return Long.compare(f1.lastModified(), f2.lastModified());
        });
        for (int i=0; i<logFiles.length-LibConstants.kNumLidarLogsToKeep+1;i++) 
        {
            logFiles[i].delete();
        }

        // create the new file and return
        String dateStr = new SimpleDateFormat("MM-dd-HH_mm_ss").format(new Date());
        File newFile = new File(logDir, "lidarLog-" + dateStr + ".dat");
        newFile.createNewFile();
        return new FileOutputStream(newFile, false);
    }

    // logPoint is invoked from 
    private void logPoint(double angle, double dist, double x, double y)
    {
        try 
        {
            if(mDataLogFile != null)
            {
                mDataLogFile.writeInt((int) (angle * 100));
                mDataLogFile.writeInt((int) (dist * 256));
                mDataLogFile.writeInt((int) (x * 256));
                mDataLogFile.writeInt((int) (y * 256));
            }
        } 
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }

    // addPoint is invoked from LidarServer::handleLine via the ReaderThread.
    // logging is only invoked from this thread, but scan data is accessed
    // asynchronously from the main thread (which, for example, performs
    // ICP matching). This accounts for the use of our read-write lock on the
    // scan data.  Any changes to mScans or an individual scan should
    // be guarded by this lock.
    public void addPoint(double ts, double angle, double dist, 
                                      boolean newScan) 
    {

        // transform by the robot's pose
        Pose2d robotLoc = null;
        if(this.mMode == OperatingMode.kAbsolute)
        {
            Pose2d robotPose = null;
            if (sLidarPoseCache.containsKey(ts)) 
            {
                robotPose = sLidarPoseCache.get(ts);
            } 
            else
            {
                // This is to correct for time smear
                robotPose = mEncoderStateMap.getFieldToVehicle(ts).transformBy(kVehicleToLidar);
                if(robotPose != null)
                    sLidarPoseCache.put(ts, robotPose);
            }
            robotLoc = robotPose;
        }
        LidarPoint lpt = new LidarPoint(ts, angle, dist); 
        Translation2d cartesian = lpt.toCartesian(robotLoc);
        logPoint(lpt.angle, lpt.distance, cartesian.x(), cartesian.y());
        if (newScan || mActiveScan == null) 
        { 
            if(mActiveScan != null)
                mScanQueue.add(mActiveScan); // <- send it to consumer

            mActiveScan  = new LidarScan();
            startNewScan(mTimeSupplier.getAsDouble());
        }
        if (!excludePoint(cartesian.x(), cartesian.y())) 
        {
            mActiveScan.addPoint(new Point(cartesian), lpt.timestamp);
        }
    }

    // TODO: Pass this from frc2019.Constants
    private static final double FIELD_WIDTH = 27 * 12, FIELD_HEIGHT = 54 * 12;
    private static final double RECT_RX = FIELD_WIDTH / 5, RECT_RY = FIELD_HEIGHT / 2;
    private static final double FIELD_CX = FIELD_WIDTH / 2, FIELD_CY = FIELD_HEIGHT / 2;
    private static final double RECT_X_MIN = FIELD_CX - RECT_RX, RECT_X_MAX = FIELD_CX + RECT_RX,
            RECT_Y_MIN = FIELD_CY - RECT_RY, RECT_Y_MAX = FIELD_CY + RECT_RY;

    private static boolean excludePoint(double x, double y) 
    {
        return false;
        // return x < RECT_X_MIN || x > RECT_X_MAX || y < RECT_Y_MIN || y > RECT_Y_MAX;
    }

    private static final double BUCKET_SIZE = 3.0; // inches

    /**
     * Cantor pairing function (to bucket & hash two doubles)
     * converts two integers into one; used as a hash key
     */
    private int getBucket(double x, double y)
    {
        int ix = (int) (x / BUCKET_SIZE);
        int iy = (int) (y / BUCKET_SIZE);
        int a = ix >= 0 ? 2 * ix : -2 * ix - 1;
        int b = iy >= 0 ? 2 * iy : -2 * iy - 1;
        int sum = a + b;
        return sum * (sum + 1) / 2 + a;
    }

    /**
     * Returns a list of points that have been thinned roughly uniformly.
     */
    private ArrayList<Point> getCulledPoints(LidarScan scan)
    {
        ArrayList<Point> list = new ArrayList<>();
        HashSet<Integer> buckets = new HashSet<>();
        for (Point p : scan.getPoints())
        {
            if (buckets.add(getBucket(p.x, p.y)))
                list.add(p);
        }
        return list;
    }

    public void startNewScan(double time) 
    {
        mRWLock.writeLock().lock();
        try 
        {
            mScanTime = time;
        } 
        finally 
        {
            mRWLock.writeLock().unlock();
        }
    }

    public double getScanStart() 
    {
        mRWLock.readLock().lock();
        try 
        {
            return mScanTime;
        } 
        finally 
        {
            mRWLock.readLock().unlock();
        }
    }

}
