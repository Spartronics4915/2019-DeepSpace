package com.spartronics4915.lib.lidar;

import com.spartronics4915.lib.LibConstants;
import com.spartronics4915.lib.util.Logger;

import java.io.BufferedReader;
import java.io.EOFException;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.File;

/**
 * Starts the <code>chezy_lidar</code> C++ program, parses its
 * output, and feeds the LIDAR points to the {@link LidarProcessor}.
 * <p>
 * Once started, a separate thread reads the stdout of the
 * <code>chezy_lidar</code> process and parses the (angle, distance)
 * values in each line. Each resulting {@link LidarPoint} is passed
 * to {@link LidarProcessor.addPoint(...)}.
 */
public class LidarServer 
{
    private final LidarProcessor mLidarProcessor;
    private static BufferedReader mBufferedReader;
    private boolean mRunning = false;
    private Thread mThread;
    private Process mProcess;
    private boolean mEnding = false;
    private File mDevFile;

    public LidarServer(LidarProcessor p)
    {
        mLidarProcessor = p;
        String os = System.getProperty("os.name").toLowerCase();
        String dev;
        if(os.indexOf("mac") >= 0)
            dev = "/dev/tty.SLAB_USBtoUART";
        else
        if(os.indexOf("linux") >= 0)
            dev = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0";
        else
            dev = "COM9";  // wouldn't work
        mDevFile = new File(dev);

        Logger.debug("LIDAR server constructed (" + os + ")");;
    }

    public boolean isLidarConnected()
    {
        // premise is: if the device becomes unresponsive or disconnected
        //   that the driver dev file disappearance.  Verified on macos and 
        //   raspi.
        if(mDevFile.exists())
            return true;
        else
            return false;
    }

    public boolean start()
    {
        if (!isLidarConnected())
        {
            Logger.error("Cannot start LidarServer: not connected");
            return false;
        }
        synchronized (this) 
        {
            if (mRunning) 
            {
                Logger.error("Cannot start LidarServer: already running");
                return false;
            }
            if (mEnding)
            {
                Logger.error("Cannot start LidarServer: thread ending");
                return false;
            }
            mRunning = true;
        }

        Logger.notice("LidarServer starting subprocess " + LibConstants.kLidarDriverPath);
        try
        {
            mProcess = new ProcessBuilder().command(LibConstants.kLidarDriverPath).start();
            mThread = new Thread(new ReaderThread());
            InputStreamReader reader = new InputStreamReader(mProcess.getInputStream());
            mBufferedReader = new BufferedReader(reader);
            mThread.start();
        } 
        catch (Exception e)
        {
            Logger.exception(e);
        }
        return true;
    }

    public boolean stop()
    {
        synchronized (this)
        {
            if (!mRunning)
            {
                Logger.error("Cannot stop LidarServer: not running");
                return false;
            }
            mRunning = false;
            mEnding = true;
        }

        Logger.notice("Stopping Lidar...");

        try
        {
            mProcess.destroyForcibly();
            mProcess.waitFor();
            mThread.join();
        }
        catch (InterruptedException e) 
        {
            Logger.error("Error: Interrupted while stopping lidar");
            Logger.exception(e);
            synchronized (this)
            {
                mEnding = false;
            }
            return false;
        }
        Logger.notice("Lidar Stopped");
        synchronized (this)
        {
            mEnding = false;
        }
        return true;
    }

    public synchronized boolean isRunning()
    {
        return mRunning;
    }

    public synchronized boolean isEnding()
    {
        return mEnding;
    }

    private void handleLine(String line) 
    {
        // NB: this method is invoked in the ReaderThread.
        boolean isNewScan = line.substring(line.length() - 1).equals("s");
        if (isNewScan)
        {
            line = line.substring(0, line.length() - 1);
        }
        String[] parts = line.split(",");
        if (parts.length == 3)
        {
            try 
            {
                // It is assumed that ts is in sync with our system's clock
                long ts = Long.parseLong(parts[0]);
                // Timestamps are in seconds, so we have to convert
                // We don't need to do the msAgo thing like in the actual codebase
                // because our timestamps use the Unix epoch, and the timestamps
                // from the sensor also use the Unix epoch. In the actual codebase,
                // timestamps use the system start up as an epoch.
                double normalizedTs = ts / 1000d;
                double angle = Double.parseDouble(parts[1]);
                double distance = Double.parseDouble(parts[2]);
                if (distance != 0)
                {
                    mLidarProcessor.addPoint(normalizedTs, angle, distance, isNewScan);
                }
            } 
            catch (java.lang.NumberFormatException e)
            {
                Logger.exception(e);
            }
        }
        else
            Logger.debug(line);
    }

    private class ReaderThread implements Runnable
    {
        // This method runs in its own thread and waits for for stdout
        // of the chezy_lidar process. Note that the handleLine method
        // operates relative to LidarServer.
        @Override
        public void run() 
        {
            while (isRunning())
            {
                try 
                {
                    if (mBufferedReader.ready())
                    {
                        String line = mBufferedReader.readLine();
                        if (line == null) // EOF
                        { 
                            throw new EOFException("End of chezy-lidar process InputStream");
                        }
                        handleLine(line);
                    }
                } 
                catch (IOException e)
                {
                    e.printStackTrace();
                    if (isLidarConnected())
                    {
                        System.err.println("Lidar sensor disconnected");
                        stop();
                    }
                }
            }
        }
    }
}
