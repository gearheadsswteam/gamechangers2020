package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotPositionFinderFactory {
    private static GEarheadsOdometryPositionFinder positionFinder;

    /**
     * Constructor for GlobalCoordinatePosition Thread
     *
     * @param verticalEncoderLeft  left odometry encoder, facing the vertical direction
     * @param verticalEncoderRight right odometry encoder, facing the vertical direction
     * @param horizontalEncoder    horizontal odometry encoder, perpendicular to the other two odometry encoder wheels
     * @param threadSleepDelay     delay in milliseconds for the GlobalPositionUpdate thread (50-75 milliseconds is suggested)
     */
    public synchronized static GEarheadsOdometryPositionFinder getPositionFinder(DcMotor verticalEncoderLeft, DcMotor verticalEncoderRight, DcMotor horizontalEncoder, double COUNTS_PER_INCH, int threadSleepDelay) {
        if (positionFinder == null) {
            positionFinder = new GEarheadsOdometryPositionFinder(verticalEncoderLeft, verticalEncoderRight, horizontalEncoder, COUNTS_PER_INCH, threadSleepDelay);
        }
        return positionFinder;
    }
}
