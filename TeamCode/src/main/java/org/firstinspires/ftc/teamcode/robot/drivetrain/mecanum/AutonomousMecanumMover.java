package org.firstinspires.ftc.teamcode.robot.drivetrain.mecanum;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.odometry.OdometryUtil;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobot;
import org.firstinspires.ftc.teamcode.robot.actionparts.SkystoneDetector;
import org.firstinspires.ftc.teamcode.robot.utils.Movement;
import org.firstinspires.ftc.teamcode.robot.utils.Position;
import org.firstinspires.ftc.teamcode.vuforia.Robot_Navigation;

public class AutonomousMecanumMover {
    //This will need tuning if you see the robot oscillating heavily
    //This is the KP coeefient for Gyro correction to be used during straffing
    public static final double STRAFFING_KP_CORRECTION_FACTOR = 0.1;

    public static final double KP_CORRECTION_FACTOR = 0.1;

    // Set PID proportional value to produce non-zero correction value when robot veers off
    // straight line. P value controls how sensitive the correction is.
    //private PIDController pidDrive = new PIDController(KP_CORRECTION_FACTOR, 0, 0);
    private PIDController pidDrive = new PIDController(.1, 0.09, 0);

    //This is the conversion factor : STRAFE DIST = FORWARD DIST * MECUNNUM_STRAFE_TO_FORWARD_DIST_SCALE
    public static final double MECUNNUM_STRAFE_TO_FORWARD_DIST_SCALE = 1.025;
    /* Declare OpMode members. */
    public MecanumDrive mecanum;
    private BNO055IMU gyro;
    public GearheadsMecanumRobot robot = null;   // Use a Gearbot's hardware
    private LinearOpMode curOpMode = null;   //current opmode
    private ElapsedTime runtime = null;//used for measuring time
    private final double FORWARD_SPEED = 0.2;
    public BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle;

    boolean isEncoderOn = false; //flag which starts and stops encoder count
    double encoderCount = 0; //counting encoder for parking computation.


    /**
     * Constructor
     *
     * @param gearheadsRobot robot to use
     * @param myOpMode       opmode that is executing
     */
    public AutonomousMecanumMover(GearheadsMecanumRobot gearheadsRobot, LinearOpMode myOpMode, MecanumDrive mecanumDrive) {
        robot = gearheadsRobot;
        curOpMode = myOpMode;
        runtime = new ElapsedTime();
        imu = gearheadsRobot.imu;
        mecanum = mecanumDrive;
    }


    /**
     * Moves the robot forward for specific time
     *
     * @param time time to move the robot in milliseconds
     */
    public void moveRobotForwardByTime(double time) {
        moveRobotForwardOrBackwardByTime(time, -FORWARD_SPEED);
    }

    /**
     * Moves the robot forward for specific time
     *
     * @param time  time to move the robot in milliseconds
     * @param speed speed to move in range (0 to 1)
     */
    public void moveRobotForwardByTime(double time, double speed) {
        moveRobotForwardOrBackwardByTime(time, -speed);
    }

    /**
     * Moves the robot backward for specific time
     *
     * @param time time to move the robot in milliseconds
     */
    public void moveRobotBackwardByTime(double time) {
        moveRobotForwardOrBackwardByTime(time, FORWARD_SPEED);
    }

    /**
     * Moves the robot backward for specific time
     *
     * @param time  time to move the robot in milliseconds
     * @param speed speed to move in range (0 to 1)
     */
    public void moveRobotBackwardByTime(double time, double speed) {
        moveRobotForwardOrBackwardByTime(time, speed);
    }

    /**
     * Moves robot right using position encoder method.
      * @param speed speed to move with
     * @param distanceT0Move horizontal distance to move
     */
    public void moveRobotRightUsingPositionEncoders(double speed, double distanceT0Move) {
        moveRobotRightOrLeftUsingPositionEncoders(speed, distanceT0Move);
    }

    /**
     * Moves robot left using position encoder method.
     * @param speed speed to move with
     * @param distanceT0Move horizontal distance to move
     */
    public void moveRobotLeftUsingPositionEncoders(double speed, double distanceT0Move) {
        moveRobotRightOrLeftUsingPositionEncoders(-speed, distanceT0Move);
    }


    /**
     * Moves robot left or right using position encoder method.
     * @param speed speed to move with
     * @param distanceT0Move horizontal distance to move
     */
    public void moveRobotRightOrLeftUsingPositionEncoders(double speed, double distanceT0Move) {
        double x = speed;
        double y = 0;

        robot.positionEncoderCenter.resetEncoderValue();
        mecanum.move(x, y, 0);

        runtime.reset();
        resetAngle();
        pidDrive.initPIDController();

        while (curOpMode.opModeIsActive() && (Math.abs(this.robot.positionEncoderCenter.distanceTravelledInInches()) < Math.abs(distanceT0Move))) {
            double correction = checkDirection();
//            // Compensate for gyro angle.
            Vector2d input = new Vector2d(x, y);
            input.rotate(-correction);
            //TODO Rishi: Add the move call and test
            //Need to add mecanum.move(x, y, 0);
            //mecanum.move(x, y, -correction*0.1/6);
        }
        // turn the motors off.
        mecanum.stopRobot();
        rotateToZeroHeading(0.1);
        pushTelemetry();
        resetAngle();
        robot.positionEncoderCenter.resetEncoderValue();
    }

    /**
     * Moves the robot from one (X,y) position to (X1, Y1) position
     *
     * @param speed          speed at which the movement is to be done
     * @param currPosition   starting position
     * @param targetPosition target position
     */
    public void moveRobotToPosition(double speed, Position currPosition, Position
            targetPosition) {
        Movement movementNeeded = new Movement(currPosition, targetPosition, speed);
        double neededX = movementNeeded.getNormalizedX();
        double neededY = movementNeeded.getNormalizedY();

        double x = neededX;
        double y = -neededY;

        mecanum.move(x, y, 0);

        runtime.reset();
        resetAngle();
        pidDrive.initPIDController();


        //TODO: add OdometryUtil implementation
        while (curOpMode.opModeIsActive() && (!OdometryUtil.hasReached(targetPosition))) {
            double correction = checkDirection();
            // Compensate for gyro angle.
            Vector2d input = new Vector2d(x, y);
            input.rotate(-correction);

            mecanum.move(input.x, input.y, 0);
        }
        // turn the motors off.
        mecanum.stopRobot();

        pushTelemetry();
        resetAngle();
    }

    /**
     * Moves robot forward or backward based on positive or negative speed for fixed time
     *
     * @param time  time to move the robot in milliseconds
     * @param speed speed to move in range (-1 to 1), -ve for forward, +ve for backward
     */
    private void moveRobotForwardOrBackwardByTime(double time, double speed) {
        double x = 0;
        double y = speed;

        mecanum.move(x, y, 0);

        runtime.reset();
        resetAngle();
        pidDrive.initPIDController();

        while (curOpMode.opModeIsActive() && (runtime.milliseconds() < time)) {
            double correction = checkDirection();
            // Compensate for gyro angle.
            Vector2d input = new Vector2d(x, y);
            input.rotate(-correction);

            mecanum.move(input.x, input.y, 0);
        }
        // turn the motors off.
        mecanum.stopRobot();

        pushTelemetry();
        resetAngle();
    }

    /**
     * Push Telemetry data
     */
    private void pushTelemetry() {
        curOpMode.telemetry.addData("Run Time", "Leg 1: %2.5f S Elapsed ", runtime.seconds());
        curOpMode.telemetry.addData("Gyro Heading ", getAngle());
        curOpMode.telemetry.update();
    }

    /**
     * Moves the robot right for specific time
     *
     * @param time time to move the robot in milliseconds
     */
    public void moveRobotRightByTime(double time) {
        moveRobotLeftOrRightByTimeConditionally(time, FORWARD_SPEED, null);
    }

    /**
     * Moves the robot right for specific time
     *
     * @param time  time to move the robot in milliseconds
     * @param speed speed to move in range (0 to 1)
     */
    public void moveRobotRightByTime(double time, double speed) {
        moveRobotLeftOrRightByTimeConditionally(time, speed, null);
    }

    /**
     * Moves the robot right for specific time
     *
     * @param time time to move the robot in milliseconds
     * @param nav  condition to be met
     */
    public void moveRobotRightByTime(double time, Robot_Navigation nav) {
        moveRobotLeftOrRightByTimeConditionally(time, FORWARD_SPEED, nav);
    }


    /**
     * Moves the robot right for specific time
     *
     * @param time  time to move the robot in milliseconds
     * @param speed speed to move in range (0 to 1)
     * @param nav   condition to be met
     */
    public void moveRobotRightByTime(double time, double speed, Robot_Navigation nav) {
        moveRobotLeftOrRightByTimeConditionally(time, speed, nav);
    }


    /**
     * Moves the robot left for specific time
     *
     * @param time time to move the robot in milliseconds
     */
    public void moveRobotLeftByTime(double time) {
        moveRobotLeftOrRightByTimeConditionally(time, -FORWARD_SPEED, null);
    }

    /**
     * Moves the robot left for specific time
     *
     * @param time  time to move the robot in milliseconds
     * @param speed speed to move in range (0 to 1)
     */
    public void moveRobotLeftByTime(double time, double speed) {
        moveRobotLeftOrRightByTimeConditionally(time, -speed, null);
    }


    /**
     * Moves the robot left for specific time
     *
     * @param time  time to move the robot in milliseconds
     * @param speed speed to move in range (0 to 1)
     * @param nav   condition to be met
     */
    public void moveRobotLeftByTime(double time, double speed, Robot_Navigation nav) {
        moveRobotLeftOrRightByTimeConditionally(time, -speed, nav);
    }


    /**
     * Moves the robot left or right for specific time or until a condition is met.
     *
     * @param time  time to move the robot in milliseconds
     * @param speed speed to move in range (-1 to 1), -ve for left & +ve for right
     * @param nav   condition to be met
     */

    private boolean moveRobotLeftOrRightByTimeConditionally(double time,
                                                            double speed, Robot_Navigation nav) {
        double x = speed;
        double y = 0;
        boolean conditionMet = false;

        mecanum.move(x, y, 0);

        runtime.reset();
        resetAngle();

        while (curOpMode.opModeIsActive() && (runtime.milliseconds() < time)) {

            if (nav != null && nav.targetsAreVisible()) {
                // Stop all motion;
                stopRobotInEncoderMode();
                conditionMet = true;
                return conditionMet;
            }

//            double correction = checkDirection();
//            // Compensate for gyro angle.
//            Vector2d input = new Vector2d(x, y);
//            input.rotate(-correction);
//
//            mecanum.move(input.x, input.y, 0);
        }
        // turn the motors off.
        mecanum.stopRobot();
        this.rotateToZeroHeading(0.1);
        pushTelemetry();

        resetAngle();
        return conditionMet;
    }


    /**
     * Rotate right
     *
     * @param degreesToRotate degrees to rotate
     */
    public void rotateRight(int degreesToRotate) {
        rotateRight(degreesToRotate, 0.3);
    }

    /**
     * Rotate left
     *
     * @param degreesToRotate degrees to rotate
     */

    public void rotateLeft(int degreesToRotate) {
        rotateLeft(degreesToRotate, 0.3);
    }

    /**
     * Rotate right
     *
     * @param degreesToRotate degrees to rotate
     * @param power           speed at which to rotate
     */

    public void rotateRight(int degreesToRotate, double power) {
        resetAngle();
        degreesToRotate = Math.abs(degreesToRotate);

        double angleRotated = Math.abs(rotate(Math.abs((int) (degreesToRotate * 0.8)), power));
        curOpMode.sleep(50);
        angleRotated = Math.abs(angleRotated);


        resetAngle();//Very important

        angleRotated += Math.abs(rotate(Math.abs((int) (degreesToRotate - angleRotated)), 0.1));

        curOpMode.telemetry.addData("Needed rotation ", degreesToRotate);
        curOpMode.telemetry.addData("Achieved rotation ", angleRotated);
        curOpMode.telemetry.update();
        resetAngle();
    }


    /**
     * Rotate left
     *
     * @param degreesToRotate degrees to rotate
     * @param power           speed at which to rotate
     */
    public void rotateLeft(int degreesToRotate, double power) {
        resetAngle();
        degreesToRotate = Math.abs(degreesToRotate);

        double angleRotated = Math.abs(rotate(-Math.abs((int) (degreesToRotate * 0.8)), power));
        curOpMode.sleep(50);
        angleRotated = Math.abs(angleRotated);

        resetAngle();//Very important

        angleRotated += Math.abs(rotate(-Math.abs((int) (degreesToRotate - angleRotated)), 0.1));
        curOpMode.telemetry.addData("Needed rotation ", degreesToRotate);
        curOpMode.telemetry.addData("Achieved rotation ", angleRotated);
        curOpMode.telemetry.update();
        resetAngle();
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, +ve is left -ve is right
     * @param power   speed of movement (Range 0 to 1)
     */
    private double rotate(int degrees, double power) {
        double turn;

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            turn = power;
        } else if (degrees > 0) {   // turn left.
            turn = -power;
        } else return getAngle();

        // set z value to rotate.
        mecanum.move(0, 0, turn);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (curOpMode.opModeIsActive() && getAngle() == 0) {
                printOrientation();
            }


            while (curOpMode.opModeIsActive() && Math.abs(getAngle()) < Math.abs(degrees)) {
                printOrientation();

            }
        } else    // left turn.

            while (curOpMode.opModeIsActive() && Math.abs(getAngle()) < Math.abs(degrees)) {
                printOrientation();
            }

        // turn the motors off.
        mecanum.stopRobot();

        // wait for rotation to stop.
        curOpMode.sleep(1000);

        return getAngle();
    }

    public void printOrientation() {
        curOpMode.telemetry.addData("Angle ", getAngle());
        curOpMode.telemetry.update();
    }

    /**
     * Rotates till zero heading is achieved
     *
     * @param power speed of rotation. Range (0 to 1)
     */
    private void rotateToZeroHeading(double power) {
        double turn;
        double curHeading = getAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (curHeading < 0) {   // turn left as it is facing right.
            turn = power;
        } else if (curHeading > 0) {   // turn right as it is facing left.
            turn = -power;
        } else return;

        // set power to rotate.
        mecanum.move(0, 0, turn);

        boolean zeroHeadingReached = false;

        // rotate until turn is completed.
        while (curOpMode.opModeIsActive() && !zeroHeadingReached) {
            double heading = getAngle();
            //Stop when heading is within +-0.5 degress, if you set absolute zero the while loop may not complete always
            zeroHeadingReached = (heading < 0.5 && heading > -0.5);
        }

        // turn the motors off.
        mecanum.stopRobot();

        // wait for rotation to stop.
        curOpMode.sleep(1000);
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double checkDirection() {
        return checkDirection(KP_CORRECTION_FACTOR);
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection(double neededGain) {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        gain = neededGain;

        angle = getAngle();

        curOpMode.telemetry.addData("II Gyro ", angle);

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = pidDrive.performPID(angle);
        ;        // reverse sign of angle for correction.

        //correction = correction * gain;

        return correction;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirectionStrafe(double neededGain) {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        gain = neededGain;

        angle = getAngle();

        curOpMode.telemetry.addData("II Gyro ", angle);

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }


    /**
     * Move robot forward
     *
     * @param speed        speed of movement. Range (0 to 1)
     * @param inchesToMove number of inches to move
     */

    public void moveRobotForwardDistance(double speed, double inchesToMove) {
        moveRobotForwardDistanceConditionally(speed, inchesToMove, null);
    }


    /**
     * Move robot forward certain distance , stop if condition is met before the distance is complete
     *
     * @param speed        speed of movement. Range (0 to 1)
     * @param inchesToMove number of inches to move
     */

    public boolean moveRobotForwardDistanceConditionally(double speed,
                                                         double inchesToMove, SkystoneDetector nav) {
        // Send telemetry message to signify robot waiting;
        curOpMode.telemetry.addData("Status", "Resetting Encoders");    //
        curOpMode.telemetry.update();

        resetAngle();
        initEncoderMode();

        // Send telemetry message to indicate successful Encoder reset
        curOpMode.telemetry.addData("Path0", "Starting at %7d :%7d",
                mecanum.fl_motor.getCurrentPosition(),
                mecanum.rr_motor.getCurrentPosition());
        curOpMode.telemetry.update();

        boolean conditionMet = encoderForwardDrive(speed, inchesToMove, inchesToMove, 180, nav);

        curOpMode.telemetry.addData("Path", "Complete");
        curOpMode.telemetry.addData("Heading ", getAngle());
        curOpMode.telemetry.update();
        resetAngle();

        return conditionMet;
    }

    /**
     * Move robot right side ways
     *
     * @param speed        speed of movement. Range (0 to 1)
     * @param inchesToMove number of inches to move
     */
    public void moveRobotRightDistance(double speed, double inchesToMove) {
        moveRobotLeftOrRightDistanceConditionally(speed, inchesToMove, null);
    }


    /**
     * Move robot right side ways
     *
     * @param speed        speed of movement. Range (0 to 1)
     * @param inchesToMove number of inches to move
     */
    public boolean moveRobotRightDistance(double speed, double inchesToMove, SkystoneDetector
            nav) {
        return moveRobotLeftOrRightDistanceConditionally(speed, inchesToMove, nav);
    }


    /**
     * Move robot left side ways
     *
     * @param speed        speed of movement.Range (0 to 1)
     * @param inchesToMove number of inches to move
     */

    public void moveRobotLeftDistance(double speed, double inchesToMove) {
        moveRobotLeftOrRightDistanceConditionally(-speed, inchesToMove, null);
    }

    /**
     * Move robot sideways certain distance , stop if condition is met before the distance is complete
     *
     * @param speed        speed of movement (-1 to 1) +ve is right, -ve is left
     * @param inchesToMove number of inches to move
     * @param nav          condition to be met
     */

    /**
     * Move robot right side ways
     *
     * @param speed        speed of movement. Range (0 to 1)
     * @param inchesToMove number of inches to move
     */
    public boolean moveRobotLeftDistance(double speed, double inchesToMove, SkystoneDetector
            nav) {
        return moveRobotLeftOrRightDistanceConditionally(-speed, inchesToMove, nav);
    }


    /**
     * move the robot in strafe
     * @param speed        speed at which to move
     * @param inchesToMove straffing distance needed
     * @param nav          condition at which to stop
     * @return true if condition met else false
     */
    private boolean moveRobotLeftOrRightDistanceConditionally(double speed,
                                                              double inchesToMove, SkystoneDetector nav) {
        // Send telemetry message to signify robot waiting;
        curOpMode.telemetry.addData("Status", "Resetting Encoders");    //
        curOpMode.telemetry.update();

        resetAngle();
        initEncoderMode();

        // Send telemetry message to indicate successful Encoder reset
        curOpMode.telemetry.addData("Path0", "Starting at %7d :%7d",
                mecanum.fl_motor.getCurrentPosition(),
                mecanum.rr_motor.getCurrentPosition());
        curOpMode.telemetry.update();

        //We take straffing distance and convert to necessary forward distance to achieve that.
        inchesToMove = inchesToMove * MECUNNUM_STRAFE_TO_FORWARD_DIST_SCALE;

        boolean conditionMet = encoderLeftOrRightDrive(speed, inchesToMove, inchesToMove, 180, nav);

        curOpMode.telemetry.addData("Path", "Complete");
        curOpMode.telemetry.addData("Heading ", getAngle());
        curOpMode.telemetry.update();

        //If the gyro heading is not zero after stop then compensate for error.
        rotateToZeroHeading(0.2);
        resetAngle();

        return conditionMet;
    }

    /**
     * Used in basic autonomous parking
     */
    public void parkFromLeft() {
        //TODO needs to be implemented
    }

    /**
     * Used in basic autonomous parking
     */

    public void parkFromRight() {
        //TODO needs to be implemented
    }

    /**
     * Stops the robot and gets into USING Encoder mode
     */
    public void stopRobotInEncoderMode() {
        mecanum.stopRobot();

        curOpMode.telemetry.addData("> ", " Robot Power 0");
        curOpMode.telemetry.update();
        mecanum.setRunUsingEncoderMode();
    }

    /**
     * Initializes Encoder mode
     */
    private void initEncoderMode() {
        mecanum.initEncoderMode();
    }

    public void setRunUsingEncoderMode() {
        mecanum.setRunUsingEncoderMode();
    }


    /**
     * Moves robot backwards a set distance at set speed
     *
     * @param speed        speed to move
     * @param inchesToMove distance inches to move
     */
    public void moveRobotBackwardDistance(double speed, double inchesToMove) {
        moveRobotBackwardDistanceConditionally(speed, inchesToMove, null);
    }

    /**
     * Moves robot backwards a set distance, stops if condition is met before that distance
     *
     * @param speed        speed to move
     * @param inchesToMove distance inches to move
     * @param nav          condition to meet while moving
     * @return true if condition is met else false
     */
    public boolean moveRobotBackwardDistanceConditionally(double speed,
                                                          double inchesToMove, Robot_Navigation nav) {
        // Send telemetry message to signify robot waiting;
        curOpMode.telemetry.addData("Status", "Resetting Encoders");    //
        curOpMode.telemetry.update();

        resetAngle();
        initEncoderMode();

        // Send telemetry message to indicate successful Encoder reset
        curOpMode.telemetry.addData("Path0", "Starting at %7d :%7d",
                mecanum.fl_motor.getCurrentPosition(),
                mecanum.rr_motor.getCurrentPosition());
        curOpMode.telemetry.update();

        boolean conditionMet = encoderBackwardDrive(speed, -inchesToMove, -inchesToMove, 180, nav);

        curOpMode.telemetry.addData("Path", "Complete");
        curOpMode.telemetry.addData("Heading ", getAngle());
        curOpMode.telemetry.update();
        resetAngle();

        return conditionMet;
    }

    /**
     * Moves robot forward based on encoder counts
     * Move will stop if any of three conditions occur:
     * *  1) Move gets to the desired position
     * *  2) Move runs out of time
     * *  3) Driver stops the opmode running.
     * *  4) Search condition is met
     *
     * @param speed       speed to move
     * @param leftInches  distance in inches to move on left wheels
     * @param rightInches distance in inches to move on right wheels
     * @param timeoutS    timeout at which movement has to stop
     * @param nav         condition to test during movement
     * @return true if condition is met else false
     */
    private boolean encoderForwardDrive(double speed,
                                        double leftInches, double rightInches,
                                        double timeoutS, SkystoneDetector nav) {
        int newFLTarget;
        int newRRTarget;
        int newFRTarget;
        int newRLTarget;

        int initialPosition = mecanum.fl_motor.getCurrentPosition();
        int distToCover = 0;

        double x = 0;
        double targetMaxSpeed = -speed;


        boolean conditionMet = false;

        // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFLTarget = mecanum.fl_motor.getCurrentPosition() + (int) (leftInches * GearheadsMecanumRobot.COUNTS_PER_INCH);
            newRRTarget = mecanum.rr_motor.getCurrentPosition() + (int) (rightInches * GearheadsMecanumRobot.COUNTS_PER_INCH);
            newFRTarget = mecanum.fr_motor.getCurrentPosition() + (int) (leftInches * GearheadsMecanumRobot.COUNTS_PER_INCH);
            newRLTarget = mecanum.rl_motor.getCurrentPosition() + (int) (rightInches * GearheadsMecanumRobot.COUNTS_PER_INCH);

            distToCover = newFLTarget - mecanum.fl_motor.getCurrentPosition();

            mecanum.fl_motor.setTargetPosition(newFLTarget);
            mecanum.rr_motor.setTargetPosition(newRRTarget);
            mecanum.fr_motor.setTargetPosition(newFRTarget);
            mecanum.rl_motor.setTargetPosition(newRLTarget);

            // Turn On RUN_TO_POSITION
            mecanum.fl_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mecanum.rr_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mecanum.fr_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mecanum.rl_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            pidDrive.initPIDController();

            double initSpeedToRun = 0.05;
            mecanum.move(x, initSpeedToRun, 0);


            /**
             *  |       ____________
             *  |      /            \
             *  |    /               \
             *  |  /                  \
             *  |/_____________________\________
             *
             */

            double speedToRun = initSpeedToRun;
            double SPEED_STEPS = 10;
            double speedIncrements = targetMaxSpeed / SPEED_STEPS;
            int accelStep = 1;
            double ACCEL_SLOPE_PERCENT = 0.1;
            double DECCEL_SLOPE_PERCENT = 0.15;

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (curOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (mecanum.fl_motor.isBusy() && mecanum.rr_motor.isBusy() && mecanum.fr_motor.isBusy() && mecanum.rl_motor.isBusy())) {


                int distCovered = mecanum.fl_motor.getCurrentPosition() - initialPosition;

                curOpMode.telemetry.addData("distCovered ", distCovered);

                if (distCovered < distToCover * ACCEL_SLOPE_PERCENT) {
                    speedToRun = Math.min(Math.abs(speedToRun) + speedIncrements * accelStep, Math.abs(targetMaxSpeed));
                    speedToRun = -speedToRun;
                    accelStep++;
                    curOpMode.telemetry.addData("++ Speed => ", speedToRun);
                }

                //If in the initial 20% dist accelerate
                if (distCovered > distToCover * ACCEL_SLOPE_PERCENT && distCovered < (distToCover * (1 - DECCEL_SLOPE_PERCENT))) {
                    speedToRun = targetMaxSpeed;
                    accelStep = 1;
                    curOpMode.telemetry.addData("== Speed => ", speedToRun);
                }

                //If in the initial 20% dist accelerate
                if (distCovered > (distToCover * (1 - DECCEL_SLOPE_PERCENT))) {
                    //speedToRun = Math.max(Math.abs(targetMaxSpeed) - speedIncrements*accelStep, 0.2);
                    speedToRun = Math.max(Math.abs(targetMaxSpeed) - Math.abs(speedIncrements * accelStep), Math.abs(0.2));
                    speedToRun = -speedToRun;
                    accelStep++;
                    curOpMode.telemetry.addData("-- speed => ", speedToRun);
                }

                if (nav != null && nav.targetsAreVisible()) {
                    // Stop all motion;
                    stopRobotInEncoderMode();
                    if (isEncoderOn) {
                        encoderCount = encoderCount + mecanum.fl_motor.getCurrentPosition();
                    }
                    conditionMet = true;
                    return conditionMet;
                }

                curOpMode.telemetry.addData("Speed to Run => ", speedToRun);
                double correction = checkDirection();
                // Compensate for gyro angle.
                Vector2d input = new Vector2d(x, speedToRun);
                input.rotate(-correction);

                mecanum.move(input.x, input.y, 0);

                // Display it for the driver.
                curOpMode.telemetry.addData("Path1", "Running to %7d :%7d %7d :%7d ", newFLTarget, newFRTarget, newRLTarget, newRRTarget);
                curOpMode.telemetry.addData("Path2", "Running at %7d :%7d %7d :%7d",
                        mecanum.fl_motor.getCurrentPosition(),
                        mecanum.fr_motor.getCurrentPosition(), mecanum.rl_motor.getCurrentPosition(), mecanum.rr_motor.getCurrentPosition());
                curOpMode.telemetry.update();
            }

            if (isEncoderOn) {
                encoderCount = encoderCount + mecanum.fl_motor.getCurrentPosition();
            }
            stopRobotInEncoderMode();
            //  sleep(250);   // optional pause after each move
        }
        return conditionMet;
    }


    /**
     * Moves robot left or right based on encoders
     * Move will stop if any of three conditions occur:
     * *  1) Move gets to the desired position
     * *  2) Move runs out of time
     * *  3) Driver stops the opmode running.
     * *  4) Search condition is met
     *
     * @param speed       speed to move
     * @param leftInches  distance in inches to move on left wheels
     * @param rightInches distance in inches to move on right wheels
     * @param timeoutS    timeout at which movement has to stop
     * @param nav         condition to test during movement
     * @return true if condition is met else false
     */
    private boolean encoderLeftOrRightDrive(double speed,
                                            double leftInches, double rightInches,
                                            double timeoutS, SkystoneDetector nav) {
        int newFLTarget;
        int newRRTarget;
        int newFRTarget;
        int newRLTarget;

        double x = speed;
        double y = 0;


        boolean conditionMet = false;

        // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {

            if (speed > 0) {
                // Determine new target position, and pass to motor controller
                newFLTarget = mecanum.fl_motor.getCurrentPosition() + (int) (leftInches * GearheadsMecanumRobot.COUNTS_PER_INCH);
                newRRTarget = mecanum.rr_motor.getCurrentPosition() + (int) (rightInches * GearheadsMecanumRobot.COUNTS_PER_INCH);
                newFRTarget = mecanum.fr_motor.getCurrentPosition() - (int) (leftInches * GearheadsMecanumRobot.COUNTS_PER_INCH);
                newRLTarget = mecanum.rl_motor.getCurrentPosition() - (int) (rightInches * GearheadsMecanumRobot.COUNTS_PER_INCH);
            } else {
                // Determine new target position, and pass to motor controller
                newFLTarget = mecanum.fl_motor.getCurrentPosition() - (int) (leftInches * GearheadsMecanumRobot.COUNTS_PER_INCH);
                newRRTarget = mecanum.rr_motor.getCurrentPosition() - (int) (rightInches * GearheadsMecanumRobot.COUNTS_PER_INCH);
                newFRTarget = mecanum.fr_motor.getCurrentPosition() + (int) (leftInches * GearheadsMecanumRobot.COUNTS_PER_INCH);
                newRLTarget = mecanum.rl_motor.getCurrentPosition() + (int) (rightInches * GearheadsMecanumRobot.COUNTS_PER_INCH);
            }
            ;

            mecanum.fl_motor.setTargetPosition(newFLTarget);
            mecanum.rr_motor.setTargetPosition(newRRTarget);
            mecanum.fr_motor.setTargetPosition(newFRTarget);
            mecanum.rl_motor.setTargetPosition(newRLTarget);

            // Turn On RUN_TO_POSITION
            mecanum.fl_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mecanum.rr_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mecanum.fr_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mecanum.rl_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            mecanum.move(x, y, 0);


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (curOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (mecanum.fl_motor.isBusy() && mecanum.rr_motor.isBusy() && mecanum.fr_motor.isBusy() && mecanum.rl_motor.isBusy())) {

                if (nav != null && nav.targetsAreVisible()) {
                    // Stop all motion;
                    stopRobotInEncoderMode();
                    conditionMet = true;
                    return conditionMet;
                }
                //The logic for gyro compensation needs to be different....don't use this for now
//                double correction = checkDirection();
//                // Compensate for gyro angle.
//                Vector2d input = new Vector2d(x, y);
//                input.rotate(-correction);
//
//                mecanum.move(input.x, input.y, 0);

                //Added on 5/25/2020
                makeGyroCorrectionsDuringStrafing(x, y);

                // Display it for the driver.
                curOpMode.telemetry.addData("Path1", "Running to %7d :%7d %7d :%7d ", newFLTarget, newFRTarget, newRLTarget, newRRTarget);
                curOpMode.telemetry.addData("Path2", "Running at %7d :%7d %7d :%7d",
                        mecanum.fl_motor.getCurrentPosition(),
                        mecanum.fr_motor.getCurrentPosition(), mecanum.rl_motor.getCurrentPosition(), mecanum.rr_motor.getCurrentPosition());
                curOpMode.telemetry.update();
            }

            stopRobotInEncoderMode();
            //  sleep(250);   // optional pause after each move
        }
        return conditionMet;
    }

    /**
     * Makes corections for robot heading using Gyro during strafing
     * @param x
     * @param y
     */
    private void makeGyroCorrectionsDuringStrafing(double x, double y) {
        double correctionFactor = STRAFFING_KP_CORRECTION_FACTOR; //Tune this for your robot to prevent wobbling
        double directionError = checkDirectionStrafe(KP_CORRECTION_FACTOR);
        //TODO - we can use a PID controller class to improve the error correction if just proportional methos does not work
        double correctionApply = -correctionFactor * directionError;
        mecanum.move(x, y, correctionApply);
        curOpMode.sleep(200);
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     *  4) Search condition is met
     */

    /**
     * Moves robot backward based on encoder counts
     * Move will stop if any of three conditions occur:
     * *  1) Move gets to the desired position
     * *  2) Move runs out of time
     * *  3) Driver stops the opmode running.
     *
     * @param speed       speed to move (0 to 1)
     * @param leftInches  distance in inches to move on left wheels
     * @param rightInches distance in inches to move on right wheels
     * @param timeoutS    timeout at which movement has to stop
     * @param nav         condition to test during movement
     * @return true if condition is met else false
     */
    private boolean encoderBackwardDrive(double speed,
                                         double leftInches, double rightInches,
                                         double timeoutS, Robot_Navigation nav) {
        int newFLTarget;
        int newRRTarget;
        int newFRTarget;
        int newRLTarget;

        int initialPosition = mecanum.fl_motor.getCurrentPosition();
        int distToCover = 0;

        double x = 0;
        double targetMaxSpeed = speed;

        boolean conditionMet = false;

        // Ensure that the opmode is still active
        if (curOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFLTarget = mecanum.fl_motor.getCurrentPosition() + (int) (leftInches * GearheadsMecanumRobot.COUNTS_PER_INCH);
            newRRTarget = mecanum.rr_motor.getCurrentPosition() + (int) (rightInches * GearheadsMecanumRobot.COUNTS_PER_INCH);
            newFRTarget = mecanum.fr_motor.getCurrentPosition() + (int) (leftInches * GearheadsMecanumRobot.COUNTS_PER_INCH);
            newRLTarget = mecanum.rl_motor.getCurrentPosition() + (int) (rightInches * GearheadsMecanumRobot.COUNTS_PER_INCH);

            distToCover = Math.abs(newFLTarget - mecanum.fl_motor.getCurrentPosition());

            mecanum.fl_motor.setTargetPosition(newFLTarget);
            mecanum.rr_motor.setTargetPosition(newRRTarget);
            mecanum.fr_motor.setTargetPosition(newFRTarget);
            mecanum.rl_motor.setTargetPosition(newRLTarget);

            // Turn On RUN_TO_POSITION
            mecanum.fl_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mecanum.rr_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mecanum.fr_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mecanum.rl_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            pidDrive.initPIDController();

            double initSpeedToRun = 0.05;
            mecanum.move(x, initSpeedToRun, 0);


            /**
             *  |       ____________
             *  |      /            \
             *  |    /               \
             *  |  /                  \
             *  |/_____________________\________
             *
             */

            double speedToRun = initSpeedToRun;
            double SPEED_STEPS = 10;
            double speedIncrements = targetMaxSpeed / SPEED_STEPS;
            int accelStep = 1;
            double ACCEL_SLOPE_PERCENT = 0.1;
            double DECCEL_SLOPE_PERCENT = 0.15;


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (curOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (mecanum.fl_motor.isBusy() && mecanum.rr_motor.isBusy() && mecanum.fr_motor.isBusy() && mecanum.rl_motor.isBusy())) {
                int distCovered = Math.abs(mecanum.fl_motor.getCurrentPosition() - initialPosition);

                curOpMode.telemetry.addData("distCovered ", distCovered);

                if (distCovered < distToCover * ACCEL_SLOPE_PERCENT) {
                    speedToRun = Math.min(Math.abs(speedToRun) + speedIncrements * accelStep, Math.abs(targetMaxSpeed));
                    accelStep++;
                    curOpMode.telemetry.addData("++ Speed => ", speedToRun);
                }

                //If in the initial 20% dist accelerate
                if (distCovered > distToCover * ACCEL_SLOPE_PERCENT && distCovered < (distToCover * (1 - DECCEL_SLOPE_PERCENT))) {
                    speedToRun = targetMaxSpeed;
                    accelStep = 1;
                    curOpMode.telemetry.addData("== Speed => ", speedToRun);
                }

                //If in the initial 20% dist accelerate
                if (distCovered > (distToCover * (1 - DECCEL_SLOPE_PERCENT))) {
                    //speedToRun = Math.max(Math.abs(targetMaxSpeed) - speedIncrements*accelStep, 0.2);
                    speedToRun = Math.max(Math.abs(targetMaxSpeed) - Math.abs(speedIncrements * accelStep), Math.abs(0.2));
                    accelStep++;
                    curOpMode.telemetry.addData("-- speed => ", speedToRun);
                }

                if (nav != null && nav.targetsAreVisible()) {
                    // Stop all motion;
                    stopRobotInEncoderMode();
                    conditionMet = true;
                    return conditionMet;
                }
                curOpMode.telemetry.addData("Speed to Run => ", speedToRun);

                double correction = checkDirection();
                // Compensate for gyro angle.
                Vector2d input = new Vector2d(x, speedToRun);
                input.rotate(-correction);

                mecanum.move(input.x, input.y, 0);

                // Display it for the driver.
                curOpMode.telemetry.addData("Path1", "Running to %7d :%7d %7d :%7d ", newFLTarget, newFRTarget, newRLTarget, newRRTarget);
                curOpMode.telemetry.addData("Path2", "Running at %7d :%7d %7d :%7d",
                        mecanum.fl_motor.getCurrentPosition(),
                        mecanum.fr_motor.getCurrentPosition(), mecanum.rl_motor.getCurrentPosition(), mecanum.rr_motor.getCurrentPosition());
                curOpMode.telemetry.update();
            }

            stopRobotInEncoderMode();
            //  sleep(250);   // optional pause after each move
        }
        return conditionMet;
    }

    /**
     * Starts Encoder counting for parking computation
     */

    public void startEncoderCounting() {
        isEncoderOn = true;
    }

    /**
     * Stops Encoder counting for parking computation
     */
    public void stopEncoderCounting() {
        isEncoderOn = false;
    }


    /**
     * resets  Encoder count used in parking computation
     */
    public void resetEncoderCounting() {
        encoderCount = 0;
    }


    /**
     * Get distance in inches for encoder count
     *
     * @return
     */
    public double getEncoderCountingForwardDistance() {
        return Math.abs(encoderCount / GearheadsMecanumRobot.COUNTS_PER_INCH);
    }
}
