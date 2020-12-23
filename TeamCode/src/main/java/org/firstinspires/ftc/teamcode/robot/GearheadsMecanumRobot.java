package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.odometry.GEarheadsOdometryPositionFinder;
import org.firstinspires.ftc.teamcode.odometry.RobotPositionFinderFactory;
import org.firstinspires.ftc.teamcode.robot.actionparts.Intakesystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.PositionEncoders;
import org.firstinspires.ftc.teamcode.robot.actionparts.RingDetector;
import org.firstinspires.ftc.teamcode.robot.actionparts.RingFlipperSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.ShootingSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.WobblegoalArmLeft;
import org.firstinspires.ftc.teamcode.robot.actionparts.WobblegoalArmRight;


/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class GearheadsMecanumRobot {

    //Different action systems used by the Robot
    public Intakesystem intakesystem;
    public ShootingSystem shootingSystem;
    public RingFlipperSystem ringFlipperSystem;
    public WobblegoalArmRight wobblegoalArmRight;
    public WobblegoalArmLeft wobblegoalArmLeft;

    //Gyro
    public BNO055IMU imu;

    //drive train
    public DcMotor fl_motor;
    public DcMotor fr_motor;
    public DcMotor rl_motor;
    public DcMotor rr_motor;


    //Odometry encoder wheels
    public DcMotor verticalRight;
    public DcMotor verticalLeft;
    public DcMotor horizontal;

    public PositionEncoders positionEncoderCenter;
    public PositionEncoders positionEncoderLeft;
    public PositionEncoders positionEncoderRight;

    //Hardware map names for the encoder wheels. Again, these will change for each robot and need to be updated below
    String verticalLeftEncoderName = "rf", verticalRightEncoderName = "lf", horizontalEncoderName = "lb";

    public GEarheadsOdometryPositionFinder globalPositionUpdate;
    public RingDetector ringDetector;

    public static final double COUNTS_PER_MOTOR_REV = 723.24;    // eg: 723.24 was 1478
    public static final double WHEEL_DIAMETER_INCHES = 3.93;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private LinearOpMode curOpMode = null;   //current opmode

    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Constructor */
    public GearheadsMecanumRobot(LinearOpMode opMode) {
        this.curOpMode = opMode;
    }


    /**
     * Initializes the intake system
     */
    private void initIntakeSystem() {
        DcMotor intakeMotor = hwMap.get(DcMotor.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        intakesystem = new Intakesystem(intakeMotor);
        intakesystem.initialize();
    }


    private void initShootingSystem() {
        //GReen Shooter
        DcMotor shootingMotorRight = hwMap.get(DcMotor.class, "shootRight");
        shootingMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootingMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shootingMotorRight.setDirection(DcMotor.Direction.REVERSE);

        //Blue motor
        DcMotor shootingMotorLeft = hwMap.get(DcMotor.class, "shootLeft");
        shootingMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootingMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shootingMotorLeft.setDirection(DcMotor.Direction.REVERSE);

        shootingSystem = new ShootingSystem(shootingMotorRight, shootingMotorLeft);
        shootingSystem.initialize();
    }

    private void intPositionEncoders() {
        positionEncoderCenter = new PositionEncoders(shootingSystem.shootingMotorRight);
        positionEncoderLeft = new PositionEncoders(shootingSystem.shootingMotorLeft);
        positionEncoderRight = null; // Need to add this on port 1
    }


    /**
     * Initializes the ring flip system
     */

    private void initRingFlipSystem() {
        Servo flipperServo = hwMap.get(Servo.class, "flipper");
        flipperServo.setDirection(Servo.Direction.FORWARD);

        ringFlipperSystem = new RingFlipperSystem(curOpMode, flipperServo);
        ringFlipperSystem.initialize();
    }

    /**
     * Initializes the Wobble goal arm system
     */

    private void initWobbleArmSystem() {
        Servo armRightServo = hwMap.get(Servo.class, "armRight");
        armRightServo.setDirection(Servo.Direction.FORWARD);

        Servo clawRightServo = hwMap.get(Servo.class, "clawRight");
        clawRightServo.setDirection(Servo.Direction.FORWARD);

        wobblegoalArmRight = new WobblegoalArmRight(armRightServo, clawRightServo);
        wobblegoalArmRight.initialize();

        Servo armLeftServo = hwMap.get(Servo.class, "armLeft");
        armLeftServo.setDirection(Servo.Direction.FORWARD);

        Servo clawLeftServo = hwMap.get(Servo.class, "clawLeft");
        clawLeftServo.setDirection(Servo.Direction.FORWARD);

        wobblegoalArmLeft = new WobblegoalArmLeft(armLeftServo, clawLeftServo);
        wobblegoalArmLeft.initialize();
    }

    /**
     * Starts the Ring detector
     */
    private void initRingDetector() {
        ringDetector = new RingDetector(curOpMode, hwMap);
        ringDetector.initialize();
    }


    /**
     * Initializes the Gyro
     *
     * @param calibrate
     */
    private void initGyro(boolean calibrate) {
        imu = hwMap.get(BNO055IMU.class, "gyro");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);

        if (calibrate) {
            curOpMode.telemetry.addData("Mode", "calibrating...");
            curOpMode.telemetry.update();

            // make sure the imu gyro is calibrated before continuing.
            while (!curOpMode.isStopRequested() && !imu.isGyroCalibrated()) {
                curOpMode.sleep(10);
                curOpMode.idle();
            }
        }

        curOpMode.telemetry.addData("Mode", "waiting for start");
        curOpMode.telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        curOpMode.telemetry.update();
    }


    /**
     * Initializes the drive train
     */
    private void initDriveMotors() {
        /**
         * Josh = orig
         * 0 br = fl
         * 1 bl = fr
         * 2 fr = rl
         * 3 fl = rr
         */


        //DRIVING
        fl_motor = hwMap.dcMotor.get("motor_fl");
        fr_motor = hwMap.dcMotor.get("motor_fr");
        rl_motor = hwMap.dcMotor.get("motor_rl");
        rr_motor = hwMap.dcMotor.get("motor_rr");


        //This is based on how motors have been mounted
        fr_motor.setDirection(DcMotor.Direction.FORWARD);
        rr_motor.setDirection(DcMotor.Direction.FORWARD);
        fl_motor.setDirection(DcMotor.Direction.REVERSE);
        rl_motor.setDirection(DcMotor.Direction.REVERSE);

        fr_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initOdometryEncoders() {
        //Assign the hardware map to the odometry wheels
        verticalLeft = hwMap.dcMotor.get(verticalLeftEncoderName);
        verticalRight = hwMap.dcMotor.get(verticalRightEncoderName);
        horizontal = hwMap.dcMotor.get(horizontalEncoderName);

        //Reset the encoders
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
        Reverse the direction of the odometry wheels. THIS WILL CHANGE FOR EACH ROBOT. Adjust the direction (as needed) of each encoder wheel
        such that when the verticalLeft and verticalRight encoders spin forward, they return positive values, and when the
        horizontal encoder travels to the right, it returns positive value
        */
        verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set the mode of the odometry encoders to RUN_WITHOUT_ENCODER
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions\
        globalPositionUpdate = RobotPositionFinderFactory.getPositionFinder(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
    }


    /* Initialize standard Hardware interfaces */
    public void initAutonomous(HardwareMap ahwMap) {
        init(ahwMap);
        initGyro(true);
        //initOdometryEncoders();
        //intPositionEncoders();
        initRingDetector();
    }

    /* Initialize standard Hardware interfaces */
    public void initTeleOp(HardwareMap ahwMap) {
        init(ahwMap);
        initGyro(false);
    }

    private void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        initDriveMotors();
        initIntakeSystem();
        initShootingSystem();
        initRingFlipSystem();
        initWobbleArmSystem();
    }
}


