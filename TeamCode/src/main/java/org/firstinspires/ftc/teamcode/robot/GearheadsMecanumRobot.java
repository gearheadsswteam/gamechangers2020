package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.actionparts.Intakesystem;
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
        shootingMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootingMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shootingMotorRight.setDirection(DcMotor.Direction.REVERSE);

        //Blue motor
        DcMotor shootingMotorLeft = hwMap.get(DcMotor.class, "shootLeft");
        shootingMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootingMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shootingMotorLeft.setDirection(DcMotor.Direction.REVERSE);

        shootingSystem = new ShootingSystem(shootingMotorRight, shootingMotorLeft);
        shootingSystem.initialize();
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


    /* Initialize standard Hardware interfaces */
    public void initAutonomous(HardwareMap ahwMap) {
        init(ahwMap);
        initGyro(true);
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


