package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class GearheadsMecanumRobot{

    public BNO055IMU imu;
    public Servo skystoneGrabServo;
    public Servo leftFoundationServo;
    public Servo rightFoundationServo;
    public Servo clawServoRight;
    public Servo clawServoLeft;
    public DcMotor fl_motor;
    public DcMotor fr_motor;
    public DcMotor rl_motor;
    public DcMotor rr_motor;
    public DigitalChannel limitSwitchUp;
    public DigitalChannel limitSwitchDown;
    public DcMotor elevatorDrive;
    public ColorSensor sensorColor;
    public DistanceSensor sensorDistance;



    public static final double     COUNTS_PER_MOTOR_REV    = 723.24 ;    // eg: 723.24 was 1478
    public static final double     WHEEL_DIAMETER_INCHES   = 3.93 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private LinearOpMode curOpMode = null;   //current opmode

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public GearheadsMecanumRobot(LinearOpMode opMode){
        this.curOpMode = opMode;

    }


    private void initServo(){
        clawServoRight = hwMap.get (Servo.class, "servoTestRight");
        clawServoRight.setDirection(Servo.Direction.FORWARD);

        clawServoLeft = hwMap.get (Servo.class, "servoTestLeft");
        clawServoRight.setDirection(Servo.Direction.REVERSE);

    }

    private void initFoundationServo(){
        leftFoundationServo = hwMap.get (Servo.class, "leftFoundationServo");
        //leftFoundationServo.setPosition(FoundationGrabber.INIT_POSITION);

        rightFoundationServo = hwMap.get (Servo.class, "rightFoundationServo");
        //rightFoundationServo.setPosition(FoundationGrabber.INIT_POSITION);
    }

    private void initSkyStoneServo(){
        skystoneGrabServo = hwMap.get (Servo.class, "skystoneServo");
        //skystoneGrabServo.setPosition(0.65);
    }

    private void initLimitSwitch(){
        limitSwitchUp = hwMap.get(DigitalChannel.class,"limitSwitchUp");
        limitSwitchUp.setMode(DigitalChannel.Mode.INPUT);

        limitSwitchDown = hwMap.get(DigitalChannel.class,"limitSwitchDown");
        limitSwitchDown.setMode(DigitalChannel.Mode.INPUT);
    }

    private void initElevator(){
        elevatorDrive = hwMap.get(DcMotor.class, "motorElevator");
        elevatorDrive.setPower(0);

        elevatorDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initColorSensor(){
        // get a reference to the color sensor.
        sensorColor = hwMap.get(ColorSensor.class, "colorSensor");

        // get a reference to the distance sensor that shares the same name.
        //sensorDistance = hwMap.get(DistanceSensor.class, "colorSensor");

    }


    private void initGyro(boolean calibrate){
        imu = hwMap.get(BNO055IMU.class, "testGyro");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);

        if(calibrate) {
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


    private void initDriveMotors(){
        //DRIVING
        fl_motor = hwMap.dcMotor.get("motor_fl");
        fr_motor = hwMap.dcMotor.get("motor_fr");
        rl_motor = hwMap.dcMotor.get("motor_rl");
        rr_motor = hwMap.dcMotor.get("motor_rr");


        //This is based on how motors have been mounted
        fr_motor.setDirection(DcMotor.Direction.REVERSE);
        rr_motor.setDirection(DcMotor.Direction.REVERSE);
        fl_motor.setDirection(DcMotor.Direction.FORWARD);
        rl_motor.setDirection(DcMotor.Direction.FORWARD);

        fr_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        initDriveMotors();
        initServo();
        initSkyStoneServo();
        initFoundationServo();
        initColorSensor();
        initLimitSwitch();
        initElevator();
        initGyro(true);
    }

    /* Initialize standard Hardware interfaces */
    public void initTeleOp(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        initDriveMotors();
        initServo();
        initSkyStoneServo();
        initFoundationServo();
        initColorSensor();
        initLimitSwitch();
        initElevator();
        initGyro(false);
    }
}


