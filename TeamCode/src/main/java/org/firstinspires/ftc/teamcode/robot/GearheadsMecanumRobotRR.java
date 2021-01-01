package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.odometry.GEarheadsOdometryPositionFinder;
import org.firstinspires.ftc.teamcode.odometry.RobotPositionFinderFactory;
import org.firstinspires.ftc.teamcode.robot.actionparts.Intakesystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.PositionEncoders;
import org.firstinspires.ftc.teamcode.robot.actionparts.RingDetector;
import org.firstinspires.ftc.teamcode.robot.actionparts.RingFlipperSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.ShootingSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.WobblegoalArmLeft;
import org.firstinspires.ftc.teamcode.robot.actionparts.WobblegoalArmRight;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;


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
public class GearheadsMecanumRobotRR {

    //Different action systems used by the Robot
    public Intakesystem intakesystem;
    public ShootingSystem shootingSystem;
    public RingFlipperSystem ringFlipperSystem;
    public WobblegoalArmRight wobblegoalArmRight;
    public WobblegoalArmLeft wobblegoalArmLeft;

    public RingDetector ringDetector;

    private LinearOpMode curOpMode = null;   //current opmode

    /* local OpMode members. */
    public HardwareMap hwMap = null;

    /* Constructor */
    public GearheadsMecanumRobotRR(LinearOpMode opMode) {
        this.curOpMode = opMode;
        hwMap = opMode.hardwareMap;
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
        shootingMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //shootingMotorRight.setDirection(DcMotor.Direction.REVERSE);

        //Blue motor
        DcMotor shootingMotorLeft = hwMap.get(DcMotor.class, "shootLeft");
        shootingMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //shootingMotorLeft.setDirection(DcMotor.Direction.REVERSE);

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
     * Starts the Ring detector
     */
    private void initRingDetector() {
        ringDetector = new RingDetector(curOpMode, hwMap);
        ringDetector.initialize();
    }


    /* Initialize standard Hardware interfaces */
    public void initAutonomous(HardwareMap ahwMap) {
        init(ahwMap);
        initRingDetector();
    }

    /* Initialize standard Hardware interfaces */
    public void initTeleOp(HardwareMap ahwMap) {
        init(ahwMap);
    }

    private void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        initIntakeSystem();
        initShootingSystem();
        initRingFlipSystem();
        initWobbleArmSystem();
    }
}


