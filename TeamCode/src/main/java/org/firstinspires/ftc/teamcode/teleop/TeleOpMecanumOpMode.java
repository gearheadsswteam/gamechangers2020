package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.robot.actionparts.Intakesystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.ShootingSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.WobblegoalArm;
import org.firstinspires.ftc.teamcode.robot.drivetrain.mecanum.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobot;
import org.firstinspires.ftc.teamcode.robot.actionparts.RingFlipperSystem;


@TeleOp(name = "Mecanum TeleOp Mode", group = "Mecanum")
//@Disabled
public class TeleOpMecanumOpMode extends LinearOpMode {

    /* Declare OpMode members. */
    private GearheadsMecanumRobot robot;   // Use gearheads robot hardware

    //Different action systems used by the Robot
    private Intakesystem intakesystem;
    private ShootingSystem shootingSystem;
    private RingFlipperSystem ringFlipperSystem;
    private WobblegoalArm wobblegoalArm;

    private MecanumDrive mecanum;
    private BNO055IMU gyro;

    private double turn;
    private double forwardPower;
    private double sidePower;

    /**
     * Constructor
     */
    public TeleOpMecanumOpMode() {
        robot = new GearheadsMecanumRobot(this);
        intakesystem = robot.intakesystem;
        shootingSystem = robot.shootingSystem;
        ringFlipperSystem = robot.ringFlipperSystem;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Wait for the game to start (driver presses PLAY)
        //Need this as first step else we get 5 point penalty
        waitForStart();

        initOpMode();

        while (opModeIsActive()) {
            adjustForFOV();
            dampenSpeed();
            //Move The robot
            moveRobot();

            //Operate Ring functions
            operateIntake();
            operateRingFlipSystem();
            operateShooter();
            operateWobblegoalArmSystem();
        }
    }

    /**
     * Initialize the opmode
     */

    private void initOpMode() {
        // Wait for the game to start (driver presses PLAY)
        //Need this as first step else we get 5 point penalty
        waitForStart();

        telemetry.addData("Status", "Started");
        telemetry.update();

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.initTeleOp(hardwareMap);

        gyro = robot.imu;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //DRIVING
        DcMotor fl_motor = robot.fl_motor;
        DcMotor fr_motor = robot.fr_motor;
        DcMotor rl_motor = robot.rl_motor;
        DcMotor rr_motor = robot.rr_motor;

        mecanum = new MecanumDrive(fl_motor, fr_motor, rl_motor, rr_motor, gyro);
    }

    /**
     * Adjust teleop driving with FOV mode
     */
    private void adjustForFOV() {
        double angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

        double tempForwardPower = gamepad1.left_stick_y;
        double tempSidePower = gamepad1.left_stick_x;

        sidePower = tempForwardPower * Math.cos(angle) + tempSidePower * Math.sin(angle);
        forwardPower = -tempForwardPower * Math.sin(angle) + tempSidePower * Math.cos(angle);
        turn = gamepad1.right_stick_x;
    }

    /**
     * Dampen the Robot driving movements if right trigger is pressed
     */
    private void dampenSpeed() {
        float speedDamper = gamepad1.right_trigger;

        if (speedDamper == 1) {
            speedDamper = (float) 0.8;
        }

        forwardPower = forwardPower * (1 - speedDamper);
        sidePower = sidePower * (1 - speedDamper);
        turn = turn * (1 - speedDamper);
    }

    /**
     * Operate the wobble post arm system
     */
    private void operateWobblegoalArmSystem() {
        if (gamepad1.b) {
            wobblegoalArm.grabWobbleGoal();
        }

        if (gamepad1.a) {
            wobblegoalArm.ungrabWobbleGoal();
        }

        if (gamepad1.y) {
            wobblegoalArm.liftWobbleGoal();
        }

        if (gamepad1.x) {
            wobblegoalArm.liftWobbleGoal();
        }
    }

    /**
     * Operate the ring Flipping system
     */
    private void operateRingFlipSystem() {
        if (gamepad2.right_bumper) {
            ringFlipperSystem.pushRing();
        }
    }

    /**
     * Operate intake system
     */
    private void operateIntake() {
        if (gamepad2.y) {
            intakesystem.startInTake();
        }
        if (gamepad2.x) {
            intakesystem.stopInTake();
        }
    }

    /**
     * Operate shooter
     */
    private void operateShooter() {
        if (gamepad2.b) {
            shootingSystem.startShooterMotor();
        }
        if (gamepad2.a) {
            shootingSystem.stopShooterMotor();
        }
    }

    /**
     * Drive the robot
     */
    private void moveRobot() {
        //Joystick Movement
        mecanum.move(forwardPower, sidePower, turn);
        //Push data
        pushTelemetry();
    }


    private void pushTelemetry() {
        telemetry.addData("Gyro Heading", gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
        telemetry.addData("Drive Data", mecanum.getDataString());
        telemetry.update();
    }
}