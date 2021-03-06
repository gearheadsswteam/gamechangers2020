package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.autonomousRR.AbstractAutonomousOpModeRR;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;
import org.firstinspires.ftc.teamcode.robot.actionparts.Intakesystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.RingFlipperSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.ShootingSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.WobblegoalArmLeft;
import org.firstinspires.ftc.teamcode.robot.actionparts.WobblegoalArmRight;
import org.firstinspires.ftc.teamcode.robot.drivetrain.mecanum.MecanumDrive;


@TeleOp(name = "Mecanum TeleOp Mode", group = "Mecanum")
//@Disabled
public class TeleOpMecanumOpMode extends LinearOpMode {

    //Reference for Josh's code: https://docs.google.com/document/d/1nJ-Rro6GFyXt1vbN69c-Y5u8U8c_oHpr-_ET3eomAbA/edit

    /* Declare OpMode members. */
    private GearheadsMecanumRobotRR robot;   // Use gearheads robot hardware

    //Different action systems used by the Robot
    private Intakesystem intakesystem;
    private ShootingSystem shootingSystem;
    private RingFlipperSystem ringFlipperSystem;
    private WobblegoalArmRight wobblegoalArmRight;
    private WobblegoalArmLeft wobblegoalArmLeft;

    int leftArmState = 0;
    int rightArmState = 0;

    private boolean leftTriggerUp = true;
    private boolean rightTriggerUp = true;

    private MecanumDrive mecanum;
    private BNO055IMU gyro;

    private double turn;
    private double forwardPower;
    private double sidePower;
    public MecanumDriveRR mecanumDriveRR;


    /**
     * Constructor
     */
    public TeleOpMecanumOpMode() {
        robot = new GearheadsMecanumRobotRR(this);
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
            operatePowerShot();
            setZeroHeading();
        }
    }

    /**
     * Turns robot left or right for power shots
     */
    private void operatePowerShot() {
        if (gamepad1.x) {
            mecanumDriveRR.turn(Math.toRadians(8));
        } else if (gamepad1.b) {
            mecanumDriveRR.turn(Math.toRadians(-8));
        }
    }

    /**
     * Turns robot left or right for power shots
     */
    private void setZeroHeading() {
        if(gamepad1.y) {
            double angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle ;
            double headingCorrection = -angle;
            mecanumDriveRR.turn(headingCorrection);
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
        telemetry.addData("Team Type", PoseStorage.TEAM_TYPE);

        telemetry.update();

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.initTeleOp(hardwareMap);

        mecanumDriveRR = new MecanumDriveRR(hardwareMap);
        //Reading position of the robot that was set in the autonomous mode.
        mecanumDriveRR.setPoseEstimate(PoseStorage.currentPose);

        gyro = robot.imu;
        intakesystem = robot.intakesystem;
        shootingSystem = robot.shootingSystem;
        ringFlipperSystem = robot.ringFlipperSystem;
        wobblegoalArmRight = robot.wobblegoalArmRight;
        wobblegoalArmLeft = robot.wobblegoalArmLeft;

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
        //Angle adjustment during TeleOP based on how the autonomous ends. In Game changers the TeleOPs starts with Rbot at 90 degrees from FOV.
        double angleFromAutonomousLastRun = PoseStorage.gyroAngle;
        //double angleFromAutonomousLastRun = Math.PI/2;

        double angle = 0;
        if (PoseStorage.TEAM_TYPE.equals(AbstractAutonomousOpModeRR.RED_TEAM)) {
            angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + angleFromAutonomousLastRun - Math.PI / 2;
        } else if (PoseStorage.TEAM_TYPE.equals(AbstractAutonomousOpModeRR.BLUE_TEAM)) {
            angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + angleFromAutonomousLastRun + Math.PI / 2;
        }


        double tempForwardPower = gamepad1.left_stick_y;
        double tempSidePower = gamepad1.left_stick_x;

        sidePower = tempForwardPower * Math.cos(angle) + tempSidePower * Math.sin(angle);
        forwardPower = -tempForwardPower * Math.sin(angle) + tempSidePower * Math.cos(angle);
        turn = -gamepad1.right_stick_x;
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
        if (gamepad1.left_bumper && leftTriggerUp) {
            leftArmState = (leftArmState + 1) % 4;
            leftTriggerUp = false;
        } else if (!gamepad1.left_bumper) {
            leftTriggerUp = true;
        }
        wobblegoalArmLeft.operateArm(leftArmState);

        if (gamepad1.right_bumper && rightTriggerUp) {
            rightArmState = (rightArmState + 1) % 4;
            rightTriggerUp = false;
        } else if (!gamepad1.right_bumper) {
            rightTriggerUp = true;
        }

        wobblegoalArmRight.operateArm(rightArmState);
    }


    /**
     * Operate the ring Flipping system
     */
    private void operateRingFlipSystem() {
        //If hitting high goals
        if (gamepad2.right_trigger > 0.3 && gamepad2.x) {
            stopRobot();//Stops movement
            ringFlipperSystem.pushRing();
            sleep(500);
            ringFlipperSystem.pushRing();
            sleep(500);
            ringFlipperSystem.pushRing();
        } else //If hitting power goals
            if (gamepad2.left_trigger > 0.3 && gamepad2.x) {
                stopRobot();//Stops movement
                ringFlipperSystem.pushRing();
                sleep(500);
            }
    }


    /**
     * Operate intake system
     */
    private void operateIntake() {
        if (gamepad2.a) {
            intakesystem.startInTake();
        }
        if (gamepad2.b) {
            intakesystem.stopInTake();
        }
        if (gamepad2.y) {
            intakesystem.startReverseInTake();
        }

        if (gamepad2.dpad_up) {
            robot.intakeGaurdServo.setPosition(0);//up
        }
        if (gamepad2.dpad_down) {
            robot.intakeGaurdServo.setPosition(0.3);//down
        }
    }


    /**
     * Operate shooter
     */
    private void operateShooter() {
        if (gamepad2.right_trigger > 0.2) {
            shootingSystem.shootHighGoals();
            telemetry.update();
        } else if (gamepad2.left_trigger > 0.2) {
            shootingSystem.shootPowerShots();
            telemetry.update();
        } else {
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

    /**
     * Drive the robot
     */
    private void stopRobot() {
        //Joystick Movement
        mecanum.move(0, 0, 0);
        //Push data
        pushTelemetry();
    }


    private void pushTelemetry() {
        telemetry.addData("Gyro Heading", gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
        telemetry.addData("Drive Data", mecanum.getDataString());
        telemetry.update();
    }

}