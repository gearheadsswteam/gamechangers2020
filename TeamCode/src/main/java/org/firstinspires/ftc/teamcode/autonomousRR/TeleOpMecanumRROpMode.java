package org.firstinspires.ftc.teamcode.autonomousRR;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;
import org.firstinspires.ftc.teamcode.robot.actionparts.Intakesystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.RingFlipperSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.ShootingSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.WobblegoalArmLeft;
import org.firstinspires.ftc.teamcode.robot.actionparts.WobblegoalArmRight;
import org.firstinspires.ftc.teamcode.robot.drivetrain.mecanum.AutonomousMecanumMoverRR;
import org.firstinspires.ftc.teamcode.robot.drivetrain.mecanum.MecanumDrive;


@TeleOp(name = "Mecanum TeleOp Mode RR", group = "Mecanum")
//@Disabled
public class TeleOpMecanumRROpMode extends LinearOpMode {

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

    // The autonomous driving software
    protected AutonomousMecanumMoverRR autonomousRobotMover;

    Pose2d initPos;

    Trajectory shootingPositionTraj;



    /**
     * Constructor
     */
    public TeleOpMecanumRROpMode() {
        robot = new GearheadsMecanumRobotRR(this);
    }

    /**
     * Run this as the first thing in the autonomous opmode
     */

    protected void initOpModeBeforeStart() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.initTeleopRR(hardwareMap);

        mecanumDriveRR = new MecanumDriveRR(hardwareMap);
        autonomousRobotMover = new AutonomousMecanumMoverRR(robot, this, mecanumDriveRR);

        //Reading position of the robot that was set in the autonomous mode.
        mecanumDriveRR.setPoseEstimate(PoseStorage.currentPose);
        initPos = PoseStorage.currentPose;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
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
            goToShootingPosition();
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

        initOpModeBeforeStart();//Odomotery starts here

        gyro = robot.imu;
        intakesystem = robot.intakesystem;
        shootingSystem = robot.shootingSystem;
        ringFlipperSystem = robot.ringFlipperSystem;
        wobblegoalArmRight = robot.wobblegoalArmRight;
        wobblegoalArmLeft = robot.wobblegoalArmLeft;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //DRIVING
        DcMotor fl_motor = mecanumDriveRR.leftFront;
        DcMotor fr_motor = mecanumDriveRR.rightFront;
        DcMotor rl_motor = mecanumDriveRR.leftRear;
        DcMotor rr_motor = mecanumDriveRR.rightRear;

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
        if (gamepad2.x) {
            ringFlipperSystem.pushRing();
        }
    }

    /**
     * Operate the ring Flipping system
     */
    private void goToShootingPosition() {
        //shootingPositionTraj = mecanumDriveRR.trajectoryBuilder(mecanumDriveRR.getPoseEstimate()).splineTo(new Vector2d(4, -24), 0)
        shootingPositionTraj = mecanumDriveRR.trajectoryBuilder(mecanumDriveRR.getPoseEstimate()).splineTo(new Vector2d(0.5, -14), 0)
                .build();
        if (gamepad1.a) {
            mecanumDriveRR.followTrajectory(shootingPositionTraj);
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
    }

    private double SHOOTER_SPEED_1 = 1 / 5;
    private double SHOOTER_SPEED_2 = 2 / 5;
    private double SHOOTER_SPEED_3 = 3 / 5;

    private int shooterSpeedState = 1;

    /**
     * Operate shooter
     */
    private void operateShooter() {
        if (gamepad2.right_trigger > 0.3) {
            shootingSystem.operateShooterMotor(0.2);
            telemetry.addData("Shooter speed = ", 0.2);
            telemetry.update();

//            if(gamepad2.left_bumper) {
//                shooterSpeedState++;
//
//                if(shooterSpeedState > 3){
//                    shooterSpeedState = 1;
//                }
//            }
//
//
//            if(shooterSpeedState == 1) {
//                //inc speed
//                shootingSystem.operateShooterMotor(SHOOTER_SPEED_1);
//                telemetry.addData("Shooter speed = ", SHOOTER_SPEED_1);
//                telemetry.update();
//
//            }if(shooterSpeedState == 2) {
//                //inc speed
//                shootingSystem.operateShooterMotor(SHOOTER_SPEED_2);
//                telemetry.addData("Shooter speed = ", SHOOTER_SPEED_2);
//                telemetry.update();
//            }if(shooterSpeedState == 3) {
//                //inc speed
//                shootingSystem.operateShooterMotor(SHOOTER_SPEED_3);
//                telemetry.addData("Shooter speed = ", SHOOTER_SPEED_3);
//                telemetry.update();
//            }

        } else {
            shootingSystem.stopShooterMotor();
            shooterSpeedState = 0;
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