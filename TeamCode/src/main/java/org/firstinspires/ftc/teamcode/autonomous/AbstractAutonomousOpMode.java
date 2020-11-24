package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobot;
import org.firstinspires.ftc.teamcode.robot.drivetrain.mecanum.AutonomousMecanumMover;
import org.firstinspires.ftc.teamcode.robot.drivetrain.mecanum.MecanumDrive;


@Autonomous(name = "Mecannum: AbstractAutonomousOpMode", group = "Mecannum")

/*
  This is the baseclass for all autonomous op modes for Meccunum robot
 */
public abstract class AbstractAutonomousOpMode extends LinearOpMode {
    // Use gearheads robot hardware
    GearheadsMecanumRobot robot;
    //The drive system
    private MecanumDrive mecanum;
    // Use gyro
    private BNO055IMU gyro;
    // The autonomous driving software
    protected AutonomousMecanumMover autonomousRobotMover;


    /**
     * Constructor
     */
    public AbstractAutonomousOpMode() {
        robot = new GearheadsMecanumRobot(this);
    }

    /**
     * Run this as the first thing in the autonomous opmode
     */

    private void initOpModeBeforeStart() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.initAutonomous(hardwareMap);
        gyro = robot.imu;

        while (!gyro.isGyroCalibrated()) {
            sleep(10);
            idle();
        }

        //DRIVING
        DcMotor fl_motor = robot.fl_motor;
        DcMotor fr_motor = robot.fr_motor;
        DcMotor rl_motor = robot.rl_motor;
        DcMotor rr_motor = robot.rr_motor;

        mecanum = new MecanumDrive(fl_motor, fr_motor, rl_motor, rr_motor, gyro);
        autonomousRobotMover = new AutonomousMecanumMover(robot, this, mecanum);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /**
     * Run this as the first thing in the autonomous opmode after initialization
     */
    protected abstract void initOpModeAfterStart();


    /**
     * This is where the actual opmode logic should be implemented
     */
    protected abstract void executeOpMode();

    @Override
    public void runOpMode() {
        initOpModeBeforeStart();
        waitForDriverAcknowledgement();
        initOpModeAfterStart();

        // Wait for the game to start (driver presses PLAY)
        while (opModeIsActive()) {
            executeOpMode();
            break;
        }

        closeOpMode();
    }

    /**
     * This method pause the op mode till driver presses start
     */
    private void waitForDriverAcknowledgement() {
        // Prompt User
        telemetry.addData(">", "Press start");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
    }

    /**
     * This method pause the op mode till driver presses start
     */
    private void closeOpMode() {
        // Prompt User
        telemetry.addData(">", "OpMode complete " + this.getClass().getSimpleName());
        telemetry.update();
    }
}


