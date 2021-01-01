package org.firstinspires.ftc.teamcode.autonomousRR;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;
import org.firstinspires.ftc.teamcode.robot.drivetrain.mecanum.AutonomousMecanumMoverRR;


@Autonomous(name = "Mecannum: AbstractAutonomousOpModeRR", group = "Mecannum")
@Disabled
/*
  This is the baseclass for all autonomous op modes for Meccunum robot
 */
public abstract class AbstractAutonomousOpModeRR extends LinearOpMode {
    // Use gearheads robot hardware
    public GearheadsMecanumRobotRR robot;
    //The drive system
    public MecanumDriveRR mecanumDriveRR;

    // The autonomous driving software
    protected AutonomousMecanumMoverRR autonomousRobotMover;


    /**
     * Constructor
     */
    public AbstractAutonomousOpModeRR() {
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
        robot.initAutonomous(hardwareMap);

        mecanumDriveRR = new MecanumDriveRR(hardwareMap);
        autonomousRobotMover = new AutonomousMecanumMoverRR(robot, this, mecanumDriveRR);

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


