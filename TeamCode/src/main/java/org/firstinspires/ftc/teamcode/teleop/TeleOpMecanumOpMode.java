package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.robot.drivetrain.mecanum.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobot;
import org.firstinspires.ftc.teamcode.robot.actionparts.ClawControl;
import org.firstinspires.ftc.teamcode.robot.actionparts.FoundationGrabber;
import org.firstinspires.ftc.teamcode.robot.actionparts.SkystoneArmServo;


@TeleOp(name = "Mecanum TeleOp Mode", group = "Mecanum")
//@Disabled
public class TeleOpMecanumOpMode extends LinearOpMode {

    /* Declare OpMode members. */
    private GearheadsMecanumRobot robot;   // Use gearheads robot hardware

    private MecanumDrive mecanum;
    private BNO055IMU gyro;

    private double turn;
    private double forwardPower;
    private double sidePower;

    private SkystoneArmServo skystoneArmServo;
    private FoundationGrabber foundationGrabber;

    public TeleOpMecanumOpMode() {
        robot = new GearheadsMecanumRobot(this);
        foundationGrabber = new FoundationGrabber(this, robot);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Wait for the game to start (driver presses PLAY)
        //Need this as first step else we get 5 point penalty
        waitForStart();

        initOpMode();

        double gripPosition;
        gripPosition = ClawControl.MIDDLE_POSITION;        // set grip to middle open

        initializeClaw(ClawControl.MIN_POSITION);
        while (opModeIsActive()) {
            adjustForFOV();
            dampenSpeed();
            //Move The robot
            moveRobot();

            gripPosition = operateClaw(gripPosition);
            operateElevator();
            operateFoundationGrabber();
            operateSkyStoneGrabber();
        }
    }

    private void operateFoundationGrabber() {
        if (gamepad1.x) {
            foundationGrabber.grabFoundation();
        }

        if (gamepad1.y) {
            foundationGrabber.ungrabFoundation();
        }

    }

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
        skystoneArmServo = new SkystoneArmServo(this, robot.skystoneGrabServo);
        initializeClaw(ClawControl.MIN_POSITION);
    }

    private void adjustForFOV() {
        double angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

        double tempForwardPower = gamepad1.left_stick_y;
        double tempSidePower = gamepad1.left_stick_x;

        sidePower = tempForwardPower * Math.cos(angle) + tempSidePower * Math.sin(angle);
        forwardPower = -tempForwardPower * Math.sin(angle) + tempSidePower * Math.cos(angle);
        turn = gamepad1.right_stick_x;
    }

    private void dampenSpeed() {
        float speedDamper = gamepad1.right_trigger;

        if (speedDamper == 1) {
            speedDamper = (float) 0.8;
        }

        forwardPower = forwardPower * (1 - speedDamper);
        sidePower = sidePower * (1 - speedDamper);
        turn = turn * (1 - speedDamper);
    }

    private double dampenElevatorSpeed(double elevatorActualSpeed) {
        float speedDamper = gamepad2.right_trigger;

        if (speedDamper == 1) {
            speedDamper = (float) 0.8;
        }

        return elevatorActualSpeed * (1 - speedDamper);
    }


    private void operateSkyStoneGrabber() {
        if (gamepad2.a) {
            skystoneArmServo.grabSkyStone();
        }
        if (gamepad2.b) {
            skystoneArmServo.dropSkyStone();
        }
    }

    private void noAdjustForFOV() {
        turn = gamepad1.right_stick_x;
        forwardPower = gamepad1.left_stick_x;
        sidePower = gamepad1.left_stick_y;
    }

    private void moveRobot() {
        //Joystick Movement
        mecanum.move(forwardPower, sidePower, turn);


        //Push data
        pushTelemetry();
    }

    private void operateElevator() {
        //Elevator
        this.telemetry.addData("Limit up ", robot.limitSwitchUp.getState());
        this.telemetry.addData("Limit down ", robot.limitSwitchDown.getState());
        this.telemetry.update();

        double elevatorDrive;
        elevatorDrive = 0.8 * gamepad2.left_stick_y;

        elevatorDrive = dampenElevatorSpeed(elevatorDrive);

        if (elevatorDrive > 0) {
            if (robot.limitSwitchUp.getState()) {
                robot.elevatorDrive.setPower(elevatorDrive);
            } else {
                robot.elevatorDrive.setPower(0);
                printLimitSwitchState();
            }
        }

        if (elevatorDrive < 0) {
            elevatorDrive = gamepad2.left_stick_y;
            if (robot.limitSwitchDown.getState()) {
                robot.elevatorDrive.setPower(elevatorDrive);
            } else {
                robot.elevatorDrive.setPower(0);
                printLimitSwitchState();
            }
        }

        if (elevatorDrive == 0) {
            robot.elevatorDrive.setPower(elevatorDrive);
        }
    }

    private double initializeClaw(double gripPosition) {
        robot.clawServoRight.setPosition(Range.clip(gripPosition, ClawControl.MIN_POSITION, ClawControl.MAX_POSITION));
        robot.clawServoLeft.setPosition(Range.clip(gripPosition, ClawControl.MIN_POSITION, ClawControl.MAX_POSITION));
        printClawState(gripPosition);
        return gripPosition;
    }


    private double operateClaw(double gripPosition) {
        // open the gripper on X button if not already at most open position.
        if (gamepad2.y && gripPosition < ClawControl.MAX_POSITION)
            gripPosition = gripPosition + .05;

        // close the gripper on Y button if not already at the closed position.
        if (gamepad2.x && gripPosition > ClawControl.MAX_CLOSE)
            gripPosition = gripPosition - .05;

        robot.clawServoRight.setPosition(Range.clip(gripPosition, ClawControl.MIN_POSITION, ClawControl.MAX_POSITION));
        robot.clawServoLeft.setPosition(Range.clip(gripPosition, ClawControl.MIN_POSITION, ClawControl.MAX_POSITION));
        printClawState(gripPosition);
        return gripPosition;
    }

    private void printLimitSwitchState() {
        this.telemetry.addData("Limit up pressed ", !robot.limitSwitchUp.getState());
        this.telemetry.addData("Limit down pressed ", !robot.limitSwitchDown.getState());
        this.telemetry.addData("Elevator stopped ", "done");
        this.telemetry.update();
    }

    private void printClawState(double gripPosition) {
        this.telemetry.addData("Claw val = ", gripPosition);
        this.telemetry.update();
    }

    private void printNav(double angle) {
        this.telemetry.addData("gamepad1.right_stick_x = ", gamepad1.right_stick_x);
        this.telemetry.addData("angle = ", angle);
        this.telemetry.update();
    }

    public void pushTelemetry() {
        telemetry.addData("Gyro Heading", gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
        telemetry.addData("Drive Data", mecanum.getDataString());
        telemetry.update();
    }
}