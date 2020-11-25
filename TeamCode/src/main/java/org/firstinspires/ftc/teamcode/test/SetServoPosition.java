package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Set Servo Position: Scan Servo", group = "Concept")
@Disabled
public class SetServoPosition extends LinearOpMode {

    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int CYCLE_MS = 500;     // period of each cycle
    static final double RESET_POSITION = 0;     // Maximum rotational position
    static final double PUSH_POSITION = 0.75;     // Minimum rotational position

    // Define class members
    Servo servo;
    //double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    double position;

    @Override
    public void runOpMode() {

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        servo = hardwareMap.get(Servo.class, "rightFlipper");
        servo.setDirection(Servo.Direction.REVERSE);//For the "rightFlipper"
        position = 0.5;
        servo.setPosition(position);
        sleep(300);

        // Wait for the start button
        telemetry.addData(">", "Press Gamepad 1 X to scan Servo");
        telemetry.update();
        waitForStart();

        // Scan servo till stop pressed.
        while (opModeIsActive()) {
            position = servo.getPosition();
            // Display the current value
            telemetry.addData("Servo Initial Position", "%5.2f", position);
            telemetry.update();
            sleep(500);

            if (gamepad1.right_bumper) {
                // Set the servo to the new position and pause;
                servo.setPosition(RESET_POSITION);
                sleep(50);
            }
            if (gamepad1.left_bumper) {
                // Set the servo to the new position and pause;
                servo.setPosition(PUSH_POSITION);
                sleep(50);
            }


            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", servo.getPosition());
            telemetry.update();

            sleep(CYCLE_MS);
            idle();
        }
    }
}
