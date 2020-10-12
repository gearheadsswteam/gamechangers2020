package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawServoMotor {
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational INITIAL_POSITION
    static final double MIN_POS     =  0.0;     // Minimum rotational INITIAL_POSITION

    // Define class members
    public static double INITIAL_POSITION = (MAX_POS - MIN_POS) / 2; // Start at halfway INITIAL_POSITION

    private LinearOpMode curOpMode = null;

    private Servo servo;  // Hardware Device Object




    public ClawServoMotor(Servo curServo, LinearOpMode opMode) {
        servo = curServo;
        curOpMode = opMode;
    }

    public void moveServoUsingTeleOp(){
        servo.setPosition (servo.getPosition () + curOpMode.gamepad1.left_stick_x / 20);
        curOpMode.telemetry.addData("Servo Angle : ", servo.getPosition ());
        curOpMode.telemetry.update ();
    }

    public void setToInitialPosition(){

    }

    public void grab(){
        servo.setPosition (servo.getPosition () + 0.1);
        curOpMode.telemetry.addData("Servo grab : ", servo.getPosition ());
        curOpMode.telemetry.update ();
    }

    public void release(){
        servo.setPosition (servo.getPosition () - 0.1);
        curOpMode.telemetry.addData("Servo release : ", servo.getPosition ());
        curOpMode.telemetry.update ();
    }

}
