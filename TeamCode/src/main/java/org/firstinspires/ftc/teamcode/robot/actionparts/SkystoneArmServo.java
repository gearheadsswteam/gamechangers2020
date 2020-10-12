package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class SkystoneArmServo {

    public static double  MIN_POSITION = 0.0, MAX_POSITION = 1, MIDDLE_POSITION = 0.5;


    private Servo skystoneGrabServo;
    LinearOpMode curOpMode;

    public SkystoneArmServo(LinearOpMode opMode, Servo skystoneServo){
        curOpMode = opMode;
        skystoneGrabServo = skystoneServo;
    }

    public double initializeSkyStoneServo() {
        double gripPosition = 0.65;
        skystoneGrabServo.setPosition(Range.clip(gripPosition, SkystoneArmServo.MIN_POSITION, SkystoneArmServo.MAX_POSITION));
        printSkystoneServoState(gripPosition);
        return gripPosition;
    }


    public double grabSkyStone() {
        double gripPosition = 0.7;
        skystoneGrabServo.setPosition(Range.clip(gripPosition, SkystoneArmServo.MIN_POSITION, SkystoneArmServo.MAX_POSITION));
        printSkystoneServoState(gripPosition);
        return gripPosition;
    }

    public double dropSkyStone() {
        double gripPosition = 0.3;
        skystoneGrabServo.setPosition(Range.clip(gripPosition, SkystoneArmServo.MIN_POSITION, SkystoneArmServo.MAX_POSITION));
        printSkystoneServoState(gripPosition);
        return gripPosition;
    }

    public double grabConstructionMat() {
        double gripPosition = 0.95;
        skystoneGrabServo.setPosition(Range.clip(gripPosition, SkystoneArmServo.MIN_POSITION, SkystoneArmServo.MAX_POSITION));
        printSkystoneServoState(gripPosition);
        return gripPosition;
    }

    public double dropConstructionMat() {
        double gripPosition = 0.45;
        skystoneGrabServo.setPosition(Range.clip(gripPosition, SkystoneArmServo.MIN_POSITION, SkystoneArmServo.MAX_POSITION));
        printSkystoneServoState(gripPosition);
        return gripPosition;
    }


    public void printSkystoneServoState(double gripPosition) {
        curOpMode.telemetry.addData("Skystone Servo val = ", gripPosition);
        curOpMode.telemetry.update();
    }

    public void printSkystoneServoState() {
        curOpMode.telemetry.addData("Skystone Servo val = ", skystoneGrabServo.getPosition());
        curOpMode.telemetry.update();
    }

}
