package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobot;

public class FoundationGrabber {

    public static double  MIN_POSITION = 0.0, MAX_POSITION = 1, MIDDLE_POSITION = 0.5;
    public static double  INIT_POSITION = 0.0, GRAB_POSITION = 0.18, UNGRAB_POSITION = 0.0;

    GearheadsMecanumRobot myRobot;
    LinearOpMode curOpMode;

    public FoundationGrabber(LinearOpMode opMode, GearheadsMecanumRobot robot){
        curOpMode = opMode;
        myRobot = robot;

    }

    public void grabFoundation(){
        curOpMode.telemetry.addData("Grab = ", "");
        curOpMode.telemetry.update();
        operateServo(GRAB_POSITION);

    }
    public void ungrabFoundation(){;
        curOpMode.telemetry.addData("unGrab = ", "");
        curOpMode.telemetry.update();
        operateServo(UNGRAB_POSITION);
    }

    public double operateServo(double gripPosition) {
        // open the gripper on X button if not already at most open position.
        myRobot.leftFoundationServo.setPosition(Range.clip(gripPosition, FoundationGrabber.MIN_POSITION, FoundationGrabber.MAX_POSITION));
        myRobot.rightFoundationServo.setPosition(Range.clip(gripPosition, FoundationGrabber.MIN_POSITION, FoundationGrabber.MAX_POSITION));
        printSkystoneServoState();
        return gripPosition;
    }

    public void printSkystoneServoState() {
        curOpMode.telemetry.addData("leftFoundationServo val = ", myRobot.leftFoundationServo.getPosition());
        curOpMode.telemetry.addData("rightFoundationServo val = ", myRobot.rightFoundationServo.getPosition());
        curOpMode.telemetry.update();
    }
}
