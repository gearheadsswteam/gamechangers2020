package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobot;

public class RingFlipperSystem {

    public Servo leftServo;
    public Servo rightServo;
    public static double  MIN_POSITION = 0.0, MAX_POSITION = 1, MIDDLE_POSITION = 0.5;
    public static double  INIT_POSITION = 0.0, GRAB_POSITION = 0.18, UNGRAB_POSITION = 0.0;

    GearheadsMecanumRobot myRobot;
    LinearOpMode curOpMode;

    public RingFlipperSystem(LinearOpMode opMode, Servo leftRobotServo,Servo rightRobotServo){
        curOpMode = opMode;
        leftServo = leftRobotServo;
        rightServo = rightRobotServo;
    }

    public void flipRings(){
        curOpMode.telemetry.addData("Grab = ", "");
        curOpMode.telemetry.update();
        operateServo(GRAB_POSITION);

    }
    public void unflipRings(){;
        curOpMode.telemetry.addData("unGrab = ", "");
        curOpMode.telemetry.update();
        operateServo(UNGRAB_POSITION);
    }

    public double operateServo(double gripPosition) {
        // open the gripper on X button if not already at most open position.
        leftServo.setPosition(Range.clip(gripPosition, RingFlipperSystem.MIN_POSITION, RingFlipperSystem.MAX_POSITION));
        rightServo.setPosition(Range.clip(gripPosition, RingFlipperSystem.MIN_POSITION, RingFlipperSystem.MAX_POSITION));
        printSkystoneServoState();
        return gripPosition;
    }

    public void printSkystoneServoState() {
        curOpMode.telemetry.addData("leftFoundationServo val = ", leftServo.getPosition());
        curOpMode.telemetry.addData("rightFoundationServo val = ", rightServo.getPosition());
        curOpMode.telemetry.update();
    }
}
