package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobot;

/**
 * Class the represents the Ring flip system
 */
public class RingFlipperSystem {

    //Servos used by this system
    public Servo leftServo;
    public Servo rightServo;

    //Positions of the servos
    public static double  MIN_POSITION = 0.0, MAX_POSITION = 1, MIDDLE_POSITION = 0.5;
    public static double  INIT_POSITION = 0.0, PUSH_POSITION = 0.18, RESET_POSITION = 0.0;

    LinearOpMode curOpMode;

    /**
     * Constructor
     * @param opMode
     * @param leftRobotServo
     * @param rightRobotServo
     */
    public RingFlipperSystem(LinearOpMode opMode, Servo leftRobotServo,Servo rightRobotServo){
        curOpMode = opMode;
        leftServo = leftRobotServo;
        rightServo = rightRobotServo;
    }

    /**
     * initializes the system
     */
    public void initialize(){

    }

    /**
     * Push a ring to the shooter
     */
    public void pushRing(){
        operateServo(PUSH_POSITION);
        curOpMode.sleep(100);
        operateServo(RESET_POSITION);
    }

    /**
     * Operates the servos to push the rings
     * @param gripPosition
     * @return
     */
    private double operateServo(double gripPosition) {
        // open the gripper on X button if not already at most open position.
        leftServo.setPosition(Range.clip(gripPosition, RingFlipperSystem.MIN_POSITION, RingFlipperSystem.MAX_POSITION));
        rightServo.setPosition(Range.clip(gripPosition, RingFlipperSystem.MIN_POSITION, RingFlipperSystem.MAX_POSITION));
        printSkystoneServoState();
        return gripPosition;
    }

    private void printSkystoneServoState() {
        curOpMode.telemetry.addData("leftFoundationServo val = ", leftServo.getPosition());
        curOpMode.telemetry.addData("rightFoundationServo val = ", rightServo.getPosition());
        curOpMode.telemetry.update();
    }
}
