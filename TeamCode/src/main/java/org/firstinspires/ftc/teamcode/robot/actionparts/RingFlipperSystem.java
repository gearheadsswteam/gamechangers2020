package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Class the represents the Ring flip system
 */
public class RingFlipperSystem {

    //Servos used by this system
    public Servo leftServo;
    public Servo rightServo;

    public static double  MIN_POSITION = 0.0, MAX_POSITION = 1, MIDDLE_POSITION = 0.5;

    //Positions of the servos
    public static double LEFT_RESET_POSITION = 0.2;
    public static double LEFT_PUSH_POSITION = 0.5;
    public static double RIGHT_RESET_POSITION = 0.2;
    public static double RIGHT_PUSH_POSITION = 0.5;


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
        operateServoToPushPosition();
        curOpMode.sleep(200);
        operateServoToResetPosition();
    }


    /**
     * Operates the servos to push the rings
     *
     * @return
     */
    private void operateServoToPushPosition() {
        // open the gripper on X button if not already at most open position.
        leftServo.setPosition(Range.clip(LEFT_PUSH_POSITION, RingFlipperSystem.MIN_POSITION, RingFlipperSystem.MAX_POSITION));
        rightServo.setPosition(Range.clip(RIGHT_PUSH_POSITION, RingFlipperSystem.MIN_POSITION, RingFlipperSystem.MAX_POSITION));
        printSkystoneServoState();
    }

    /**
     * Operates the servos to push the rings
     *
     * @return
     */
    private void operateServoToResetPosition() {
        // open the gripper on X button if not already at most open position.
        leftServo.setPosition(Range.clip(LEFT_RESET_POSITION, RingFlipperSystem.MIN_POSITION, RingFlipperSystem.MAX_POSITION));
        rightServo.setPosition(Range.clip(RIGHT_RESET_POSITION, RingFlipperSystem.MIN_POSITION, RingFlipperSystem.MAX_POSITION));
        printSkystoneServoState();
    }


    private void printSkystoneServoState() {
        curOpMode.telemetry.addData("leftFoundationServo val = ", leftServo.getPosition());
        curOpMode.telemetry.addData("rightFoundationServo val = ", rightServo.getPosition());
        curOpMode.telemetry.update();
    }
}
