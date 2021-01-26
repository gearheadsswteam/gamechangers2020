package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Class the represents the Ring flip system
 */
public class RingFlipperSystem {

    //Servos used by this system
    private Servo flipperServo;

    //Positions of the servos
    private static double FLIPPER_RESET_POSITION = 0.50;
    private static double FLIPPER_PUSH_POSITION = 0.3;
    private static double MIN_POSITION = 0.25;
    private static double MAX_POSITION = 0.50;

    private LinearOpMode curOpMode;

    /**
     * Constructor
     * @param opMode
     * @param flipper
     *
     */
    public RingFlipperSystem(LinearOpMode opMode, Servo flipper){
        curOpMode = opMode;
        this.flipperServo = flipper;
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
        curOpMode.sleep(300);
        operateServoToResetPosition();
    }

    /**
     * Push a ring to the shooter
     */
    public void resetPosition(){
        operateServoToResetPosition();
    }

    /**
     * Push a ring to the shooter
     */
    public void pushPosition(){
        operateServoToPushPosition();
    }


    /**
     * Operates the servos to push the rings
     *
     * @return
     */
    private void operateServoToPushPosition() {
        // open the gripper on X button if not already at most open position.
        flipperServo.setPosition(Range.clip(FLIPPER_PUSH_POSITION, RingFlipperSystem.MIN_POSITION, RingFlipperSystem.MAX_POSITION));
    }

    /**
     * Operates the servos to push the rings
     *
     * @return
     */
    private void operateServoToResetPosition() {
        // open the gripper on X button if not already at most open position.
        flipperServo.setPosition(Range.clip(FLIPPER_RESET_POSITION, RingFlipperSystem.MIN_POSITION, RingFlipperSystem.MAX_POSITION));
    }


    private void printSkystoneServoState(String data) {
        curOpMode.telemetry.addData("=======" + data,"");
        curOpMode.telemetry.addData("flipperServo val = ", flipperServo.getPosition());
        curOpMode.telemetry.update();
        curOpMode.sleep(500);
    }
}
