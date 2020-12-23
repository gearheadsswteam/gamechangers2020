package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Class that represnts the wobble arm grab system
 */
public class WobblegoalArmLeft implements WobbleGoalArm{

    //The Servo to lift the arm up and down
    private Servo liftServo;

    //Servo used to grab the wobble arm
    private Servo grabServo;

    //Servo Positions
    private final double UP_POSITION = 0.450;
    private final double DOWN_POSITION = 0.600;
    private final double CLOSED_POSITION = 0.390;
    private final double OPEN_POSITION = 0;


    /**
     * Constructor
     * @param liftServo
     * @param grabServo
     */
    public WobblegoalArmLeft(Servo liftServo, Servo grabServo) {
        this.liftServo = liftServo;
        this.grabServo = grabServo;
    }

    /**
     * Initialize the Wobble arm system
     */
    public void initialize(){

    }

    /**
     * Grabs the wobble goal
     */
    public void grabWobbleGoal() {
        grabServo.setPosition(CLOSED_POSITION);
    }

    /**
     * Ungrabs the wobble goal
     */
    public void ungrabWobbleGoal() {
        grabServo.setPosition(OPEN_POSITION);
    }

    /**
     * Lifts the wobble goal post
     */

    public void liftWobbleGoal() {
        liftServo.setPosition(UP_POSITION);
    }

    /**
     * Sets the wobble goal post down
     */
    public void setWobbleGoal() {
        liftServo.setPosition(DOWN_POSITION);
    }

    /**
     * The position to move the arm to
     *
     * @param leftArmState
     */
    public void operateArm(int leftArmState) {
        switch (leftArmState) {
            case 0:
                liftServo.setPosition(UP_POSITION);
                grabServo.setPosition(OPEN_POSITION);
                break;
            case 1:
                liftServo.setPosition(DOWN_POSITION);
                grabServo.setPosition(OPEN_POSITION);
                break;
            case 2:
                liftServo.setPosition(DOWN_POSITION);
                grabServo.setPosition(CLOSED_POSITION);
                break;
            case 3:
                liftServo.setPosition(UP_POSITION);
                grabServo.setPosition(CLOSED_POSITION);
                break;
        }
    }
}
