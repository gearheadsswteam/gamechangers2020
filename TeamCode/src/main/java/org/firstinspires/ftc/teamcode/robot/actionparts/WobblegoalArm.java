package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Class that represnts the wobble arm grab system
 */
public class WobblegoalArm {

    //The Servo to lift the arm up and down
    private Servo liftServo;

    //Servo used to grab the wobble arm
    private Servo grabServo;

    //Servo positions
    private static double WOBBLE_GRAB_POSTION = 0.5;
    private static double WOBBLE_UNGRAB_POSTION = 0;
    private static double WOBBLE_LIFT_POSTION = 0.5;
    private static double WOBBLE_SET_POSTION = 0;

    /**
     * Constructor
     * @param liftServo
     * @param grabServo
     */
    public WobblegoalArm(Servo liftServo, Servo grabServo) {
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
    public void grabWobbleGoal(){
        grabServo.setPosition(WOBBLE_GRAB_POSTION);
    }

    /**
     * Ungrabs the wobble goal
     */
    public void ungrabWobbleGoal(){
        grabServo.setPosition(WOBBLE_UNGRAB_POSTION);
    }

    /**
     * Lifts the wobble goal post
      */

    public void liftWobbleGoal(){
        liftServo.setPosition(WOBBLE_LIFT_POSTION);
    }

    /**
     * Sets the wobble goal post down
     */
    public void setWobbleGoal(){
        liftServo.setPosition(WOBBLE_SET_POSTION);
    }
}
