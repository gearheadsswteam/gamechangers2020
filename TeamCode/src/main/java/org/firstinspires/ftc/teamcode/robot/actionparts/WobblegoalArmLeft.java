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

    //Servo positions
    private static double WOBBLE_GRAB_POSTION = 0.5;
    private static double WOBBLE_UNGRAB_POSTION = 0.7;

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
    public void grabWobbleGoal(){
        double position = grabServo.getPosition();
        position = position + 0.05;
        grabServo.setPosition(position);
    }

    /**
     * Ungrabs the wobble goal
     */
    public void ungrabWobbleGoal(){
        double position = grabServo.getPosition();
        position = position - 0.05;
        grabServo.setPosition(position);
    }

    /**
     * Lifts the wobble goal post
      */

    public void liftWobbleGoal(){
        double position = liftServo.getPosition();
        position = position + 0.05;
        liftServo.setPosition(position);
    }

    /**
     * Sets the wobble goal post down
     */
    public void setWobbleGoal(){
        double position = liftServo.getPosition();
        position = position - 0.05;
        liftServo.setPosition(position);
    }
}
