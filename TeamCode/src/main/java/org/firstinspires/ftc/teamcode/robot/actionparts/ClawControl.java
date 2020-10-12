package org.firstinspires.ftc.teamcode.robot.actionparts;

import org.firstinspires.ftc.teamcode.robot.actionparts.ClawServoMotor;

public class ClawControl {

    public static double  MIN_POSITION = 0.0, MAX_POSITION = 0.3, MIDDLE_POSITION = 0.3, MAX_CLOSE = 0.0;

    /* Declare OpMode members. */

    private ClawServoMotor curServoMotor = null;

    public ClawControl(ClawServoMotor servoMotor) {
        curServoMotor = servoMotor;
    }

    public void setToInitialPosition() {
        curServoMotor.setToInitialPosition();
    }

    public void grab() {
        curServoMotor.grab();
    }

    public void release() {
        curServoMotor.release();
    }
}
