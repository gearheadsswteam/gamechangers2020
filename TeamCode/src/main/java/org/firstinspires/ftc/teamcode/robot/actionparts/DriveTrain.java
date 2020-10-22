package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveTrain {
    //DRIVING
    public DcMotor fl_motor;
    public DcMotor fr_motor;
    public DcMotor rl_motor;
    public DcMotor rr_motor;

    public DriveTrain(DcMotor fl_motor, DcMotor fr_motor, DcMotor rl_motor, DcMotor rr_motor) {
        this.fl_motor = fl_motor;
        this.fr_motor = fr_motor;
        this.rl_motor = rl_motor;
        this.rr_motor = rr_motor;
    }
}
