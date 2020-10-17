package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Intakesystem {
    public DcMotor intakeMotor;

    public Intakesystem(DcMotor intakeMotor) {
        this.intakeMotor = intakeMotor;
    }

    public void startInTake() {
        intakeMotor.setPower(0.5);
    }

    public void stopInTake() {
        intakeMotor.setPower(0);
    }
}
