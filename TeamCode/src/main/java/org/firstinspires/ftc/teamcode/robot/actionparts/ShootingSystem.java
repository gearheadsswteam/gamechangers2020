package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ShootingSystem {
    public DcMotor shootingMotor;

    public ShootingSystem(DcMotor shootingMotor) {
        this.shootingMotor = shootingMotor;
    }


    public void startShooterMotor(){
        shootingMotor.setPower(0.5);
    }

    public void stopShooterMotor(){
        shootingMotor.setPower(0);
    }
}
