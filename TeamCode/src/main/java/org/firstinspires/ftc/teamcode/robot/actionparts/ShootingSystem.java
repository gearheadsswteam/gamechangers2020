package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Class that represents the ring shooting system
 */
public class ShootingSystem {
    //DC motor used by the Shooting system
    public DcMotor shootingMotorRight;
    public DcMotor shootingMotorLeft;

    /**
     * Constructor
     * @param shootingMotor
     */
    public ShootingSystem(DcMotor shootingMotor) {
        this.shootingMotorRight = shootingMotor;
    }

    /**
     * Constructor
     * @param shootingMotor
     */
    public ShootingSystem(DcMotor shootingMotorRight, DcMotor shootingMotorLeft) {
        this.shootingMotorRight = shootingMotorRight;
        this.shootingMotorLeft = shootingMotorLeft;
    }

    /**
     * Initiatlize the system
     */
    public void initialize(){

    }

    /**
     * Starts the ring shooting motor
     */
    public void startShooterMotor() {
        if(shootingMotorRight != null) {
            shootingMotorRight.setPower(1);
        }
        if(shootingMotorLeft != null) {
            shootingMotorLeft.setPower(1);
        }
    }

    public void operateShooterMotor(double power) {
        if(shootingMotorRight != null) {
            shootingMotorRight.setPower(power);
        }
        if(shootingMotorLeft != null) {
            shootingMotorLeft.setPower(power);
        }
    }


    /**
     * Stops the ring shooting motor
     */
    public void stopShooterMotor() {
        if(shootingMotorRight != null) {
            shootingMotorRight.setPower(0);
        }
        if(shootingMotorLeft != null) {
            shootingMotorLeft.setPower(0);
        }
    }
}
