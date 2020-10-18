package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Class that represents the ring shooting system
 */
public class ShootingSystem {
    //DC motor used by the Shooting system
    public DcMotor shootingMotor;

    /**
     * Constructor
     * @param shootingMotor
     */
    public ShootingSystem(DcMotor shootingMotor) {
        this.shootingMotor = shootingMotor;
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
        shootingMotor.setPower(0.5);
    }

    /**
     * Stops the ring shooting motor
     */
    public void stopShooterMotor() {
        shootingMotor.setPower(0);
    }
}
