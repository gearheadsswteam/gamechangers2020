package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.hardware.DcMotor;

public class PositionEncoders {
    private DcMotor motor;
    private final double MM_PER_INCH = 25.4;
    private final double DIAMETER_INCHES = 35/MM_PER_INCH; //TODO need to find this wheelsize
    private final double COUNTS_PER_REV = 8192;
    private final double ONE_REV_DISTANCE = 3.14*DIAMETER_INCHES;


    public PositionEncoders(DcMotor motor) {
        this.motor = motor;
    }

    public int getEncoderValue(){
        return motor.getCurrentPosition();
    }

    public void resetEncoderValue(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double distanceTravelledInInches(){
        int encoderValue = getEncoderValue();
        double revs = encoderValue/COUNTS_PER_REV;
        return revs * ONE_REV_DISTANCE;
    }
}
