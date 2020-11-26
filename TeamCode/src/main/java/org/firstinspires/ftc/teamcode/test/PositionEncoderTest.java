package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomousOpMode;

@Autonomous(name = "Mecannum: PositionEncoderTest", group = "Mecannum")
public class PositionEncoderTest extends AbstractAutonomousOpMode {
    @Override
    protected void initOpModeAfterStart() {

    }

    @Override
    protected void executeOpMode() {
        autonomousRobotMover.robot.positionEncoderCenter.resetEncoderValue();
        while(opModeIsActive()){
            int encoderValue = autonomousRobotMover.robot.positionEncoderCenter.getEncoderValue();
            telemetry.addData("Encoder Value ", encoderValue);
            telemetry.addData("Distance Travelled ", autonomousRobotMover.robot.positionEncoderCenter.distanceTravelledInInches());
            telemetry.update();
            sleep(1000);
        }

    }
}
