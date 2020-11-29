package org.firstinspires.ftc.teamcode.autonomous.blueteam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomousOpMode;
import org.firstinspires.ftc.teamcode.robot.drivetrain.mecanum.AutonomousMecanumMover;

@Autonomous(name = "BlueAutonomousMode", group = "Mecannum")

public class BlueAutonomousMode extends AbstractAutonomousOpMode {


    @Override
    protected void initOpModeAfterStart() {

    }

    @Override
    protected void executeOpMode() {
        int rings = robot.ringDetector.detectRings();
        telemetry.addData("Rings ", rings);
        telemetry.update();

        if(rings == 0){
            ///execute Rings = 0 case
            BlueRingCase0AutonomousOpMode ringCase0AutonomousOpMode = new BlueRingCase0AutonomousOpMode(autonomousRobotMover, this);
            ringCase0AutonomousOpMode.executeOpMode();
        }else if(rings == 1){
            ///execute Rings = 1 case
            BlueRingCase1AutonomousOpMode ringCase0AutonomousOpMode = new BlueRingCase1AutonomousOpMode(autonomousRobotMover, this);
            ringCase0AutonomousOpMode.executeOpMode();
        }else if(rings == 4){
            ///execute Rings = 4 case
            BlueRingCase4AutonomousOpMode ringCase0AutonomousOpMode = new BlueRingCase4AutonomousOpMode(autonomousRobotMover, this);
            ringCase0AutonomousOpMode.executeOpMode();
        }
    }
}
