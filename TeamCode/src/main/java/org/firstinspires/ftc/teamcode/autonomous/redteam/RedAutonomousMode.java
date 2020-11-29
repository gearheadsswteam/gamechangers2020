package org.firstinspires.ftc.teamcode.autonomous.redteam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomousOpMode;
import org.firstinspires.ftc.teamcode.autonomous.blueteam.BlueRingCase0AutonomousOpMode;
import org.firstinspires.ftc.teamcode.autonomous.blueteam.BlueRingCase1AutonomousOpMode;
import org.firstinspires.ftc.teamcode.autonomous.blueteam.BlueRingCase4AutonomousOpMode;

@Autonomous(name = "RedAutonomousMode", group = "Mecannum")

public class RedAutonomousMode extends AbstractAutonomousOpMode {
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
            RedRingCase0AutonomousOpMode ringCase0AutonomousOpMode = new RedRingCase0AutonomousOpMode(autonomousRobotMover, this);
            ringCase0AutonomousOpMode.executeOpMode();
        }else if(rings == 1){
            ///execute Rings = 1 case
            RedRingCase1AutonomousOpMode ringCase0AutonomousOpMode = new RedRingCase1AutonomousOpMode(autonomousRobotMover, this);
            ringCase0AutonomousOpMode.executeOpMode();
        }else if(rings == 4){
            ///execute Rings = 4 case
            RedRingCase4AutonomousOpMode ringCase0AutonomousOpMode = new RedRingCase4AutonomousOpMode(autonomousRobotMover, this);
            ringCase0AutonomousOpMode.executeOpMode();
        }
    }
}
