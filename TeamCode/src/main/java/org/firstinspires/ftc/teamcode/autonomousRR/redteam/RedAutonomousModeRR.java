package org.firstinspires.ftc.teamcode.autonomousRR.redteam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomousRR.AbstractAutonomousOpModeRR;

@Autonomous
public class RedAutonomousModeRR extends AbstractAutonomousOpModeRR {
    int ringNum;
    Pose2d initPos = new Pose2d(0, 0, 0);

    @Override
    protected void initOpModeBeforeStart() {
        super.initOpModeBeforeStart();
        driveSystem.setPoseEstimate(initPos);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

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
            RedRingCase0AutonomousOpModeRR ringCase0AutonomousOpMode = new RedRingCase0AutonomousOpModeRR(driveSystem, autonomousRobotMover.robot,this);
            ringCase0AutonomousOpMode.executeOpMode();
        }else if(rings == 1){

        }else if(rings == 4){

        }
    }
}
