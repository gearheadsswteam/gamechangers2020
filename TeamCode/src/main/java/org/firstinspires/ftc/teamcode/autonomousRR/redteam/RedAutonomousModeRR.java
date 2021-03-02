package org.firstinspires.ftc.teamcode.autonomousRR.redteam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomousRR.AbstractAutonomousOpModeRR;

@Autonomous
public class RedAutonomousModeRR extends AbstractAutonomousOpModeRR {
    int ringNum;

    Pose2d initPos = new Pose2d(-60, -48, 0);

    public RedAutonomousModeRR() {
        super.TEAM_TYPE = AbstractAutonomousOpModeRR.RED_TEAM;
    }

    @Override
    protected void initOpModeBeforeStart() {
        super.initOpModeBeforeStart();
        mecanumDriveRR.setPoseEstimate(initPos);

        int rings = robot.ringDetector.detectRings();
        //rings = 4;
        telemetry.addData("Rings ", rings);
        telemetry.update();
        sleep(500);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    protected void initOpModeAfterStart() {

    }

    @Override
    protected void executeOpMode() {

        int rings = robot.ringDetector.detectRings();
//        rings = 4;
//        telemetry.addData("Rings ", rings);
//        telemetry.update();
//        sleep(500);


        if (rings == 0) {
            ///execute Rings = 0 case
            RedRingCase0AutonomousOpModeRR ringCase0AutonomousOpMode = new RedRingCase0AutonomousOpModeRR(mecanumDriveRR, autonomousRobotMover.robot, this);
            ringCase0AutonomousOpMode.setLastPos(initPos);
            ringCase0AutonomousOpMode.executeOpMode();
        } else if (rings == 1) {
            RedRingCase1AutonomousOpModeRR ringCase1AutonomousOpMode = new RedRingCase1AutonomousOpModeRR(mecanumDriveRR, autonomousRobotMover.robot, this);
            ringCase1AutonomousOpMode.setLastPos(initPos);
            ringCase1AutonomousOpMode.executeOpMode();
        } else if (rings == 4) {
            RedRingCase4AutonomousOpModeRR ringCase4AutonomousOpMode = new RedRingCase4AutonomousOpModeRR(mecanumDriveRR, autonomousRobotMover.robot, this);
            ringCase4AutonomousOpMode.setLastPos(initPos);
            ringCase4AutonomousOpMode.executeOpMode();
        }
    }
}
