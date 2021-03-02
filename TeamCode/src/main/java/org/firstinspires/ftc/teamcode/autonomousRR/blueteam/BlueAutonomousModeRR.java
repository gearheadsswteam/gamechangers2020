package org.firstinspires.ftc.teamcode.autonomousRR.blueteam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomousRR.AbstractAutonomousOpModeRR;

@Autonomous
public class BlueAutonomousModeRR extends AbstractAutonomousOpModeRR {
    int ringNum;

    //Blue Team init position
    Pose2d initPos = new Pose2d(-60, 48, 0);

    public BlueAutonomousModeRR() {
        super.TEAM_TYPE = AbstractAutonomousOpModeRR.BLUE_TEAM;
    }

    @Override
    protected void initOpModeBeforeStart() {
        super.initOpModeBeforeStart();
        mecanumDriveRR.setPoseEstimate(initPos);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    protected void initOpModeAfterStart() {

    }

    @Override
    protected void executeOpMode() {

        int rings = robot.ringDetector.detectRings();
        rings = 4;
        telemetry.addData("Rings ", rings);
        telemetry.update();
        sleep(500);


        if (rings == 0) {
            ///execute Rings = 0 case
            BlueRingCase0AutonomousOpModeRR ringCase0AutonomousOpMode = new BlueRingCase0AutonomousOpModeRR(mecanumDriveRR, autonomousRobotMover.robot, this);
            ringCase0AutonomousOpMode.setLastPos(initPos);
            ringCase0AutonomousOpMode.executeOpMode();
        } else if (rings == 1) {
            BlueRingCase1AutonomousOpModeRR ringCase1AutonomousOpMode = new BlueRingCase1AutonomousOpModeRR(mecanumDriveRR, autonomousRobotMover.robot, this);
            ringCase1AutonomousOpMode.setLastPos(initPos);
            ringCase1AutonomousOpMode.executeOpMode();
        } else if (rings == 4) {
            BlueRingCase4AutonomousOpModeRR ringCase4AutonomousOpMode = new BlueRingCase4AutonomousOpModeRR(mecanumDriveRR, autonomousRobotMover.robot, this);
            ringCase4AutonomousOpMode.setLastPos(initPos);
            ringCase4AutonomousOpMode.executeOpMode();

//            TestRedPath tp= new TestRedPath(mecanumDriveRR, autonomousRobotMover.robot, this);
//            tp.setLastPos(RedTeamPositions.SHOOTING_POS);
//            tp.executeOpMode();
        }
    }
}
