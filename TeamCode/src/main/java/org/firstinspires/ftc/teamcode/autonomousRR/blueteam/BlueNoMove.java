package org.firstinspires.ftc.teamcode.autonomousRR.blueteam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomousRR.AbstractAutonomousOpModeRR;

@Autonomous(name = "BlueNoMove", group = "Blue")
public class BlueNoMove extends AbstractAutonomousOpModeRR {
    private Pose2d initPos = BlueTeamPositions.WG2_START_POS;
    private Pose2d destPos = BlueTeamPositions.SHOOTING_POS;
    private long delayMs = 5000;

    public BlueNoMove() {
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
       //Do nothing
    }
}
