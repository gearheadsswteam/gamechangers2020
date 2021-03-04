package org.firstinspires.ftc.teamcode.autonomousRR.blueteam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomousRR.AbstractAutonomousOpModeRR;

@Autonomous
public class BlueTeamMoveRobot extends AbstractAutonomousOpModeRR {
    private Pose2d initPos = BlueTeamPositions.WG2_START_POS;
    private Pose2d destPos = new Pose2d(12, 24, 0);
    private long delayMs = 5000;

    public BlueTeamMoveRobot() {
        super.TEAM_TYPE = AbstractAutonomousOpModeRR.RED_TEAM;
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
        //From Starting position to Case 0 drop zone
        Trajectory traj1 = mecanumDriveRR.trajectoryBuilder(initPos, 0)
                .splineToSplineHeading(destPos, 0)
                .build();
        this.sleep(delayMs);
        mecanumDriveRR.followTrajectory(traj1);

    }
}
