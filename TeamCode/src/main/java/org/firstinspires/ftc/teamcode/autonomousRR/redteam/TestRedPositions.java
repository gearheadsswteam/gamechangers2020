package org.firstinspires.ftc.teamcode.autonomousRR.redteam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomousRR.AbstractAutonomousOpModeRR;
import org.firstinspires.ftc.teamcode.autonomousRR.RedTeamPositions;

@Autonomous
public class TestRedPositions extends AbstractAutonomousOpModeRR {
    Pose2d initPos = RedTeamPositions.POS1;

    @Override
    protected void initOpModeBeforeStart() {
        super.initOpModeBeforeStart();
        mecanumDriveRR.setPoseEstimate(RedTeamPositions.POS1);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    @Override
    protected void initOpModeAfterStart() {

    }

    @Override
    protected void executeOpMode() {

        Trajectory trajectory = mecanumDriveRR.trajectoryBuilder(RedTeamPositions.POS1, 0).lineToLinearHeading(RedTeamPositions.POS2).build();
        mecanumDriveRR.followTrajectory(trajectory);
        Pose2d lastPos = mecanumDriveRR.getPoseEstimate();
        sleep(500);

        trajectory = mecanumDriveRR.trajectoryBuilder(lastPos, 0).lineToLinearHeading(RedTeamPositions.POS3).build();
        mecanumDriveRR.followTrajectory(trajectory);
        lastPos = mecanumDriveRR.getPoseEstimate();
        sleep(500);

        trajectory = mecanumDriveRR.trajectoryBuilder(lastPos, 0).lineToLinearHeading(RedTeamPositions.POS4).build();
        mecanumDriveRR.followTrajectory(trajectory);
        lastPos = mecanumDriveRR.getPoseEstimate();
        sleep(500);

        trajectory = mecanumDriveRR.trajectoryBuilder(lastPos, 0).lineToLinearHeading(RedTeamPositions.POS5).build();
        mecanumDriveRR.followTrajectory(trajectory);
        lastPos = mecanumDriveRR.getPoseEstimate();
        sleep(500);

        trajectory = mecanumDriveRR.trajectoryBuilder(lastPos, 0).lineToLinearHeading(RedTeamPositions.POS6).build();
        mecanumDriveRR.followTrajectory(trajectory);
        lastPos = mecanumDriveRR.getPoseEstimate();
        sleep(500);

        trajectory = mecanumDriveRR.trajectoryBuilder(lastPos, 0).lineToLinearHeading(RedTeamPositions.POS7).build();
        mecanumDriveRR.followTrajectory(trajectory);
        lastPos = mecanumDriveRR.getPoseEstimate();
        sleep(500);

        trajectory = mecanumDriveRR.trajectoryBuilder(lastPos, 0).lineToLinearHeading(RedTeamPositions.POS8).build();
        mecanumDriveRR.followTrajectory(trajectory);
        lastPos = mecanumDriveRR.getPoseEstimate();
        sleep(500);
    }
}
