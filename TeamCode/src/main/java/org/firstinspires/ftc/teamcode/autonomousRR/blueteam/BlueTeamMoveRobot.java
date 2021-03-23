package org.firstinspires.ftc.teamcode.autonomousRR.blueteam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomousRR.AbstractAutonomousOpModeRR;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

@Autonomous(name = "BlueRingPark", group = "Blue")
public class BlueTeamMoveRobot extends AbstractAutonomousOpModeRR {
    private Pose2d initPos = BlueTeamPositions.WG2_START_POS;
    private Pose2d intermeditePoint = new Pose2d(-60+24, 12, 0);;
    private Pose2d destPos = BlueTeamPositions.SHOOTING_POS;
    private long delayMs = 5000;

    public BlueTeamMoveRobot() {
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
        TrajectoryConstraints slowConstraints = new MecanumConstraints(DriveConstants.SLOW_ROBOT_CONSTRAINTS, DriveConstants.TRACK_WIDTH);
        Trajectory traj0 = mecanumDriveRR.trajectoryBuilder(initPos, 0)
                .splineToSplineHeading(intermeditePoint,0,slowConstraints)
                .splineToSplineHeading(BlueTeamPositions.PARK_RING_DROP_POS, 0,slowConstraints)
                .build();

        //From Starting position to Case 0 drop zone
        Trajectory traj1 = mecanumDriveRR.trajectoryBuilder(traj0.end(), 0)
                .splineToSplineHeading(BlueTeamPositions.SHOOTING_POS_CASE_1, 0)
                .build();

        Trajectory traj2 = mecanumDriveRR.trajectoryBuilder(traj1.end(), 0)
                .forward(23).build();

        robot.shootingSystem.shootHighGoals();
        this.sleep(delayMs);
        mecanumDriveRR.followTrajectory(traj0);
        mecanumDriveRR.followTrajectory(traj1);

        robot.ringFlipperSystem.pushRing();
        sleep(500);
        robot.ringFlipperSystem.pushRing();
        sleep(500);
        robot.ringFlipperSystem.pushRing();

        mecanumDriveRR.followTrajectory(traj2);

        robot.shootingSystem.stopShooterMotor();
    }
}
