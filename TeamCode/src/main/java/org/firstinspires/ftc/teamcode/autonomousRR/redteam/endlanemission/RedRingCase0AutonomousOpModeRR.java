package org.firstinspires.ftc.teamcode.autonomousRR.redteam.endlanemission;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomousRR.redteam.RedTeamPositions;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;
import org.firstinspires.ftc.teamcode.robot.actionparts.Intakesystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.RingFlipperSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.ShootingSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.WobblegoalArmLeft;
import org.firstinspires.ftc.teamcode.robot.actionparts.WobblegoalArmRight;

class RedRingCase0AutonomousOpModeRR {

    private MecanumDriveRR mecanumDriveRR;
    private ShootingSystem shootingSystem;
    private RingFlipperSystem ringFlipperSystem;
    private Intakesystem intakesystem;
    public WobblegoalArmRight wobblegoalArmRight;
    public WobblegoalArmLeft wobblegoalArmLeft;
    private LinearOpMode currOpMode;
    private Pose2d initPos;
    private Pose2d lastPos;

     RedRingCase0AutonomousOpModeRR(MecanumDriveRR mecanumDriveRR, GearheadsMecanumRobotRR gearheadsMecanumRobotRR, LinearOpMode currOpMode) {
        this.mecanumDriveRR = mecanumDriveRR;
        this.shootingSystem = gearheadsMecanumRobotRR.shootingSystem;
        this.intakesystem = gearheadsMecanumRobotRR.intakesystem;
        this.ringFlipperSystem = gearheadsMecanumRobotRR.ringFlipperSystem;
        this.wobblegoalArmLeft = gearheadsMecanumRobotRR.wobblegoalArmLeft;
        this.wobblegoalArmRight = gearheadsMecanumRobotRR.wobblegoalArmRight;
        this.currOpMode = currOpMode;
        this.initPos = RedTeamPositions.INIT_POS;
    }

    public void setLastPos(Pose2d lastKnownPos){
        this.lastPos = lastKnownPos;
    }

    public void executeOpMode() {

        //From Starting position to Case 0 drop zone
        Trajectory traj1 = mecanumDriveRR.trajectoryBuilder(initPos, 0)
                .splineTo(new Vector2d(4.97-3, -59.69), 5.6)
                .build();

        //From Case 1 drop zone to shooting position
        Trajectory traj2 = mecanumDriveRR.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(RedTeamPositions.PARK_ONLY__RIGHT_LANE_POSITION)
                .build();

        mecanumDriveRR.followTrajectory(traj1);
        mecanumDriveRR.followTrajectory(traj2);
    }
}
