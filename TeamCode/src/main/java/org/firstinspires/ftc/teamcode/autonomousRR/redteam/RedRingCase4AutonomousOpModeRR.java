package org.firstinspires.ftc.teamcode.autonomousRR.redteam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomousRR.RedTeamPositions;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;
import org.firstinspires.ftc.teamcode.robot.actionparts.Intakesystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.RingFlipperSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.ShootingSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.WobblegoalArmLeft;
import org.firstinspires.ftc.teamcode.robot.actionparts.WobblegoalArmRight;

import static java.lang.Math.PI;

public class RedRingCase4AutonomousOpModeRR {

    private MecanumDriveRR mecanumDriveRR;
    private ShootingSystem shootingSystem;
    private RingFlipperSystem ringFlipperSystem;
    private Intakesystem intakesystem;
    public WobblegoalArmRight wobblegoalArmRight;
    public WobblegoalArmLeft wobblegoalArmLeft;
    private LinearOpMode currOpMode;
    private Pose2d initPos;
    private Pose2d lastPos;

    public RedRingCase4AutonomousOpModeRR(MecanumDriveRR mecanumDriveRR, GearheadsMecanumRobotRR gearheadsMecanumRobotRR, LinearOpMode currOpMode) {
        this.mecanumDriveRR = mecanumDriveRR;
        this.shootingSystem = gearheadsMecanumRobotRR.shootingSystem;
        this.intakesystem = gearheadsMecanumRobotRR.intakesystem;
        this.ringFlipperSystem = gearheadsMecanumRobotRR.ringFlipperSystem;
        this.wobblegoalArmLeft = gearheadsMecanumRobotRR.wobblegoalArmLeft;
        this.wobblegoalArmRight = gearheadsMecanumRobotRR.wobblegoalArmRight;
        this.currOpMode = currOpMode;
        this.initPos = new Pose2d(-60, -48, 0);
    }

    public void setLastPos(Pose2d lastKnownPos){
        this.lastPos = lastKnownPos;
    }

    public void executeOpMode() {
        initPos = RedTeamPositions.SHOOTING_POS;
        mecanumDriveRR.setPoseEstimate(initPos);

//        //From Starting position to Case 0 drop zone
//        Trajectory traj1 = mecanumDriveRR.trajectoryBuilder(initPos, 0)
//                .splineTo(new Vector2d(4.97 + 48, -59.69), 5.6)
//                .build();
//
//        //From Case 0 drop zone to shooting position
//        Trajectory traj2 = mecanumDriveRR.trajectoryBuilder(traj1.end())
//                .splineToLinearHeading(RedTeamPositions.SHOOTING_POS, 0)//Shooting angle was 0.65
//                .build();

        //From Shooting position to Wobble goal 2 catch position
        Trajectory traj3 = mecanumDriveRR.trajectoryBuilder(RedTeamPositions.SHOOTING_POS, 0)
                .splineTo(new Vector2d(-39.6, 3.1), 3.7)
                .splineTo(new Vector2d(-56.37, -5.82), 4.75)  //Newly added point
                .splineTo(RedTeamPositions.WOBBLE_GOAL_2_PICKUP_XY, RedTeamPositions.WOBBLE_GOAL_2_PICKUP_HEADING)
                .build();

//
//        //To slow down robot, from Wobble goal 2 catch position to Case 0 drop position
//        TrajectoryConstraints slowConstraints = new MecanumConstraints(DriveConstants.SLOW_ROBOT_CONSTRAINTS, DriveConstants.TRACK_WIDTH);
//        Trajectory traj4 = mecanumDriveRR.trajectoryBuilder(traj3.end())
//                .splineTo(new Vector2d(-51.3, -17.56), 5.121, slowConstraints)
//                .splineTo(new Vector2d(3.76+ 48, -62.76), 0, slowConstraints).build();
//
//
//        shootingSystem.operateShooterMotors(0.15, 0.075);
//        ringFlipperSystem.resetPosition();
//
//
//        mecanumDriveRR.followTrajectory(traj1);
//        mecanumDriveRR.followTrajectory(traj2);
//
//        ringFlipperSystem.pushRing();
//        currOpMode.sleep(500);
//        ringFlipperSystem.pushRing();
//        currOpMode.sleep(500);
//        ringFlipperSystem.pushRing();
//
//        shootingSystem.stopShooterMotor();
        mecanumDriveRR.followTrajectory(traj3);
//        currOpMode.sleep(1000);
//        mecanumDriveRR.followTrajectory(traj4);
//
//        Trajectory traj5 = mecanumDriveRR.trajectoryBuilder(traj4.end())
//                .back(48).build();
//        mecanumDriveRR.followTrajectory(traj5);
    }
}
