package org.firstinspires.ftc.teamcode.autonomousRR.redteam;

import android.provider.MediaStore;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;
import org.firstinspires.ftc.teamcode.robot.actionparts.Intakesystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.RingFlipperSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.ShootingSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.WobblegoalArmLeft;
import org.firstinspires.ftc.teamcode.robot.actionparts.WobblegoalArmRight;

public class RedRingCase0AutonomousOpModeRR {

    private MecanumDriveRR mecanumDriveRR;
    private ShootingSystem shootingSystem;
    private RingFlipperSystem ringFlipperSystem;
    private Intakesystem intakesystem;
    public WobblegoalArmRight wobblegoalArmRight;
    public WobblegoalArmLeft wobblegoalArmLeft;
    private LinearOpMode currOpMode;
    private Pose2d initPos;
    private Pose2d lastPos;

    public RedRingCase0AutonomousOpModeRR(MecanumDriveRR mecanumDriveRR, GearheadsMecanumRobotRR gearheadsMecanumRobotRR, LinearOpMode currOpMode) {
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

        Trajectory traj1 = mecanumDriveRR.trajectoryBuilder(initPos, 0).splineTo(new Vector2d(2, -50), -0.4)
                .build();

        Trajectory traj2 = mecanumDriveRR.trajectoryBuilder(traj1.end(), Math.PI - 0.4)
                .splineToLinearHeading(new Pose2d(-2, -36, 0.35), 0.60)//Shooting angle was 0.65
                .build();

        Trajectory traj3 = mecanumDriveRR.trajectoryBuilder(traj2.end(), 0.75)
                .splineTo(new Vector2d(-56, -11), -Math.PI / 3).build();

        currOpMode.sleep(500);

        //To slow down robot
        TrajectoryConstraints slowConstraints = new MecanumConstraints(DriveConstants.SLOW_ROBOT_CONSTRAINTS, DriveConstants.TRACK_WIDTH);
        Trajectory traj4 = mecanumDriveRR.trajectoryBuilder(traj3.end())
                .splineTo(new Vector2d(-47.5, -21.5), -Math.PI / 3, slowConstraints)
                .splineTo(new Vector2d(7, -44), 0, slowConstraints).build();


        shootingSystem.operateShooterMotors(0.15, 0.075);
        ringFlipperSystem.resetPosition();


        mecanumDriveRR.followTrajectory(traj1);
        mecanumDriveRR.followTrajectory(traj2);

        ringFlipperSystem.pushRing();


        //mecanumDriveRR.turn(0.1);
        currOpMode.sleep(400);
        ringFlipperSystem.pushRing();
        //mecanumDriveRR.turn(0.1);
        currOpMode.sleep(400);
        ringFlipperSystem.pushRing();

        shootingSystem.stopShooterMotor();
        mecanumDriveRR.followTrajectory(traj3);
        mecanumDriveRR.followTrajectory(traj4);

        //Park and set for TeleOps
//        Trajectory traj5 = mecanumDriveRR.trajectoryBuilder(traj4.end())
//                .back(6).build();
//        mecanumDriveRR.followTrajectory(traj5);
//        mecanumDriveRR.turn(Math.toRadians(90));
//
//        Trajectory traj6 = mecanumDriveRR.trajectoryBuilder(traj5.end())
//                .strafeRight(6).build();
//        mecanumDriveRR.followTrajectory(traj6);
    }



}
