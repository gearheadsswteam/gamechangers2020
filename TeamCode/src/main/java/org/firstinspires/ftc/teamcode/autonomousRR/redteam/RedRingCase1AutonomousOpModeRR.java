package org.firstinspires.ftc.teamcode.autonomousRR.redteam;

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

public class RedRingCase1AutonomousOpModeRR {

    private MecanumDriveRR mecanumDriveRR;
    private ShootingSystem shootingSystem;
    private RingFlipperSystem ringFlipperSystem;
    private Intakesystem intakesystem;
    public WobblegoalArmRight wobblegoalArmRight;
    public WobblegoalArmLeft wobblegoalArmLeft;
    private LinearOpMode currOpMode;
    private Pose2d initPos;
    private Pose2d lastPos;

    public RedRingCase1AutonomousOpModeRR(MecanumDriveRR mecanumDriveRR, GearheadsMecanumRobotRR gearheadsMecanumRobotRR, LinearOpMode currOpMode) {
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
        Trajectory traj0 = mecanumDriveRR.trajectoryBuilder(initPos, 0)
                .splineTo(new Vector2d(-24, -56), 0)
                .build();


        //From Starting position to Case 1 drop zone
        Trajectory traj1 = mecanumDriveRR.trajectoryBuilder(traj0.end(), 0)
                .splineTo(new Vector2d(4.97+24, -59.69+24), 5.6)
                .build();

        //From Case 0 drop zone to shooting position
        //TODO Why Math.PI -0.4
        Trajectory traj2 = mecanumDriveRR.trajectoryBuilder(traj1.end())
                .splineToLinearHeading(new Pose2d(4.21, -36.82, 0), 0)//Shooting angle was 0.65
                .build();

        //From Shooting position to Wobble goal 2 catch position
        Trajectory traj3 = mecanumDriveRR.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(-57.52, -7.70),5.121).build();


        //currOpMode.sleep(2000);

        //To slow down robot, from Wobble goal 2 catch position to Case 0 drop position
        TrajectoryConstraints slowConstraints = new MecanumConstraints(DriveConstants.SLOW_ROBOT_CONSTRAINTS, DriveConstants.TRACK_WIDTH);
        Trajectory traj4 = mecanumDriveRR.trajectoryBuilder(traj3.end())
                .splineTo(new Vector2d(-51.3, -17.56), 5.121, slowConstraints)
                .splineTo(new Vector2d(3.76+24, -62.76+24), 0, slowConstraints).build();


        shootingSystem.operateShooterMotors(0.15, 0.075);
        ringFlipperSystem.resetPosition();


        mecanumDriveRR.followTrajectory(traj1);
        mecanumDriveRR.followTrajectory(traj2);

        ringFlipperSystem.pushRing();
        currOpMode.sleep(400);
        ringFlipperSystem.pushRing();
        currOpMode.sleep(400);
        ringFlipperSystem.pushRing();

        shootingSystem.stopShooterMotor();
        mecanumDriveRR.followTrajectory(traj3);
        currOpMode.sleep(1500);
        mecanumDriveRR.followTrajectory(traj4);

        //Park and set for TeleOps
//        Trajectory traj5 = mecanumDriveRR.trajectoryBuilder(traj4.end())
//                .back(15).build();
//        mecanumDriveRR.followTrajectory(traj5);
//
//
//        Trajectory traj6 = mecanumDriveRR.trajectoryBuilder(traj5.end())
//                .splineTo(new Vector2d(5.02, -35.08), 1.492, slowConstraints).build();
//
//        mecanumDriveRR.followTrajectory(traj6);
    }
}
