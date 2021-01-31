package org.firstinspires.ftc.teamcode.autonomousRR.redteam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomousRR.RedTeamPositions;
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
        Trajectory traj1 = mecanumDriveRR.trajectoryBuilder(initPos, 0).splineTo(new Vector2d((2 + 48), -56), -0.4)
                .build();

        Trajectory traj2 = mecanumDriveRR.trajectoryBuilder(new Pose2d((2 + 48), -56, -0.4), Math.PI - 0.4)
                .splineToLinearHeading(new Pose2d(-2, -36, 0.35), 0.35)
                .build();

        Trajectory traj3 = mecanumDriveRR.trajectoryBuilder(new Pose2d(-2, -36, 0.55), 0.55)
                .splineTo(new Vector2d(-56, -11), -Math.PI / 3)
                .splineTo(new Vector2d(-47.5, -21.5), -Math.PI / 3)
                .splineTo(new Vector2d((2 + 48), -50), 0).build();

        shootingSystem.operateShooterMotors(0.15, 0.075);
        ringFlipperSystem.resetPosition();


        mecanumDriveRR.followTrajectory(traj1);
        mecanumDriveRR.followTrajectory(traj2);

        ringFlipperSystem.pushRing();


        mecanumDriveRR.turn(0.1);
        currOpMode.sleep(400);
        ringFlipperSystem.pushRing();
        mecanumDriveRR.turn(0.1);
        currOpMode.sleep(400);
        ringFlipperSystem.pushRing();

        shootingSystem.stopShooterMotor();
        mecanumDriveRR.followTrajectory(traj3);
    }
}
