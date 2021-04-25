package org.firstinspires.ftc.teamcode.autonomousRR.redteam.endlanelasertech;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomousRR.redteam.RedTeamPositions;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;
import org.firstinspires.ftc.teamcode.robot.actionparts.Intakesystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.RingFlipperSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.ShootingSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.WobblegoalArmLeft;
import org.firstinspires.ftc.teamcode.robot.actionparts.WobblegoalArmRight;

class RedRingCase4AutonomousOpModeRR {

    private MecanumDriveRR mecanumDriveRR;
    private ShootingSystem shootingSystem;
    private RingFlipperSystem ringFlipperSystem;
    private Intakesystem intakesystem;
    public WobblegoalArmRight wobblegoalArmRight;
    public WobblegoalArmLeft wobblegoalArmLeft;
    public Servo intakeGaurdServoMotor;
    private LinearOpMode currOpMode;
    private Pose2d initPos;
    private Pose2d lastPos;

    RedRingCase4AutonomousOpModeRR(MecanumDriveRR mecanumDriveRR, GearheadsMecanumRobotRR gearheadsMecanumRobotRR, LinearOpMode currOpMode) {
        this.mecanumDriveRR = mecanumDriveRR;
        this.shootingSystem = gearheadsMecanumRobotRR.shootingSystem;
        this.intakesystem = gearheadsMecanumRobotRR.intakesystem;
        this.ringFlipperSystem = gearheadsMecanumRobotRR.ringFlipperSystem;
        this.wobblegoalArmLeft = gearheadsMecanumRobotRR.wobblegoalArmLeft;
        this.wobblegoalArmRight = gearheadsMecanumRobotRR.wobblegoalArmRight;
        this.intakeGaurdServoMotor = gearheadsMecanumRobotRR.intakeGaurdServo;
        this.currOpMode = currOpMode;
        this.initPos = RedTeamPositions.INIT_POS;
    }

    public void setLastPos(Pose2d lastKnownPos) {
        this.lastPos = lastKnownPos;
    }

    public void executeOpMode() {
        //Clear the ring set up position + drop WG 1
        Trajectory traj1 = mecanumDriveRR.trajectoryBuilder(initPos, 0)
                .splineTo(new Vector2d(-24, -56), 0)
                .splineTo(new Vector2d(4.97 - 3 + 48, -59.69 + 2), 5.6)
                .build();


        //From Case 0 drop zone to shooting position
        Trajectory traj2 = mecanumDriveRR.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(RedTeamPositions.PARK_ONLY__RIGHT_LANE_SHOOTING_POSITION)//Shooting angle was 0.65
                .build();


        TrajectoryConstraints slowConstraints = new MecanumConstraints(DriveConstants.SLOW_1_ROBOT_CONSTRAINTS, DriveConstants.TRACK_WIDTH);
        //From Case 0 drop zone to shooting position
        Trajectory traj3 = mecanumDriveRR.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(RedTeamPositions.PARK_ONLY__RIGHT_LANE_POSITION, slowConstraints)//Shooting angle was 0.65
                .build();

        shootingSystem.shootHighGoals();
        ringFlipperSystem.resetPosition();

        mecanumDriveRR.followTrajectory(traj1);

        mecanumDriveRR.followTrajectory(traj2);

        ringFlipperSystem.pushRing();
        currOpMode.sleep(500);
        ringFlipperSystem.pushRing();
        currOpMode.sleep(500);
        ringFlipperSystem.pushRing();
        currOpMode.sleep(500);

        mecanumDriveRR.followTrajectory(traj3);
    }
}
