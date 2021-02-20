package org.firstinspires.ftc.teamcode.autonomousRR.redteam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomousRR.AbstractAutonomousOpModeRR;
import org.firstinspires.ftc.teamcode.autonomousRR.RedTeamPositions;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;
import org.firstinspires.ftc.teamcode.robot.actionparts.Intakesystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.RingFlipperSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.ShootingSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.WobblegoalArmLeft;
import org.firstinspires.ftc.teamcode.robot.actionparts.WobblegoalArmRight;

public class TestRedPath  {
    private MecanumDriveRR mecanumDriveRR;
    private ShootingSystem shootingSystem;
    private RingFlipperSystem ringFlipperSystem;
    private Intakesystem intakesystem;
    public WobblegoalArmRight wobblegoalArmRight;
    public WobblegoalArmLeft wobblegoalArmLeft;
    private LinearOpMode currOpMode;
    private Pose2d initPos;
    private Pose2d lastPos;

    public TestRedPath (MecanumDriveRR mecanumDriveRR, GearheadsMecanumRobotRR gearheadsMecanumRobotRR, LinearOpMode currOpMode) {
        this.mecanumDriveRR = mecanumDriveRR;
        this.shootingSystem = gearheadsMecanumRobotRR.shootingSystem;
        this.intakesystem = gearheadsMecanumRobotRR.intakesystem;
        this.ringFlipperSystem = gearheadsMecanumRobotRR.ringFlipperSystem;
        this.wobblegoalArmLeft = gearheadsMecanumRobotRR.wobblegoalArmLeft;
        this.wobblegoalArmRight = gearheadsMecanumRobotRR.wobblegoalArmRight;
        this.currOpMode = currOpMode;
        this.initPos = RedTeamPositions.SHOOTING_POS;
    }

    public void setLastPos(Pose2d lastKnownPos){
        this.lastPos = lastKnownPos;
    }

    public void executeOpMode() {
        initPos = RedTeamPositions.SHOOTING_POS;
        mecanumDriveRR.setPoseEstimate(initPos);

//
        Trajectory traj3 = mecanumDriveRR.trajectoryBuilder(RedTeamPositions.SHOOTING_POS, 0)
                .splineTo(new Vector2d(-39.6, 9.1), 3.3)
                //.splineTo(new Vector2d(-56.37, -5.82), 4.75)  //Newly added point
                .splineTo(new Vector2d(-57.52+6, -7.70), 5.0)
                .build();

//
//        //To slow down robot, from Wobble goal 2 catch position to Case 0 drop position
       TrajectoryConstraints slowConstraints = new MecanumConstraints(DriveConstants.SLOW_ROBOT_CONSTRAINTS, DriveConstants.TRACK_WIDTH);
        Trajectory traj4 = mecanumDriveRR.trajectoryBuilder(traj3.end())
                .splineTo(new Vector2d(-51.3+2, -17.56), 5.0, slowConstraints)
                .splineTo(new Vector2d(3.76+ 48, -62.76), 0, slowConstraints).build();
//
//
//
        mecanumDriveRR.followTrajectory(traj3);
        currOpMode.sleep(1000);
        mecanumDriveRR.followTrajectory(traj4);
//
    }



}
