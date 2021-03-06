package org.firstinspires.ftc.teamcode.autonomousRR.blueteam;

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

class BlueRingCase0AutonomousOpModeRR {

    private MecanumDriveRR mecanumDriveRR;
    private ShootingSystem shootingSystem;
    private RingFlipperSystem ringFlipperSystem;
    private Intakesystem intakesystem;
    public WobblegoalArmRight wobblegoalArmRight;
    public WobblegoalArmLeft wobblegoalArmLeft;
    private LinearOpMode currOpMode;
    private Pose2d initPos;
    private Pose2d lastPos;

    BlueRingCase0AutonomousOpModeRR(MecanumDriveRR mecanumDriveRR, GearheadsMecanumRobotRR gearheadsMecanumRobotRR, LinearOpMode currOpMode) {
        this.mecanumDriveRR = mecanumDriveRR;
        this.shootingSystem = gearheadsMecanumRobotRR.shootingSystem;
        this.intakesystem = gearheadsMecanumRobotRR.intakesystem;
        this.ringFlipperSystem = gearheadsMecanumRobotRR.ringFlipperSystem;
        this.wobblegoalArmLeft = gearheadsMecanumRobotRR.wobblegoalArmLeft;
        this.wobblegoalArmRight = gearheadsMecanumRobotRR.wobblegoalArmRight;
        this.currOpMode = currOpMode;
    }

    public void setLastPos(Pose2d lastKnownPos) {
        this.initPos = lastKnownPos;
    }

    public void executeOpMode() {
        //From Starting position to Case 0 drop zone
        Trajectory traj1 = mecanumDriveRR.trajectoryBuilder(initPos, 0)
                .splineTo(new Vector2d(4.97 - 3, 59.69), -5.6)
                .build();

        //From Case 0 drop zone to shooting position
        Trajectory traj2 = mecanumDriveRR.trajectoryBuilder(traj1.end())
                .splineToLinearHeading(BlueTeamPositions.SHOOTING_POS, 0)//Shooting angle was 0.65
                .build();

        //Shooting position to wobble goal 2 grab position
        Trajectory traj3 = mecanumDriveRR.trajectoryBuilder(traj2.end(), 0)
                .splineTo(new Vector2d(-39.6, -9.1), -3.3)
                .splineTo(new Vector2d(-57.52 + 6, 7.70 - 3), -5.0)
                .build();


        //To slow down robot, from Wobble goal 2 catch position to Case 0 drop position
        TrajectoryConstraints slowConstraints = new MecanumConstraints(DriveConstants.SLOW_ROBOT_CONSTRAINTS, DriveConstants.TRACK_WIDTH);
        Trajectory traj4 = mecanumDriveRR.trajectoryBuilder(traj3.end())
                .lineTo(new Vector2d(-51.3, 17.56), slowConstraints)
                .splineToSplineHeading(new Pose2d(3.76 + 3, 52.76, 0), 0, slowConstraints).build();


        shootingSystem.shootHighGoals();
        ringFlipperSystem.resetPosition();


        mecanumDriveRR.followTrajectory(traj1);
        mecanumDriveRR.followTrajectory(traj2);

        ringFlipperSystem.pushRing();
        currOpMode.sleep(500);
        ringFlipperSystem.pushRing();
        currOpMode.sleep(500);
        ringFlipperSystem.pushRing();

        shootingSystem.stopShooterMotor();

        //        mecanumDriveRR.followTrajectory(traj3);
//        currOpMode.sleep(1000);
//        mecanumDriveRR.followTrajectory(traj4);

        park(traj2);
    }

    /**
     * Park from shooring position
     *
     * @param shootingPosition
     */
    private void park(Trajectory shootingPosition) {
        //Go to the Ring stack and hit it
        Trajectory trajForRingStack1 = mecanumDriveRR.trajectoryBuilder(shootingPosition.end())
                .forward(20-4).build();
        mecanumDriveRR.followTrajectory(trajForRingStack1);
        currOpMode.sleep(500);
    }

}
