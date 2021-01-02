package org.firstinspires.ftc.teamcode.autonomousRR.redteam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
        initPos = new Pose2d();
        mecanumDriveRR.setMotorPowers(0.5,0.5,0.5,0.5);
    }

    public void setLastPos(Pose2d lastKnownPos){
        this.lastPos = lastKnownPos;
    }

    public void executeOpMode() {
        goToWobbleGoalDropPosition();
        dropWobbleGoal();

        //Move from Wobble goal drop point to shooting position
        gotoShootingPosition();

        //Shoot the preloaded rings in the goal
        shootPreloadedRings();

        gotoGrabRingPosition();

        grabRings();

        gotoShootingPosition();

        shootPreloadedRings();

        //Park the robot
        park();
    }

    /**
     * Drops the wobble goal on the mat
     */
    private void dropWobbleGoal() {
        wobblegoalArmRight.setWobbleGoal();
        currOpMode.sleep(100);
        wobblegoalArmRight.ungrabWobbleGoal();
        currOpMode.sleep(750);
    }
    /**
     * Move the first wobble goal to the correct position
     */
    private void goToWobbleGoalDropPosition() {
        Trajectory traj1 = mecanumDriveRR.trajectoryBuilder(lastPos,0).lineToSplineHeading(RedTeamPositions.GOAL_0_POS).build();
        mecanumDriveRR.followTrajectory(traj1);
        lastPos = mecanumDriveRR.getPoseEstimate();
        currOpMode.sleep(500);
    }

    private void gotoShootingPosition() {
        Trajectory traj1 = mecanumDriveRR.trajectoryBuilder(lastPos,0).lineToLinearHeading(RedTeamPositions.SHOOTING_POS).build();
        mecanumDriveRR.followTrajectory(traj1);
        lastPos = mecanumDriveRR.getPoseEstimate();
        currOpMode.sleep(500);
    }

    private void gotoGrabRingPosition() {
        Trajectory traj1 = mecanumDriveRR.trajectoryBuilder(lastPos,0).lineToLinearHeading(RedTeamPositions.RING_GRAB_POS).build();
        mecanumDriveRR.followTrajectory(traj1);
        lastPos = mecanumDriveRR.getPoseEstimate();
        currOpMode.sleep(500);
    }

    private void shootPreloadedRings() {

        //Start the two shooting motors
        shootingSystem.operateShooterMotor(0.3);
        ///Give time for motors to speed up
        currOpMode.sleep(2000);

        //Push the first ring
        ringFlipperSystem.pushRing();
        //Wait for 500 ms
        currOpMode.sleep(500);
        //Push the second ring
        ringFlipperSystem.pushRing();
        //Wait for 500 ms
        currOpMode.sleep(500);
        //Push the third ring
        ringFlipperSystem.pushRing();
        //Wait for 500 ms
        currOpMode.sleep(500);
        //Stops the shooting motors
        shootingSystem.stopShooterMotor();
    }

    private void grabRings() {
        intakesystem.startInTake();
        currOpMode.sleep(2000);
        intakesystem.stopInTake();

    }


    private void moveSecondWobbleGoal() {

    }

    private void park() {
        Trajectory traj1 = mecanumDriveRR.trajectoryBuilder(lastPos,0).lineToLinearHeading(RedTeamPositions.PARK_POS).build();
        mecanumDriveRR.followTrajectory(traj1);
        lastPos = mecanumDriveRR.getPoseEstimate();
    }
}
