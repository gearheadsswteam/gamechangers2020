package org.firstinspires.ftc.teamcode.autonomousRR.redteam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.robot.actionparts.Intakesystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.RingFlipperSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.ShootingSystem;
import org.firstinspires.ftc.teamcode.robot.drivetrain.mecanum.AutonomousMecanumMover;

public class RedRingCase0AutonomousOpModeRR {

    private MecanumDriveRR mecanumDriveRR;
    private ShootingSystem shootingSystem;
    private RingFlipperSystem ringFlipperSystem;
    private Intakesystem intakesystem;
    private LinearOpMode currOpMode;
    private Pose2d initPos;
    private Pose2d lastPos;

    public RedRingCase0AutonomousOpModeRR(MecanumDriveRR mecanumDriveRR, ShootingSystem shootingSystem, Intakesystem intakesystem, RingFlipperSystem ringFlipperSystem, LinearOpMode currOpMode) {
        this.mecanumDriveRR = mecanumDriveRR;
        this.shootingSystem = shootingSystem;
        this.intakesystem = intakesystem;
        this.ringFlipperSystem = ringFlipperSystem;
        this.currOpMode = currOpMode;
        initPos = new Pose2d();
    }

    public void executeOpMode() {
        //Move the first wobble goal to the correct position
        moveFirstWobbleGoal();

        //Move from Wobble goal drop point to shooting position
        gotoShootingPosition();

        //Shoot the preloaded rings in the goal
        shootPreloadedRings();

        grabRings();

        shootPreloadedRings();

        //Park the robot
        park();
    }


    /**
     * Move the first wobble goal to the correct position
     */
    private void moveFirstWobbleGoal() {
        Trajectory traj1 = mecanumDriveRR.trajectoryBuilder(initPos,0).back(24).build();
        mecanumDriveRR.followTrajectory(traj1);
        lastPos = mecanumDriveRR.getPoseEstimate();
        currOpMode.sleep(500);
    }

    private void gotoShootingPosition() {
        Trajectory traj1 = mecanumDriveRR.trajectoryBuilder(lastPos,0).strafeLeft(12).build();
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
        Trajectory traj1 = mecanumDriveRR.trajectoryBuilder(lastPos,0).forward(12).build();
        mecanumDriveRR.followTrajectory(traj1);
        lastPos = mecanumDriveRR.getPoseEstimate();
        currOpMode.sleep(500);

        intakesystem.startInTake();
        currOpMode.sleep(2000);
        intakesystem.stopInTake();

    }


    private void moveSecondWobbleGoal() {

    }

    private void park() {
        Trajectory traj1 = mecanumDriveRR.trajectoryBuilder(lastPos,0).back(12).build();
        mecanumDriveRR.followTrajectory(traj1);
        lastPos = mecanumDriveRR.getPoseEstimate();
    }
}
