package org.firstinspires.ftc.teamcode.autonomousRR.blueteam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;
import org.firstinspires.ftc.teamcode.robot.actionparts.Intakesystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.RingFlipperSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.ShootingSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.WobblegoalArmLeft;
import org.firstinspires.ftc.teamcode.robot.actionparts.WobblegoalArmRight;

class BlueRingCase1AutonomousOpModeRR {

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

    BlueRingCase1AutonomousOpModeRR(MecanumDriveRR mecanumDriveRR, GearheadsMecanumRobotRR gearheadsMecanumRobotRR, LinearOpMode currOpMode) {
        this.mecanumDriveRR = mecanumDriveRR;
        this.shootingSystem = gearheadsMecanumRobotRR.shootingSystem;
        this.intakesystem = gearheadsMecanumRobotRR.intakesystem;
        this.ringFlipperSystem = gearheadsMecanumRobotRR.ringFlipperSystem;
        this.wobblegoalArmLeft = gearheadsMecanumRobotRR.wobblegoalArmLeft;
        this.wobblegoalArmRight = gearheadsMecanumRobotRR.wobblegoalArmRight;
        this.intakeGaurdServoMotor = gearheadsMecanumRobotRR.intakeGaurdServo;
        this.currOpMode = currOpMode;
    }

    public void setLastPos(Pose2d lastKnownPos) {
        this.initPos = lastKnownPos;
    }

    public void executeOpMode() {
        //Clear the ring set up position
        Trajectory traj1 = mecanumDriveRR.trajectoryBuilder(initPos, 0)
                .splineTo(new Vector2d(-24, 56), 0)
                .splineTo(new Vector2d(3.76 + 24, 62.76 - 24 + 8), -0.2)
                .build();


        //From Case 1 drop zone to shooting position
        Trajectory traj2 = mecanumDriveRR.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(BlueTeamPositions.SHOOTING_POS_CASE_1)
                .build();


        shootingSystem.shootHighGoals();
        ringFlipperSystem.resetPosition();
        intakeGaurdServoMotor.setPosition(0.3);//down


        mecanumDriveRR.followTrajectory(traj1);
        mecanumDriveRR.followTrajectory(traj2);


        ringFlipperSystem.pushRing();
        currOpMode.sleep(500);
        ringFlipperSystem.pushRing();
        currOpMode.sleep(500);
        ringFlipperSystem.pushRing();

        //Go grab and shoot the 1 ring from the fllor
        Trajectory shootRingsTraj = grabAndShootRings(traj2);

        /**
         * Use dropSecondWobbleGoal if you want to grab WG 2 and park
         *
         * Use park if you want to park after shooting all rings without dropping WG 2
         */

        //Go from Shooring position to WG 3 drop point & park
        //dropSecondWobbleGoal(shootRingsTraj);//If we want to drop second wobble
        park(shootRingsTraj);
    }

    /**
     * Grab ring from the mat and shoot
     *
     * @param shootingPosition
     * @return
     */
    private Trajectory grabAndShootRings(Trajectory shootingPosition) {
        //Start the intake
        intakesystem.startInTake();

        //Go to the Ring stack and hit it
        Trajectory trajForRingStack1 = mecanumDriveRR.trajectoryBuilder(shootingPosition.end())
                .back(38).build();
        mecanumDriveRR.followTrajectory(trajForRingStack1);
        currOpMode.sleep(500);


        //Move forward to shooting position
        Pose2d shootingPositionToGoTo = shootingPosition.end();
        Trajectory trajForRingStack3 = mecanumDriveRR.trajectoryBuilder(trajForRingStack1.end())
                .lineToLinearHeading(BlueTeamPositions.SHOOTING_POS_CASE_1).build();
        mecanumDriveRR.followTrajectory(trajForRingStack3);

        ringFlipperSystem.pushRing();
        currOpMode.sleep(500);
        ringFlipperSystem.pushRing();
        currOpMode.sleep(500);
        ringFlipperSystem.pushRing();

        return trajForRingStack3;
    }

    /**
     * Drops the second wobel goal by stating from shooting position
     *
     * @param shootingPsotion
     */
    private void dropSecondWobbleGoal(Trajectory shootingPsotion) {
        //Shooting position to wobble goal 2 grab position
        Trajectory traj3 = mecanumDriveRR.trajectoryBuilder(shootingPsotion.end(), 0)
                .splineTo(new Vector2d(-39.6, -9.1), -3.3)
                .splineTo(new Vector2d(-57.52 + 6, 7.70 - 3), -5.0)
                .build();


        //To slow down robot, from Wobble goal 2 catch position to Case 1 drop position
        TrajectoryConstraints slowConstraints = new MecanumConstraints(DriveConstants.SLOW_ROBOT_CONSTRAINTS, DriveConstants.TRACK_WIDTH);
        Trajectory traj4 = mecanumDriveRR.trajectoryBuilder(traj3.end())
                .lineTo(new Vector2d(-51.3, 17.56), slowConstraints)
                .splineTo(new Vector2d(4.97 + 21 + 3, 59.69 - 24 + 12 - 2 - 3), 0, slowConstraints).build();


        shootingSystem.stopShooterMotor();
        mecanumDriveRR.followTrajectory(traj3);
        currOpMode.sleep(1000);
        mecanumDriveRR.followTrajectory(traj4);

        Trajectory traj5 = mecanumDriveRR.trajectoryBuilder(traj4.end())
                .back(22).build();
        mecanumDriveRR.followTrajectory(traj5);
    }


    /**
     * Park from shooring position
     *
     * @param shootingPosition
     */
    private void park(Trajectory shootingPosition) {
        //Go to the Ring stack and hit it
        Trajectory trajForRingStack1 = mecanumDriveRR.trajectoryBuilder(shootingPosition.end())
                .forward(20).build();
        mecanumDriveRR.followTrajectory(trajForRingStack1);
        currOpMode.sleep(500);
    }

}
