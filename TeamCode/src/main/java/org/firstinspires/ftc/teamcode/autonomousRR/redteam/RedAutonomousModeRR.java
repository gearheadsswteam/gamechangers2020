package org.firstinspires.ftc.teamcode.autonomousRR.redteam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.redteam.RedRingCase0AutonomousOpMode;
import org.firstinspires.ftc.teamcode.autonomousRR.AbstractAutonomousOpModeRR;

@Autonomous
public class RedAutonomousModeRR extends AbstractAutonomousOpModeRR {
    int ringNum;
    Pose2d initPos = new Pose2d(0, 0, 0);

    @Override
    protected void initOpModeBeforeStart() {
        super.initOpModeBeforeStart();
        mecanum.setPoseEstimate(initPos);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    @Override
    protected void initOpModeAfterStart() {

    }

    @Override
    protected void executeOpMode() {
        int rings = robot.ringDetector.detectRings();

        telemetry.addData("Rings ", rings);
        telemetry.update();

        if(rings == 0){
            ///execute Rings = 0 case
            RedRingCase0AutonomousOpModeRR ringCase0AutonomousOpMode = new RedRingCase0AutonomousOpModeRR(mecanum, autonomousRobotMover.robot.shootingSystem, autonomousRobotMover.robot.intakesystem, robot.ringFlipperSystem, this);
            ringCase0AutonomousOpMode.executeOpMode();
        }else if(rings == 1){

        }else if(rings == 4){

        }
    }




    private void gotoShootingPosition() {
        Trajectory traj1 = mecanum.trajectoryBuilder(initPos,0).back(24).build();
        mecanum.followTrajectory(traj1);
        Pose2d lastPos = mecanum.getPoseEstimate();
        sleep(500);
    }


    private void shootPreloadedRings() {

        //Start the two shooting motors
        autonomousRobotMover.robot.shootingSystem.operateShooterMotor(0.3);
        ///Give time for motors to speed up
        sleep(2000);

        //Push the first ring
        autonomousRobotMover.robot.ringFlipperSystem.pushRing();
        //Wait for 500 ms
        sleep(500);
        //Push the second ring
        autonomousRobotMover.robot.ringFlipperSystem.pushRing();
        //Wait for 500 ms
        sleep(500);
        //Push the third ring
        autonomousRobotMover.robot.ringFlipperSystem.pushRing();
        //Wait for 500 ms
        sleep(500);
        //Stops the shooting motors
        autonomousRobotMover.robot.shootingSystem.stopShooterMotor();
    }

    private void grabRings() {

        autonomousRobotMover.robot.intakesystem.startInTake();
        sleep(2000);
        autonomousRobotMover.robot.intakesystem.stopInTake();
    }


    private void moveSecondWobbleGoal() {

    }
}
