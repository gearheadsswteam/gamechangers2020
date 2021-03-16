package org.firstinspires.ftc.teamcode.autonomous.blueteam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomousOpMode;
import org.firstinspires.ftc.teamcode.robot.drivetrain.mecanum.AutonomousMecanumMover;

public class BlueRingCase4AutonomousOpMode {

    private AutonomousMecanumMover autonomousRobotMover;
    private LinearOpMode currOpMode;

    public BlueRingCase4AutonomousOpMode(AutonomousMecanumMover autonomousMecanumMover, LinearOpMode opmode) {
        this.autonomousRobotMover = autonomousMecanumMover;
        currOpMode = opmode;
    }



    public void executeOpMode() {
        moveFirstWobbleGoal();
        gotoShootingPosition();
        shootPreloadedRings();
        grabRings();
        shootPreloadedRings();
        //gotoToSecondWobbleGoal();
        moveSecondWobbleGoal();
        park();
    }

    private void moveFirstWobbleGoal(){
        autonomousRobotMover.rotateLeft(15, 0.1);
        autonomousRobotMover.moveRobotBackwardDistance(0.3,40);
    }

    private void gotoShootingPosition(){
        autonomousRobotMover.moveRobotForwardDistance(0.3,6);
        autonomousRobotMover.rotateRight(15, 0.1);
        autonomousRobotMover.moveRobotLeftUsingPositionEncoders(0.2,9);
        autonomousRobotMover.moveRobotLeftUsingPositionEncoders(0.2,9);
    }

    private void gotoToSecondWobbleGoal(){

    }

    private void shootPreloadedRings(){
        autonomousRobotMover.robot.shootingSystem.shootHighGoals();
        currOpMode.sleep(500);
        autonomousRobotMover.robot.ringFlipperSystem.pushRing();
        currOpMode.sleep(500);
        autonomousRobotMover.robot.ringFlipperSystem.pushRing();
        currOpMode.sleep(500);
        autonomousRobotMover.robot.ringFlipperSystem.pushRing();
        currOpMode.sleep(500);
        autonomousRobotMover.robot.shootingSystem.stopShooterMotor();
    }

    private void grabRings(){
        autonomousRobotMover.moveRobotForwardDistance(0.3,10);
        autonomousRobotMover.robot.intakesystem.startInTake();
        currOpMode.sleep(5000);
        autonomousRobotMover.robot.intakesystem.stopInTake();
        autonomousRobotMover.moveRobotBackwardDistance(0.3,10);
    }



    private void moveSecondWobbleGoal(){

    }

    private void park(){
        autonomousRobotMover.moveRobotBackwardDistance(0.5,6);
    }
}
