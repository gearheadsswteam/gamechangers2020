package org.firstinspires.ftc.teamcode.autonomous.redteam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomousOpMode;

public class RedRingCase0AutonomousOpMode extends AbstractAutonomousOpMode {

    @Override
    protected void initOpModeAfterStart() {

    }

    @Override
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
        autonomousRobotMover.moveRobotLeftDistance(0.2,9);
        autonomousRobotMover.moveRobotLeftDistance(0.2,9);
    }

    private void gotoToSecondWobbleGoal(){

    }

    private void shootPreloadedRings(){
        autonomousRobotMover.robot.shootingSystem.operateShooterMotor(1);
        sleep(500);
        autonomousRobotMover.robot.ringFlipperSystem.pushRing();
        sleep(500);
        autonomousRobotMover.robot.ringFlipperSystem.pushRing();
        sleep(500);
        autonomousRobotMover.robot.ringFlipperSystem.pushRing();
        sleep(500);
        autonomousRobotMover.robot.shootingSystem.stopShooterMotor();
    }

    private void grabRings(){
        autonomousRobotMover.moveRobotForwardDistance(0.3,10);
        autonomousRobotMover.robot.intakesystem.startInTake();
        sleep(5000);
        autonomousRobotMover.robot.intakesystem.stopInTake();
        autonomousRobotMover.moveRobotBackwardDistance(0.3,10);
    }



    private void moveSecondWobbleGoal(){

    }

    private void park(){
        autonomousRobotMover.moveRobotBackwardDistance(0.5,6);
    }
}
