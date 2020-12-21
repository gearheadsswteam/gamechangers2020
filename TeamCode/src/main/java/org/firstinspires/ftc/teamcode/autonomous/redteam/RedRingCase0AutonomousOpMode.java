package org.firstinspires.ftc.teamcode.autonomous.redteam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomousOpMode;
import org.firstinspires.ftc.teamcode.robot.drivetrain.mecanum.AutonomousMecanumMover;

public class RedRingCase0AutonomousOpMode {

    private AutonomousMecanumMover autonomousRobotMover;
    private LinearOpMode currOpMode;

    public RedRingCase0AutonomousOpMode(AutonomousMecanumMover autonomousMecanumMover, LinearOpMode opmode) {
        this.autonomousRobotMover = autonomousMecanumMover;
        currOpMode = opmode;
    }

    public void executeOpMode() {
        //operateArm();

        //Move the first wobble goal to the correct position
        moveFirstWobbleGoal();

        //Move from Wobble goal drop point to shooting position
        gotoShootingPosition();

        //Shoot the preloaded rings in the goal
       shootPreloadedRings();

        //Grab the rings from the floor
        //grabRings();

        //Shoot the rings
        //shootPreloadedRings();

        //Go to the second wobble goal position
        //gotoToSecondWobbleGoal();

        //Move the second wobble goal
        //moveSecondWobbleGoal();

        //Park the robot
        park();
    }

    private void operateArm(){
        autonomousRobotMover.robot.wobblegoalArmRight.grabWobbleGoal();
        currOpMode.sleep(500);
        autonomousRobotMover.robot.wobblegoalArmRight.setWobbleGoal();
        currOpMode.sleep(500);
        autonomousRobotMover.robot.wobblegoalArmRight.ungrabWobbleGoal();
        currOpMode.sleep(500);
    }

    /**
     * Move the first wobble goal to the correct position
     */
    private void moveFirstWobbleGoal(){

        //Rotate robot so that it faces the wobble goal parking postion
        autonomousRobotMover.rotateLeft(40, 0.1);
        //Move towards the Wobble goal position
        autonomousRobotMover.moveRobotBackwardDistance(0.3,35);
    }

    private void gotoShootingPosition(){

        //Move the robot away from wobble goal drop point
        //autonomousRobotMover.moveRobotForwardDistance(0.3,6);

        //Rotate right so that the robot is straight ...facing the goal post
        autonomousRobotMover.rotateLeft(4, 0.1);//was 2

        //Move or strafe left to get to shooting position
        //autonomousRobotMover.moveRobotLeftDistance(0.2,3);//was 8
        autonomousRobotMover.moveRobotForwardDistance(0.2,6);//was 8

    }

    private void gotoToSecondWobbleGoal(){

    }

    private void shootPreloadedRings(){

        //Start the two shooting motors
        autonomousRobotMover.robot.shootingSystem.operateShooterMotor(1);
        ///Give time for motors to speed up
        currOpMode.sleep(2000);

        //Push the first ring
        autonomousRobotMover.robot.ringFlipperSystem.pushRing();
        //Wait for 500 ms
        currOpMode.sleep(500);
        //Push the second ring
        autonomousRobotMover.robot.ringFlipperSystem.pushRing();
        //Wait for 500 ms
        currOpMode.sleep(500);
        //Push the third ring
        autonomousRobotMover.robot.ringFlipperSystem.pushRing();
        //Wait for 500 ms
        currOpMode.sleep(500);
        //Stops the shooting motors
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
