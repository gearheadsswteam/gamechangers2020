package org.firstinspires.ftc.teamcode.autonomous.redteam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomousOpMode;
import org.firstinspires.ftc.teamcode.autonomous.blueteam.BlueRingCase0AutonomousOpMode;
import org.firstinspires.ftc.teamcode.autonomous.blueteam.BlueRingCase1AutonomousOpMode;
import org.firstinspires.ftc.teamcode.autonomous.blueteam.BlueRingCase4AutonomousOpMode;

@Autonomous(name = "RedAutonomousMode", group = "Mecannum")

public class RedAutonomousMode extends AbstractAutonomousOpMode {
    @Override
    protected void initOpModeAfterStart() {

    }

    @Override
    protected void executeOpMode() {
        //Grab the wobble goal for transport
        grabWobbleGoal();

        //Move closer to rings so that we can detect them
        autonomousRobotMover.moveRobotBackwardDistance(0.3,4.5);
        this.sleep(1500);

        int rings = robot.ringDetector.detectRings();
        telemetry.addData("Rings ", rings);
        telemetry.update();

        if(rings == 0){
            ///execute Rings = 0 case
            RedRingCase0AutonomousOpMode ringCase0AutonomousOpMode = new RedRingCase0AutonomousOpMode(autonomousRobotMover, this);
            ringCase0AutonomousOpMode.executeOpMode();
        }else if(rings == 1){
            ///execute Rings = 1 case
            RedRingCase0AutonomousOpMode ringCase0AutonomousOpMode = new RedRingCase0AutonomousOpMode(autonomousRobotMover, this);
            ringCase0AutonomousOpMode.executeOpMode();
        }else if(rings == 4){
            ///execute Rings = 4 case
            RedRingCase0AutonomousOpMode ringCase0AutonomousOpMode = new RedRingCase0AutonomousOpMode(autonomousRobotMover, this);
            ringCase0AutonomousOpMode.executeOpMode();
        }
    }

    /**
     * Grabs the wobble goal tight
     */
    private void grabWobbleGoal() {
        autonomousRobotMover.robot.wobblegoalArmRight.grabWobbleGoal();
        this.sleep(500);
    }
}
