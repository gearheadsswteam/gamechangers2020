package org.firstinspires.ftc.teamcode.autonomousRR.redteam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomousRR.AbstractAutonomousOpModeRR;
import org.firstinspires.ftc.teamcode.autonomousRR.RedTeamPositions;

@Autonomous
public class RedAutonomousModeRR extends AbstractAutonomousOpModeRR {
    int ringNum;
    Pose2d initPos = new Pose2d(0, 0, 0);

    @Override
    protected void initOpModeBeforeStart() {
        super.initOpModeBeforeStart();
        mecanumDriveRR.setPoseEstimate(initPos);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    @Override
    protected void initOpModeAfterStart() {

    }

    @Override
    protected void executeOpMode() {
        //Grab the wobble goal for transport
        grabWobbleGoal();
        Pose2d lastPos = goToRingDetectionPosition();

        int rings = robot.ringDetector.detectRings();

        telemetry.addData("Rings ", rings);
        telemetry.update();

        if(rings == 0){
            ///execute Rings = 0 case
            RedRingCase0AutonomousOpModeRR ringCase0AutonomousOpMode = new RedRingCase0AutonomousOpModeRR(mecanumDriveRR, autonomousRobotMover.robot,this);
            ringCase0AutonomousOpMode.setLastPos(lastPos);
            ringCase0AutonomousOpMode.executeOpMode();
        }else if(rings == 1){

        }else if(rings == 4){

        }
    }

    /**
     * Grabs the wobble goal tight
     */
    private void grabWobbleGoal() {
        autonomousRobotMover.robot.wobblegoalArmRight.grabWobbleGoal();
        this.sleep(500);
    }

    private Pose2d goToRingDetectionPosition(){
        Trajectory trajectory = mecanumDriveRR.trajectoryBuilder(RedTeamPositions.POS1, 0).splineToSplineHeading(RedTeamPositions.RING_DETECTION_POS,0).build();
        mecanumDriveRR.followTrajectory(trajectory);
        return mecanumDriveRR.getPoseEstimate();
    }
}
