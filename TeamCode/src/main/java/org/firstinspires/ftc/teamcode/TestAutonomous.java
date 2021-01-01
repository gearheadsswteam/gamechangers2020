package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomousRR.AbstractAutonomousOpModeRR;

@Autonomous
public class TestAutonomous extends AbstractAutonomousOpModeRR {

    int ringNum;
    Pose2d initPos = new Pose2d(0, 0, 0);

    protected void initOpModeBeforeStart(){
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
        ringNum = 4;
        if (ringNum == 4) {
            Pose2d lastPos;
            Trajectory traj1 = mecanumDriveRR.trajectoryBuilder(initPos,0).back(24).build();
            mecanumDriveRR.followTrajectory(traj1);
            lastPos = mecanumDriveRR.getPoseEstimate();
            sleep(500);

            Trajectory traj2 = mecanumDriveRR.trajectoryBuilder(lastPos,0).forward(24).build();
            mecanumDriveRR.followTrajectory(traj2);
            lastPos = mecanumDriveRR.getPoseEstimate();
            sleep(500);

            Trajectory traj3 = mecanumDriveRR.trajectoryBuilder(lastPos,0).forward(24).build();
            mecanumDriveRR.followTrajectory(traj3);
            lastPos = mecanumDriveRR.getPoseEstimate();
            sleep(500);
//
//            Trajectory traj4 = drive.trajectoryBuilder(lastPos,0).strafeLeft(24).build();
//            drive.followTrajectory(traj4);
//            lastPos = drive.getPoseEstimate();
//            sleep(500);
//
//            Trajectory traj5 = drive.trajectoryBuilder(lastPos,0).strafeRight(24).build();
//            drive.followTrajectory(traj5);
//            sleep(500);
//
//            drive.turn(Math.toRadians(90));
//            sleep(2500);
//            drive.turn(Math.toRadians(-90));
        }
    }
}