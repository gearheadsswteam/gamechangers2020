package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomousOpModeRR;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveRR;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import static java.lang.Math.*;
@Autonomous
public class TestAutonomous extends AbstractAutonomousOpModeRR {

    int ringNum;
    Pose2d initPos = new Pose2d(0, 0, 0);


    protected void initOpModeBeforeStart(){
        mecanum = new MecanumDriveRR(hardwareMap);
        mecanum.setPoseEstimate(initPos);

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
            Trajectory traj1 = mecanum.trajectoryBuilder(initPos,0).back(24).build();
            mecanum.followTrajectory(traj1);
            lastPos = mecanum.getPoseEstimate();
            sleep(500);

            Trajectory traj2 = mecanum.trajectoryBuilder(lastPos,0).forward(24).build();
            mecanum.followTrajectory(traj2);
            lastPos = mecanum.getPoseEstimate();
            sleep(500);

            Trajectory traj3 = mecanum.trajectoryBuilder(lastPos,0).forward(24).build();
            mecanum.followTrajectory(traj3);
            lastPos = mecanum.getPoseEstimate();
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