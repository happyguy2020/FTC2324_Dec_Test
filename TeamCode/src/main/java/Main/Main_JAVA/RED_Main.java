package Main.Main_JAVA;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import Main.Drive_controls.SampleMecanumDrive;
import Main.trajectorysequence.TrajectorySequence;

@Autonomous
public class RED_Main extends LinearOpMode {

    OpenCvWebcam webcam1 = null;

    Pose2d startPose = new Pose2d(0,0,0);
    Pose2d step2Pose = new Pose2d();
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    Trajectory teampropLeft = drive.trajectoryBuilder(startPose)
            .lineToLinearHeading(new Pose2d(0,28,Math.toRadians(90)))
            .build();

    Trajectory teampropCenter = drive.trajectoryBuilder(startPose)
            .lineToLinearHeading(new Pose2d(0,41,Math.toRadians(180)))
            .build();

    Trajectory teampropRight = drive.trajectoryBuilder(startPose)
            .lineToLinearHeading(new Pose2d(0,28,Math.toRadians(-90)))
            .build();

    Trajectory toStack = drive.trajectoryBuilder(step2Pose)
            .lineToLinearHeading(new Pose2d(24.5,52,Math.toRadians(90)))
            .build();

    TrajectorySequence toBackdrop = drive.trajectorySequenceBuilder(new Pose2d(24.5,52,Math.toRadians(90)))
            .lineToConstantHeading(new Vector2d(-61.25,52))
            .addTemporalMarker(0, ()->{

            })
            .build();

    //call Trajectories
    public void runOpMode(){

        init();
        waitForStart();

        int prop_position = Main.Main_JAVA.RED_BotHardware.teamprop_position;

        ///\\if (isStopRequested()) return;
        while (opModeIsActive()) {
            if (prop_position == 0){
                drive.followTrajectory(teampropLeft);
            }
            if (prop_position == 1){
                drive.followTrajectory(teampropCenter);
            }
            if (prop_position ==2){
                drive.followTrajectory(teampropRight);
            }

            drive.followTrajectory(toStack);

            sleep(1000);

            sleep(1000);

            drive.followTrajectorySequence(toBackdrop);
        }
    }
}


