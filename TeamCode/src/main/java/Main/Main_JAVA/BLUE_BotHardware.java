package Main.Main_JAVA;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
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

public class BLUE_BotHardware {

    private LinearOpMode myOpMode = null;

    DcMotor frontLeft = null;
    DcMotor frontRight = null;
    DcMotor backLeft = null;
    DcMotor backRight = null;
    DcMotor sliderLeft = null;
    DcMotor sliderRight = null;
    DcMotor intake = null;
    //DcMotor horizontalSlider = null;

    Servo clawL = null;
    Servo intakeArmL = null;
    Servo intakeArmR = null;
    Servo scoring = null;
    Servo scoringTurn = null;
    Servo scoringFlipL = null;
    Servo scoringFlipR = null;
    Servo droneLauncher = null;
    Servo intakeWheel = null;
    Rev2mDistanceSensor scoringLeft = null;
    Rev2mDistanceSensor scoringRight = null;

    Servo clawR = null;
    IMU imu = null;
    /*RevColorSensorV3 scoringColorLeft = null;
    RevColorSensorV3 scoringColorRight = null;
    RevColorSensorV3 clawColorLeft = null;
    RevColorSensorV3 clawColorRight = null;*/
    public static final boolean USE_WEBCAM = true;
    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTag;

    public static int teamprop_position = 0; //0 = left, 1=center, 2=right



    public BLUE_BotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }


    public void init() {

        frontLeft = myOpMode.hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = myOpMode.hardwareMap.get(DcMotor.class, "frontRight");
        backRight = myOpMode.hardwareMap.get(DcMotor.class, "backRight");
        backLeft = myOpMode.hardwareMap.get(DcMotor.class, "backLeft");
        sliderLeft = myOpMode.hardwareMap.get(DcMotor.class, "LVert");
        sliderRight = myOpMode.hardwareMap.get(DcMotor.class, "RVert");
        //horizontalSlider = myOpMode.hardwareMap.get(DcMotor.class, "horizontalSlider");
        intake = myOpMode.hardwareMap.get(DcMotor.class, "Intake");
        WebcamName webcamName = myOpMode.hardwareMap.get(WebcamName.class, "webcam");


        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        sliderRight.setDirection(DcMotor.Direction.REVERSE);

        sliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        sliderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        intakeArmL = myOpMode.hardwareMap.get(Servo.class, "ArmL");
        intakeArmR = myOpMode.hardwareMap.get(Servo.class, "ArmR");
        scoring = myOpMode.hardwareMap.get(Servo.class, "scoring");
        scoringFlipL = myOpMode.hardwareMap.get(Servo.class, "scoringhL");
        scoringFlipR = myOpMode.hardwareMap.get(Servo.class, "scoringhR");
        scoringTurn = myOpMode.hardwareMap.get(Servo.class, "scoringTurn");
        droneLauncher = myOpMode.hardwareMap.get(Servo.class, "drone");
        intakeWheel = myOpMode.hardwareMap.get(Servo.class, "Intakeservo");

        imu=myOpMode.hardwareMap.get(IMU.class, "imu");

        /*scoringColorLeft = myOpMode.hardwareMap.get(RevColorSensorV3.class, "scoringColorLeft");
        scoringColorRight = myOpMode.hardwareMap.get(RevColorSensorV3.class, "scoringColorRight");
        clawColorRight = myOpMode.hardwareMap.get(RevColorSensorV3.class, "clawColorRight");
        clawColorLeft = myOpMode.hardwareMap.get(RevColorSensorV3.class, "clawColorLeft"); */

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        //Initialize camera
        int cameraMonitorViewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id", myOpMode.hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        webcam1.setPipeline(new bluePipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                webcam1.startStreaming(640,360, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {

            }
        });

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }


    public void sliderMoveToPos(int targetPos){
        sliderRight.setTargetPosition(targetPos);
        sliderLeft.setTargetPosition(targetPos);
        sliderRight.setPower(1);
        sliderLeft.setPower(1);
        sliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /*public void horizontalSliderMoveToPos(int targetPos){
        horizontalSlider.setTargetPosition(targetPos);
        horizontalSlider.setPower(1);
        horizontalSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }*/
    public void setupScoring (double flipPos, double turnPos){
        scoringFlipL.setPosition(flipPos);
        scoringFlipR.setPosition(flipPos);
        scoringTurn.setPosition(turnPos);
    }
    public void initAprilTag(){
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);
        visionPortal = new VisionPortal.Builder()
                .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);
    }

    //RED ONLY - OpenCv team prop recognition code
    public class redPipeline extends OpenCvPipeline {
        Mat HSV = new Mat();
        Mat leftCrop;
        Mat midCrop;
        Mat rightCrop;
        double leftavgfin;
        double midavgfin;
        double rightavgfin;
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(255.0,0.0,0.0);
        Scalar boundingRect = new Scalar(60.0, 255, 255);

        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input,HSV, Imgproc.COLOR_RGB2HSV);
            telemetry.addLine("pipeline running");

            Rect leftRect = new Rect(1, 160, 100, 100);
            Rect MidRect = new Rect(250, 160, 100, 100);
            Rect rightRect = new Rect(530, 160, 100, 100);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, rectColor, 2);
            Imgproc.rectangle(outPut, MidRect, rectColor, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor, 2);

            leftCrop = HSV.submat(leftRect);
            midCrop = HSV.submat(MidRect);
            rightCrop = HSV.submat(rightRect);

            //creating coundaries for red
            Scalar lowHSV = new Scalar(0, 80, 70); //lenient lower bound
            Scalar highHSV = new Scalar(10, 255, 255);//lenient higher bound

            //appying red filter
            Core.inRange(leftCrop, lowHSV, highHSV, leftCrop);
            Core.inRange(midCrop, lowHSV, highHSV, midCrop);
            Core.inRange(rightCrop, lowHSV, highHSV, rightCrop);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar midavg = Core.mean(midCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = leftavg.val[0];
            midavgfin = midavg.val[0];
            rightavgfin = rightavg.val[0];

            if (leftavgfin > midavgfin && leftavgfin > rightavgfin) {
                telemetry.addLine("Left");
                Imgproc.rectangle(outPut, leftRect, boundingRect, -1);
                teamprop_position = 0;
            } else if (midavgfin > rightavgfin && midavgfin > leftavgfin){
                telemetry.addLine("Middle");
                Imgproc.rectangle(outPut, MidRect, boundingRect, -1);
                teamprop_position = 1;
            } else if (rightavgfin > midavgfin && rightavgfin > leftavgfin){
                telemetry.addLine("Right");
                Imgproc.rectangle(outPut, rightRect, boundingRect, -1);
                teamprop_position = 2;
            }

            telemetry.addData("Leftavg", leftavg.val[0]);
            telemetry.addData("Midavg", midavg.val[0]);
            telemetry.addData("Rightavg", rightavg.val[0]);
            return(outPut);
        }
    }

    //BLUE ONLY - OpenCv team prop recognition code
    public class bluePipeline extends OpenCvPipeline {
        Mat HSV = new Mat();
        Mat leftCrop;
        Mat midCrop;
        Mat rightCrop;
        double leftavgfin;
        double midavgfin;
        double rightavgfin;
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(255.0,0.0,0.0);
        Scalar boundingRect = new Scalar(60.0, 255, 255);

        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input,HSV, Imgproc.COLOR_RGB2HSV);
            telemetry.addLine("pipeline running");

            Rect leftRect = new Rect(1, 160, 100, 100);
            Rect MidRect = new Rect(250, 160, 100, 100);
            Rect rightRect = new Rect(530, 160, 100, 100);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, rectColor, 2);
            Imgproc.rectangle(outPut, MidRect, rectColor, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor, 2);

            leftCrop = HSV.submat(leftRect);
            midCrop = HSV.submat(MidRect);
            rightCrop = HSV.submat(rightRect);

            //creating coundaries for blue
            Scalar lowHSV = new Scalar(80, 80, 70); //lenient lower bound
            Scalar highHSV = new Scalar(110, 240, 255);

            //appying red filter
            Core.inRange(leftCrop, lowHSV, highHSV, leftCrop);
            Core.inRange(midCrop, lowHSV, highHSV, midCrop);
            Core.inRange(rightCrop, lowHSV, highHSV, rightCrop);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar midavg = Core.mean(midCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = leftavg.val[0];
            midavgfin = midavg.val[0];
            rightavgfin = rightavg.val[0];

            if (leftavgfin > midavgfin && leftavgfin > rightavgfin) {
                telemetry.addLine("Left");
                Imgproc.rectangle(outPut, leftRect, boundingRect, -1);
                teamprop_position = 0;
            } else if (midavgfin > rightavgfin && midavgfin > leftavgfin){
                telemetry.addLine("Middle");
                Imgproc.rectangle(outPut, MidRect, boundingRect, -1);
                teamprop_position = 1;
            } else if (rightavgfin > midavgfin && rightavgfin > leftavgfin){
                telemetry.addLine("Right");
                Imgproc.rectangle(outPut, rightRect, boundingRect, -1);
                teamprop_position = 2;
            }

            telemetry.addData("Leftavg", leftavg.val[0]);
            telemetry.addData("Midavg", midavg.val[0]);
            telemetry.addData("Rightavg", rightavg.val[0]);
            return(outPut);
        }
    }


}
