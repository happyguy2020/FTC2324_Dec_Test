package Main.Main_JAVA;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

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

import Main.OpenCv.CVMaster;
import Main.OpenCv.OpenCv_HSV_RED;

@Autonomous(name="Robot: December_Draft", group="Robot")
public class Main extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         leftDrive1   = null;
    private DcMotor         leftDrive2   = null;
    private DcMotor         rightDrive1  = null;
    private DcMotor         rightDrive2  = null;

    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftDrive1  = hardwareMap.get(DcMotor.class, "left_drive_1");
        leftDrive2  = hardwareMap.get(DcMotor.class, "left_drive_2");
        rightDrive1 = hardwareMap.get(DcMotor.class, "right_drive1");
        rightDrive2 = hardwareMap.get(DcMotor.class, "right_drive2");

        leftDrive1.setDirection(DcMotor.Direction.REVERSE);
        leftDrive2.setDirection(DcMotor.Direction.REVERSE);
        rightDrive1.setDirection(DcMotor.Direction.FORWARD);
        rightDrive2.setDirection(DcMotor.Direction.FORWARD);

        leftDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                leftDrive1.getCurrentPosition(),
                leftDrive2.getCurrentPosition(),
                rightDrive1.getCurrentPosition(),
                rightDrive2.getCurrentPosition(),
        telemetry.update());

        waitForStart();

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget1;
        int newLeftTarget2;
        int newRightTarget1;
        int newRightTarget2;

        if (opModeIsActive()) {

            // Determine new target position
            newLeftTarget1 = leftDrive1.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLeftTarget2 = leftDrive2.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget1 = rightDrive1.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightTarget2 = rightDrive2.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftDrive1.setTargetPosition(newLeftTarget1);
            leftDrive2.setTargetPosition(newLeftTarget2);
            rightDrive1.setTargetPosition(newRightTarget1);
            rightDrive2.setTargetPosition(newRightTarget2);

            // Turn On RUN_TO_POSITION
            leftDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive1.setPower(Math.abs(speed));
            leftDrive2.setPower(Math.abs(speed));
            rightDrive1.setPower(Math.abs(speed));
            rightDrive2.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDrive1.isBusy() && leftDrive2.isBusy() && rightDrive1.isBusy() && rightDrive2.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget1, newLeftTarget2, newRightTarget1, newRightTarget2);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        leftDrive1.getCurrentPosition(), leftDrive2.getCurrentPosition(), rightDrive1.getCurrentPosition(), rightDrive2.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftDrive1.setPower(0);
            leftDrive2.setPower(0);
            rightDrive1.setPower(0);
            rightDrive2.setPower(0);


            // Turn off RUN_TO_POSITION
            leftDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }



}
