package Main.OpenCv;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

public class OpenCv_HSV_RED extends OpenCvPipeline{

    public OpenCv_HSV_RED(){}

        Mat HSV = new Mat();
        Mat leftCrop;
        Mat midCrop;
        Mat rightCrop;
        public double leftavgfin;
        public double midavgfin;
        public double rightavgfin;

        int position = 0;
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(255.0,0.0,0.0);
        Scalar boundingRect = new Scalar(60.0, 255, 255);

        @Override
        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input,HSV, Imgproc.COLOR_RGB2HSV);

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

            //Eliminating the suspected values that

            if (leftavgfin > midavgfin && leftavgfin > rightavgfin) {
                position = 1;
                Imgproc.rectangle(outPut, leftRect, boundingRect, -1);
            } else if (midavgfin > rightavgfin && midavgfin > leftavgfin){
                position = 2;
                Imgproc.rectangle(outPut, MidRect, boundingRect, -1);
            } else if (rightavgfin > midavgfin && rightavgfin > leftavgfin){
                position = 3;
                Imgproc.rectangle(outPut, rightRect, boundingRect, -1);

            }

            return(outPut);


        }
    }


