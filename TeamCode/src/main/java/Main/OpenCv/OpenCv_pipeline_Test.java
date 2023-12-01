package Main;

import static android.graphics.Color.GREEN;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class OpenCv_pipeline_Test extends OpenCvPipeline {

    Mat output = new Mat();
    Mat HSV = new Mat();
    Mat leftcrop;
    Mat midcrop;
    Mat rightcrop;

    double leftavgfin;
    double midavgfin;
    double rightavgfin;

    public enum Teamprop_Position{
        LEFT,
        MID,
        RIGHT
    }

    public Teamprop_Position position;


    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV); //mat turns into HSV Value
        if (output.empty()) {
            return input;
        }
        telemetry.addData("pipeline running", " ");

        //draw rectangles
        Rect leftRect = new Rect(1, 200, 40, 50);
        Rect midRect = new Rect(320,1 , 40, 50);
        Rect rightRect = new Rect(600,200 , 40, 50);

        Scalar Rect_colour = new Scalar(255.0,0.0,0.0);

        //display rectangles in output
        input.copyTo(output);
        Imgproc.rectangle(output, leftRect, Rect_colour, 2);
        Imgproc.rectangle(output, midRect, Rect_colour, 2);
        Imgproc.rectangle(output, rightRect, Rect_colour, 2);

        //defining submats
        leftcrop = HSV.submat(leftRect);
        midcrop = HSV.submat(midRect);
        rightcrop = HSV.submat(rightRect);

        //creating coundaries for red
        Scalar lowHSV = new Scalar(0, 80, 70); //lenient lower bound
        Scalar highHSV = new Scalar(10, 255, 255);//lenient higher bound

        //appying red filter
        Core.inRange(leftcrop, lowHSV, highHSV, leftcrop);
        Core.inRange(midcrop, lowHSV, highHSV, midcrop);
        Core.inRange(rightcrop, lowHSV, highHSV, rightcrop);

        //Average of the red colour
        /*Scalar leftavg = Core.mean(leftcrop);
        Scalar midavg = Core.mean(midcrop);
        Scalar rightavg = Core.mean(rightcrop);

        //Storing the values in an array
        leftavgfin = leftavg.val[0];
        midavgfin = midavg.val[0];
        rightavgfin = rightavg.val[0];

        //Comparing the Values
        double maxOneTwo = Math.max(leftavgfin, midavgfin);
        double max = Math.max(maxOneTwo, rightavgfin);

        if(max == leftavgfin){
            position = Teamprop_Position.LEFT;
            Imgproc.rectangle(
                    output, // Buffer to draw on
                    LEFTRECT_pointA, // First point which defines the rectangle
                    LEFTRECT_pointB, // Second point which defines the rectangle
                    Scalar.all(GREEN), // The color the rectangle is drawn in
                    2);
            telemetry.addData("Left", " ");
        }
        else if(max == midavgfin){
            position = Teamprop_Position.MID;
            Imgproc.rectangle(
                    output, // Buffer to draw on
                    MIDRECT_pointA, // First point which defines the rectangle
                    MIDRECT_pointB, // Second point which defines the rectangle
                    Scalar.all(GREEN), // The color the rectangle is drawn in
                    2);
            telemetry.addData("Middle", " ");
        }
        else if(max == rightavgfin){
            position = Teamprop_Position.RIGHT;
            Imgproc.rectangle(
                    output, // Buffer to draw on
                    RIGHTRECT_pointA, // First point which defines the rectangle
                    MIDRECT_pointB, // Second point which defines the rectangle
                    Scalar.all(GREEN), // The color the rectangle is drawn in
                    2);
            telemetry.addData("right", " ");
        }

        //update telemetry
        /*if(position == Teamprop_Position.LEFT){
            telemetry.addLine("Left");
        }
        else if(position == Teamprop_Position.MID){
            telemetry.addLine("Mid");
        }
        else if(position == Teamprop_Position.RIGHT){
            telemetry.addLine("Right");
        }*/

        //release data
        output.release();
        return (output);
    }
}
