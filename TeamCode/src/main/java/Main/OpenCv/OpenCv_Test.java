package Main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous
public class OpenCv_Test extends OpMode {
    OpenCvCamera camera = null;

    public static double Pivot;
    public static double presentvalue = 100;
    int maxpos = 0;
    double maxval = 0;


    @Override
    public void init() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        OpenCv_pipeline_Test opencv = new OpenCv_pipeline_Test();

        //Camera initialization

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                camera.setPipeline(opencv);
                camera.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            //error method
            public void onError(int errorCode) {

            }
        });

    }
    @Override
    public void loop(){

    }

    public static OpenCvPipeline Observer(){
        //Run detection() function from StickObserverPipeline class

        return null;
    }

}
