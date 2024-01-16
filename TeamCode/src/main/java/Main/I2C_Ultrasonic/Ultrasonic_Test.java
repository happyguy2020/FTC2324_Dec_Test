package Main.I2C_Ultrasonic;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "RCWL_9620", group = "Tests")
public class Ultrasonic_Test extends LinearOpMode{

    public RCWL_9620 Ultrasonic_sensor;

    @Override
    public void runOpMode() throws InterruptedException {
        Ultrasonic_sensor = hardwareMap.get(RCWL_9620.class, "Ultrasonic_sensor");

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Reading", Ultrasonic_sensor.getDistance());
            telemetry.addData("Reading", Ultrasonic_sensor.getManufacturer());
            telemetry.addData("Reading", Ultrasonic_sensor.getDeviceName());
            telemetry.update();
        }

    }
}
