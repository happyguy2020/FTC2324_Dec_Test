package Main.I2C_Ultrasonic;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

@SuppressWarnings({"WeakerAccess", "unused"})
@I2cDeviceType
@DeviceProperties(name = "RCWL_9620 Ultrasonic Sensor", description = "Ultrasonic Sensor from Shenzhen Technologies", xmlTag = "RCWL_9620")

public class RCWL_9620 extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    //Registers
    public enum Register
    {
        ACTIVATE(0x01),
        WRITE(0xAE),
        READ(0xAF);

        public int bVal;

        Register(int bVal) {
            this.bVal = bVal;
        }
    }

    //Construction and Initialization

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x57);

    public RCWL_9620(I2cDeviceSynch deviceClient){
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    //Read and write methods
    protected void writeShort(final Register reg, short value){
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected int readShort(Register reg){
        byte[] byteArray;
        byteArray = deviceClient.read(0xAF, 3);

        //define an int array
        int[] intArray = new int[byteArray.length];

        //converting byteArray to intArray
        for (int i = 0; i < byteArray.length; intArray[i] = byteArray[i++]);

        //Name the data in intArray as variables
        int Data_H = intArray[0];
        int Data_M = intArray[1];
        int Data_L = intArray[2];

        //int distance;
        //distance = (((Data_H<<16) + (Data_M<<8) + Data_L)/1000); //Calculated as MM value
        return (byteArray[0]+byteArray[1]+byteArray[2]);
        //return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 3));
    }


    @Override
    public Manufacturer getManufacturer(){

        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize(){
        return true;
    }

    @Override
    public String getDeviceName(){

        return "RCWL_9620 Ultrasonic Sensor";
    }

    //Read Distance Method
    public int getDistance() throws InterruptedException {
        while(true) {
            writeShort(Register.WRITE, (short) 1);
            Thread.sleep(100);

            int distance = readShort(Register.READ);
            Thread.sleep(50);
            return distance;
        }
    }
}
