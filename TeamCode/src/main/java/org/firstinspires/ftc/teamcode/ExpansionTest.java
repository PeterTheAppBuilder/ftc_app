package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cIrSeekerSensorV3;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Rev Test", group = "yee")
@Disabled

public class ExpansionTest extends OpMode {

    //we poll three analog devices to simulate a robot having multiple sensors
    //AnalogInput analogSensor1;
    //AnalogInput analogSensor2;
    //AnalogInput analogSensor3;


    //ModernRoboticsI2cColorSensor c;
    //ModernRoboticsI2cRangeSensor r;
    ModernRoboticsI2cIrSeekerSensorV3 i;
    @Override
    public void init() {

        //initialize the analog sensors. To make this work, go into the phone
        //configuration file and name three analog devices to sensor1, sensor2,
        //and sensor3.
        //analogSensor1 = (AnalogInput) hardwareMap.get("sensor1");
        //analogSensor2 = (AnalogInput) hardwareMap.get("sensor2");
        //analogSensor3 = (AnalogInput) hardwareMap.get("sensor3");

        //c = (ModernRoboticsI2cColorSensor) hardwareMap.get("color");

        //r = (ModernRoboticsI2cRangeSensor) hardwareMap.get("range");

        i = (ModernRoboticsI2cIrSeekerSensorV3) hardwareMap.get("ir");

    }
    @Override
    public void init_loop() {

    }


    @Override
    public void start()
    {
        startTime = SystemClock.uptimeMillis();
    }
    long startTime;
    long updates = 0;
    double lastUpdatesPerSecond = 0;

    double lastReadingSensor1 = 0;
    @Override
    public void loop() {



        //read the analog sensor 3 times every update to simulate having multiple sensors
        //double exampleReading = analogSensor1.getVoltage() / analogSensor1.getMaxVoltage();
        //double exampleReading1 = analogSensor2.getVoltage() / analogSensor2.getMaxVoltage();
        //double exampleReading2 = analogSensor3.getVoltage() / analogSensor3.getMaxVoltage();

        //    //increment the number of updates
        //    updates++;
        //}
        //lastReadingSensor1 = exampleReading;

        //int b = c.blue();
        //int r = c.red();
        //int g = c.green();
        //telemetry.addData("blue",b);
        double test = i.getAngle();
        telemetry.addData("ir",test);

        //r.cmOptical();

        telemetry.addData("UPS", (int) lastUpdatesPerSecond);
        telemetry.update();

        updates++;
        //if 1 second has elapsed, display the number of updates per second
        double deltaTime = (SystemClock.uptimeMillis() - startTime) / 1000;
        if (deltaTime > 1) {
            lastUpdatesPerSecond = updates / deltaTime;
            startTime = SystemClock.uptimeMillis();
            updates = 0;

        }
    }
}
