/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.text.DecimalFormat;

import RobotUtilities.Tracker;
import ftc.vision.FrameGrabber;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "MyTeleOp3", group = "MyTeleOp4")
public class Main extends OpMode {
    public static final double shooter_max_speed = 0.62;
    public static final int All_telemetry_trackers_pollingRate = 0;
    /////////////////////////////////VARS////////////////////////////
    public final float turnScale = 0.6f;
    public final double moveScalingFactor = 18.2667;//when you divide the encoder values by this it returns cm
    public double worldXPosition=0;
    public double worldYPosition=0;
    public double worldAngle_rad = 0;//start the world angle at 0 degrees
    public double wheelLeftLast = 0;
    public double wheelRightLast = 0;
    public double wheelAuxLast = 0;
    public double fastestSpeed = 0;
    public double targetAngle = Math.toRadians(90.0f);
    DecimalFormat df = new DecimalFormat("#.00");
    BNO055IMU imu;
    DcMotor motorTL;
    DcMotor motorTR;
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motor_shooter;
    DcMotor motor_collector;
    //////////////////////////////////////////////////////////////////
    Servo door_servo;
    //right Tracker
    AnalogInput E_tracker_r;
    Tracker tracker_r;
    //left Tracker
    AnalogInput E_tracker_l;
    Tracker tracker_l;
    //third, sideways Tracker
    AnalogInput E_tracker_a;
    Tracker tracker_a;
    //store how different aspects of our movement in vars so that
    //we don't set the power of a motor twice in an update. Change
    //these vars when applying power.
    double movement_y = 0;
    double movement_x = 0;
    double movement_turn = 0;
    boolean shooter_master = false;
    boolean shooter_reving = false;
    long shooter_rev_start_time = 0;
    double shooter_current_speed;
    BallFindingAuto.trigStates triggerStage = BallFindingAuto.trigStates.secureDown;
    double door_servo_current_pos = 0;
    boolean fire = false;
    long fireStartTime = 0;
    long secureDown_start_time = 0;
    double GyroAngle = 0;
    Orientation angles;
    long programStartTime = 0;
    long startTime;
    long longestUpdate = 0;
    long lastUpdate = 0;
    double lastUpdatesPerSecond = 0;
    boolean yPressed = false;
    boolean collectorActivated = false;
    long All_telemetry_trackers_lastUpdate =0;
  private ElapsedTime runtime = new ElapsedTime();

    public void activateShooter(){
        shooter_master = true;
        //if the last update we were not reving up, we know that this
        //is the first initialization update of the reving sequence
        if(!shooter_reving){
            shooter_reving = true;
            shooter_rev_start_time = SystemClock.uptimeMillis();
        }
    }

    public void turnOff_shooter(){
        shooter_master = false;
    }

    public void shooter(){
        //////Aux controls/////
        if(gamepad1.b){
            activateShooter();
        }
        if(gamepad1.a){
            turnOff_shooter();
        }







        ///////////////////////////Acceleration and Decelerations (not a thing)////////////
        if(shooter_reving){
            double speedPercent = (SystemClock.uptimeMillis()-shooter_rev_start_time);
            if(speedPercent>=1){
                shooter_reving = false;
            }
            shooter_current_speed = speedPercent*shooter_max_speed;
        }
        if(!shooter_reving && shooter_master){
            shooter_current_speed = shooter_max_speed;
        }
        if(!shooter_master){
            shooter_reving = false;
            shooter_current_speed = 0;
        }
        motor_shooter.setPower(shooter_current_speed);
    }

    public void fire(){
        //if during the last update, we were not firing, initialize
        if(!fire){
            fireStartTime = SystemClock.uptimeMillis();
        }
        fire = true;
        triggerStage = BallFindingAuto.trigStates.engaged;
    }

    public void stopFiring(){
        fire = false;
    }

    public void trigger(){
        telemetry.addData("triggerStage: ", triggerStage);
        if(gamepad1.x){
            fire();
        }else{
            stopFiring();
        }
        if(triggerStage == BallFindingAuto.trigStates.engaged){
            door_servo_current_pos = 1;
            if(SystemClock.uptimeMillis()-fireStartTime >= 1000 && !fire){
                triggerStage = BallFindingAuto.trigStates.secureDown;
                secureDown_start_time = SystemClock.uptimeMillis();
            }
        }


        if(triggerStage == BallFindingAuto.trigStates.secureDown){
            door_servo_current_pos = 0;
            if(SystemClock.uptimeMillis()-secureDown_start_time > 800){
                triggerStage = BallFindingAuto.trigStates.stopped;
            }
        }
        if(triggerStage == BallFindingAuto.trigStates.stopped){
            door_servo_current_pos = 0.5;
        }

        door_servo.setPosition(door_servo_current_pos);
    }

    @Override
    public void init() {
        this.msStuckDetectInit = 10000;

        initializeIMU();


        motorTL = hardwareMap.dcMotor.get("motorTL");
        motorTR = hardwareMap.dcMotor.get("motorTR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        motor_shooter = hardwareMap.dcMotor.get("shooter");
        motor_shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_collector = hardwareMap.dcMotor.get("collector");
        motor_collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_collector.setDirection(DcMotor.Direction.REVERSE);
        door_servo = hardwareMap.servo.get("door");


        motorTL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorTR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorTR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);




    //initialize our telemetry trackers
    E_tracker_r = (AnalogInput) hardwareMap.get("tracker_r");
    tracker_r = new Tracker(E_tracker_r);
    E_tracker_l = (AnalogInput) hardwareMap.get("tracker_l");
    tracker_l = new Tracker(E_tracker_l);
    E_tracker_a = (AnalogInput) hardwareMap.get("tracker_a");
    tracker_a = new Tracker(E_tracker_a);





      telemetry.setMsTransmissionInterval(10);


  }

  @Override
  public void init_loop() {

  }

    @Override
    public void start() {
        runtime.reset();
        programStartTime = SystemClock.uptimeMillis();


        //////////reset encoders///////////////
        tracker_l.reset();
        tracker_r.reset();
        tracker_a.reset();
        wheelLeftLast = tracker_l.getPos();
        wheelRightLast= -tracker_r.getPos();
        ///////////////////////////////////////


        worldXPosition = 0;
        worldYPosition = 0;
        worldAngle_rad = Math.toRadians(0);
        targetAngle = Math.toRadians(0);

        startTime = SystemClock.uptimeMillis();
        DbgLog.msg("ProgramStart");
    }

    public void readIMU(){
        telemetry.clearAll();

        // Acquiring the angles is relatively expensive; we don't want
        // to do that in each of the three items that need that info, as that's
        // three times the necessary expense.
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        if(angles != null){
            GyroAngle = angles.firstAngle;

        }

        telemetry.addData("Gyro Angle:", GyroAngle);


    }

    @Override
    public void loop() {
        readIMU();
        DbgLog.msg("rot*,"+(SystemClock.uptimeMillis()-programStartTime)+","+GyroAngle+","+Math.toDegrees(worldAngle_rad));

        lastUpdate = SystemClock.uptimeMillis();
        MovementCalc();
        ApplyMovement();
        ReadEncodersOT();
        PositioningCalculations();
        shooter();
        trigger();
        collector();

    }

    public void collector(){
        if(gamepad1.y){
            if(!yPressed){
                collectorActivated = !collectorActivated;
            }
            yPressed = true;
        }else{
            yPressed = false;
        }

        if(collectorActivated){
            motor_collector.setPower(1);
        }else{
            motor_collector.setPower(0);
        }

    }

    public void  initializeIMU(){

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public void PositioningCalculations(){





        double wheelLeftCurrent = tracker_l.getPos();
        double wheelRightCurrent= -tracker_r.getPos();
        double wheelAuxCurrent = tracker_a.getPos();
        if(Math.abs(wheelLeftCurrent-wheelLeftLast) > fastestSpeed){
            fastestSpeed = Math.abs(wheelLeftCurrent-wheelLeftLast);
        }
        telemetry.addData("fastest Speed",df.format(fastestSpeed));

        double wheelLeftDelta = (wheelLeftCurrent - wheelLeftLast)*moveScalingFactor;
        double wheelRightDelta = (wheelRightCurrent - wheelRightLast)*moveScalingFactor;
        double wheelAuxDelta = (wheelAuxCurrent - wheelAuxLast)*moveScalingFactor;


        double robotWidth = 35.5155;//this is effectively the turning scale factor

        double Angleincrement = (wheelRightDelta-wheelLeftDelta)/robotWidth;
        worldAngle_rad += Angleincrement;
        worldAngle_rad = AngleWrap(worldAngle_rad);

        //relative y translation
        double r_yDistance = (wheelRightDelta+wheelLeftDelta)/2;

        double tracker_a_prediction = Angleincrement/12.86;
        double r_xDistance = wheelAuxDelta-tracker_a_prediction;







        worldXPosition += (Math.cos(worldAngle_rad) * r_yDistance) + (Math.sin(worldAngle_rad) * r_xDistance);
        worldYPosition += (Math.sin(worldAngle_rad) * r_yDistance) - (Math.cos(worldAngle_rad) * r_xDistance);



        wheelLeftLast = wheelLeftCurrent;
        wheelRightLast = wheelRightCurrent;
        wheelAuxLast = wheelAuxCurrent;

        telemetry.addLine()
                .addData("XPos:",df.format(worldXPosition))
                .addData("YPos:",df.format(worldYPosition))
                .addData("Angle:",df.format(Math.toDegrees(worldAngle_rad)));


        FrameGrabber.x = worldXPosition;
        FrameGrabber.y = worldYPosition;





    }

    public double AngleWrap(double angle){
        while (angle<-Math.PI){
            angle += 2*Math.PI;
        }
        while (angle>Math.PI){
            angle -= 2*Math.PI;
        }
        return angle;
    }

    public void ReadEncodersOT(){
        ////////////////////////////PositionTrackers///////////////////////////////
        if(SystemClock.uptimeMillis()- All_telemetry_trackers_lastUpdate > All_telemetry_trackers_pollingRate){
            All_telemetry_trackers_lastUpdate = SystemClock.uptimeMillis();
            tracker_l.update();
            tracker_r.update();
            tracker_a.update();
        }
        telemetry.addData("tracker_r:", df.format(tracker_r.getPos()))
                .addData("tracker_l:", df.format(tracker_l.getPos()))
                .addData("tracker_a",df.format(tracker_a.getPos()));

    }

  public void MovementCalc(){

        movement_turn = gamepad1.left_stick_x;
        if(movement_turn>0){
            movement_turn = (float) Math.pow(Math.abs(movement_turn),1.5);
        }else{
            movement_turn = (float) -Math.pow(Math.abs(movement_turn),1.5);
        }
        movement_turn *=turnScale;

        movement_y = -gamepad1.right_stick_y;
        movement_x = gamepad1.right_stick_x;


        //////Absolute controls/////////////////

        /*
        double movementAngle = Math.atan2(gamepad1.right_stick_y,-gamepad1.right_stick_x);
        double relativeAngle = AngleWrap(movementAngle-worldAngle_rad);
        double amplitude = Math.sqrt(Math.pow(gamepad1.right_stick_x,2)+Math.pow(gamepad1.right_stick_y,2));
        movement_y = (Math.cos(relativeAngle)) * amplitude;
        movement_x = (Math.sin(relativeAngle)) * amplitude;

        //only update the target angle if the gamepad is 3% away from the center
        if(Math.sqrt(Math.pow(gamepad1.left_stick_x,2)+Math.pow(gamepad1.left_stick_y,2))>0.03){
            targetAngle = Math.atan2(-gamepad1.left_stick_y,gamepad1.left_stick_x);
        }






        movement_turn = ((AngleWrap((worldAngle_rad-targetAngle))/Math.toRadians(45)))*turnScale;

        telemetry.addData("realtive Angle:",df.format(relativeAngle));
        telemetry.addData("delta Angle",df.format(Math.toDegrees(AngleWrap((worldAngle_rad-targetAngle)))));

        */

  }

  public void ApplyMovement(){
      motorTL.setPower(-movement_y-movement_turn-movement_x);
      motorBL.setPower(-movement_y-movement_turn+movement_x);
      motorBR.setPower(-movement_y+movement_turn-movement_x);
      motorTR.setPower(-movement_y+movement_turn+movement_x);
  }


    public enum trigStates {
        secureDown,
        stopped,
        engaged;
        public BallFindingAuto.trigStates getNext() {
            return this.ordinal() < BallFindingAuto.trigStates.values().length - 1
                    ? BallFindingAuto.trigStates.values()[this.ordinal() + 1]
                    : null;
        }
    }
}
