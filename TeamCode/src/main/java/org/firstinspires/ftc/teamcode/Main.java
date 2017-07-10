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

import android.net.wifi.WifiEnterpriseConfig;
import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;

import ftc.vision.FrameGrabber;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "MyTeleOp3", group = "MyTeleOp4")
public class Main extends OpMode {
  private ElapsedTime runtime = new ElapsedTime();

    DecimalFormat df = new DecimalFormat("#.00");
    /////////////////////////////////VARS////////////////////////////
  public final float turnScale = 0.6f;
    DcMotor motorTL;
    DcMotor motorTR;
    DcMotor motorBL;
    DcMotor motorBR;
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
    float movement_y = 0;
    float movement_x = 0;
    float movement_turn = 0;
    //////////////////////////////////////////////////////////////////

  @Override
  public void init() {
    motorTL = hardwareMap.dcMotor.get("motorTL");
    motorTR = hardwareMap.dcMotor.get("motorTR");
    motorBL = hardwareMap.dcMotor.get("motorBL");
    motorBR = hardwareMap.dcMotor.get("motorBR");

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



        //////////reset encoders///////////////
        tracker_l.reset();
        tracker_r.reset();
        tracker_a.reset();
        wheelLeftLast = tracker_l.getPos();
        wheelRightLast= -tracker_r.getPos();
        ///////////////////////////////////////

        
        worldXPosition = 0;
        worldYPosition = 0;
        worldAngle_rad = Math.toRadians(90);
    }
    @Override
    public void loop() {
        MovementCalc();
        ApplyMovement();
        ReadEncodersOT();
        PositioningCalculations();
    }



    public final double moveScalingFactor = 18.2667;//when you divide the encoder values by this it returns cm
    public double worldXPosition=0;
    public double worldYPosition=0;
    public double worldAngle_rad = 0;//start the world angle at 0 degrees

    public double wheelLeftLast = 0;
    public double wheelRightLast = 0;
    public double wheelAuxLast = 0;
    public void PositioningCalculations(){

        double wheelLeftCurrent = tracker_l.getPos();
        double wheelRightCurrent= -tracker_r.getPos();
        double wheelAuxCurrent = tracker_a.getPos();

        double wheelLeftDelta = (wheelLeftCurrent - wheelLeftLast)*moveScalingFactor;
        double wheelRightDelta = (wheelRightCurrent - wheelRightLast)*moveScalingFactor;
        double wheelAuxDelta = (wheelAuxCurrent - wheelAuxLast)*moveScalingFactor;

        double robotWidth = 35.1327;//this is effectively the turning scale factor

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







    long All_telemetry_trackers_lastUpdate =0;
    public static final int All_telemetry_trackers_pollingRate = 5;
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
        movement_y = gamepad1.right_stick_y;
        movement_x = gamepad1.right_stick_x;
  }


  public void ApplyMovement(){
      motorTL.setPower(movement_y-movement_turn-movement_x);
      motorBL.setPower(movement_y-movement_turn+movement_x);
      motorBR.setPower(movement_y+movement_turn-movement_x);
      motorTR.setPower(movement_y+movement_turn+movement_x);
  }
}
