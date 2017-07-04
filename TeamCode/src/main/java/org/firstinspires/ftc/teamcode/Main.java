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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import ftc.vision.FrameGrabber;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "MyTeleOp3", group = "MyTeleOp4")
public class Main extends OpMode {
  private ElapsedTime runtime = new ElapsedTime();

  /////////////////////////////////VARS////////////////////////////
  public final float turnScale = 1.0f;

    DcMotor motorTL;
    DcMotor motorTR;
    DcMotor motorBL;
    DcMotor motorBR;
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





  }

  /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
  @Override
  public void init_loop() {


  }

  /*
   * This method will be called ONCE when start is pressed
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void start() {
    runtime.reset();
  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
    @Override
    public void loop() {

        MovementCalc();

        if(FrameGrabber.bestBallX!=-1){
            //movement_y = 0.2f;
            if(FrameGrabber.bestBallX > 864/2){
                movement_turn = -0.1f;
            }else{
                movement_turn = 0.1f;
            }
        }



        ApplyMovement();
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
      motorTL.setPower(movement_y-movement_turn +movement_x);
      motorBL.setPower(movement_y-movement_turn-movement_x);
      motorBR.setPower(movement_y+movement_turn+movement_x);
      motorTR.setPower(movement_y+movement_turn-movement_x);
  }
}
