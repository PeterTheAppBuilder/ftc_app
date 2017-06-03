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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    float turnAmount = gamepad1.left_stick_x;
    if(turnAmount>0){
      turnAmount = (float) Math.pow(turnAmount,1.5);
    }else{
      turnAmount = (float) -Math.pow(turnAmount,1.5);
    }
    turnAmount *=turnScale;
    motorTL.setPower(gamepad1.right_stick_y-turnAmount +gamepad1.right_stick_x);
    motorBL.setPower(gamepad1.right_stick_y-turnAmount-gamepad1.right_stick_x);
    motorBR.setPower(gamepad1.right_stick_y+turnAmount+gamepad1.right_stick_x);
    motorTR.setPower(gamepad1.right_stick_y+turnAmount-gamepad1.right_stick_x);

  }
}
