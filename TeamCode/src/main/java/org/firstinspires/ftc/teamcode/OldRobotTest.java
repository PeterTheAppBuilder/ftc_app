/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.media.MediaPlayer;
import android.os.SystemClock;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.w3c.dom.NamedNodeMap;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="MyTeleOp: MyTeleOp2", group="Iterative Opmode")  // @Autonomous(...) is the other common choice

public class MyTeleOp extends OpMode
{
    ////////////////////DEFINE VARIABLES//////////////////////////////////////

    //Auxiliary Globals.target_shooter_speed variables
    boolean left_bumper_down = false;
    boolean right_bumper_down = false;



    public float DistanceToTargetInches=0.0f;
    public float DistanceToTargetCentimeters=0.0f;

    public DcMotor wheelRight;
    public DcMotor wheelLeft;
    public DcMotor shooter_motor1;
    public DcMotor shooter_motor2;
    public DcMotor collector_motor;
    public DcMotor dead_motor;
    public DcMotor CapBallMotor1;
    public DcMotor CapBallMotor2;
    public Servo aim_servo1;
    public Servo beacon_servo_1;
    public Servo beacon_servo_2;

    public OpticalDistanceSensor LSA_Sensor1;
    public OpticalDistanceSensor LSA_Sensor2;


    //public AnalogInput WheelRightOT;
    //public AnalogInput WheelLeftOT;

    public Servo deployWheelServo1;
    public Servo deployWheelServo2;







    public DcMotorController ShooterEncoder;
    public DcMotorController OmniWheelEncoders;
    public Servo triggerServo;


    float StevenEstimation = 0.0f;//this variable will be set when steven estimates the distance to the target





    public boolean CapBallEngaged = false;



    public float wheelLeftLastPower = 0.0f;
    public float wheelRightLastPower = 0.0f;







    public TouchSensor triggerSensor;

    boolean isAccelerating = false;//if you are currently accelerating
    boolean isDecelerating = false;//if you are currently decelerating
    boolean DoneAccelerating = false;
    boolean StevenIsEstimating = false;//if steven is in estimation mode

    //////////////////////////////////////Button Down Variables/////////////////////////////////////
    boolean Gamepad2Left_BumperDown = false;//if game pad 1 is holding the x button (we are auto aiming)
    boolean Gamepad2LeftBumperDown = false;
    boolean Gamepad2DpadUpDown = false;//if the up button is down on gamepad 2
    boolean Gamepad2DpadRightDown = false;//if the right button is down on gamepad 2
    boolean Gamepad2DpadDownDown = false;//if the down button is down on gamepad 2
    boolean Gamepad2DpadLeftDown = false;//if the left button is down on gamepad 2
    boolean Gamepad2BDown = false;//if gamepad 2 is pressing a currently
    boolean Gamepad1RightStickDown = false;//if gamepad1 is currently pressing the right stick
    boolean movementInversed = false;//whether the robot drives backwards or forwards
    ////////////////////////////////////////////////////////////////////////////////////////////////

    boolean wheelsSpinning = false;
    /////////////////////////////Acceleration of the Wheels Variables//////////////////////////////
    float targetSpeedR = 0.0f; // the target speed of the right wheel
    float targetSpeedL = 0.0f; // the target speed of the left wheel
    float currentSpeedR = 0.0f; // the current speed of the right wheel
    float currentSpeedL = 0.0f; // the current speed of the left wheel
    float rampUpRate = 0.001f;// the amount added or subtraced eah update. this determines accel time

    float shooterConstant = 0.7691f;

    long lastDecelUpdate = 0;//the time of the last update in the decel algorithm
    long lastAccelUpdate = 0;//the time of the last update in the accel algorithm
    long AccelStartTime = 0;//when the shooter started accelerating in milliseconds

    public final float moveScalingFactor = 45.423f;//when you divide the encoder values by this it returns cm
    public float SpeedScale = 0.0f;//percent of total speed to be maximum speez
    public float target_shooter_power = 0;//the speed of the shooter when it accelerates
    public float shooter_max_speed = 0.184f;//the maximum allowed shooter speed before it shoots above the height limit
    public float shooter_current_power = 0;//the current speed the shooter is at

    public float aimServoPositionMaster = 0.0f;//the master position
    public float aimServoPositionSteven = 0.0f;//steven's manual shooter position



    public float aimServoPosition1;//the aiming servo position

    public boolean ypressed = false;
    public boolean ydown = false;

    public float worldStartingPositionX = 23.18f;
    public float worldStartingPositionY = 8.5f;

    //the robot doesn't start in the exact corner, so measurements needed to be taken
    public float worldXPosition=worldStartingPositionX;
    public float worldYPosition=worldStartingPositionY;
    public float worldAngle_rad = (float) Math.PI/2.0f;//start the world angle at 90 degrees (upward)



    long All_telemetry_trackers_lastUpdate =0;
    public static final int All_telemetry_trackers_pollingRate = 5;
    public static final float All_telemetry_trackers_wrapAroundThreshold = 0.3f;

    float WheelRightOT_curr_sensor_reading = 0.0f;
    float WheelRightOT_last_sensor_reading = 0.0f;
    float WheelRightOT_position_tracked =0.0f;
    float WheelRightOT_lastvalidreading = 0.0f;
    boolean WheelRightOT_WrapingAround = false;

    float WheelLeftOT_curr_sensor_reading = 0.0f;
    float WheelLeftOT_last_sensor_reading = 0.0f;
    float WheelLeftOT_position_tracked =0.0f;
    float WheelLeftOT_lastvalidreading = 0.0f;
    boolean WheelLeftOT_WrapingAround = false;





    public float fieldLength = 358.775f;//the length of the feild in cm
    public float HalffieldLength = fieldLength/2.0f;//the length of half field
    float robotLength = 44.5f;//robot height in centimeters

    //the position of the goal in centimeters
    public float goalPositionX = HalffieldLength + (float) Math.cos(Math.toRadians(-45.0f))*15.5f*2.54f;;
    public float goalPositionY = HalffieldLength + (float) Math.sin(Math.toRadians(-45.0f))*15.5f*2.54f;


    public float wheelLeftLast = 0;
    public float wheelRightLast = 0;

    float shooterServoK = 0.57f;
    float shooterVertical = 0.53f;




    public enum trigStates{
        state_ready,
        state_reload_begin,
        state_reload_busy,
        state_fire,
        state_firing,
        move,
        stop
    }



    public ColorSensor colorSensor1;





    public trigStates triggerStage = trigStates.stop;

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    public ModernRoboticsI2cRangeSensor loadSensor;

    @Override
    public void init() {
        Globals.target_shooter_speed = Globals.target_shooter_speed_base;//cool
        goalPositionX_offset = 0.0f;
        goalPositionY_offset = 0.0f;

        telemetry.setMsTransmissionInterval(10);
        telemetry.setAutoClear(true);
        ydown = false;
        ypressed = false;


        colorSensor1 = hardwareMap.colorSensor.get("colorSensorRed");
        colorSensor1.setI2cAddress( I2cAddr.create8bit(0x3c));
        colorSensor1.enableLed(false);



        LSA_Sensor1 =(ModernRoboticsAnalogOpticalDistanceSensor) hardwareMap.get("LSA_Sensor1");
        LSA_Sensor1.enableLed(true);
        LSA_Sensor2 =(ModernRoboticsAnalogOpticalDistanceSensor) hardwareMap.get("LSA_Sensor2");
        LSA_Sensor2.enableLed(true);


        loadSensor = (ModernRoboticsI2cRangeSensor) hardwareMap.get("loadSensor");
        loadSensor.setI2cAddress( I2cAddr.create8bit(0x20));

        wheelRight = hardwareMap.dcMotor.get("motor_1");
        wheelLeft = hardwareMap.dcMotor.get("motor_2");
        wheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        wheelLeft.setDirection(DcMotor.Direction.REVERSE);



        OmniWheelEncoders = hardwareMap.dcMotorController.get("collector and aim");


        ShooterEncoder = hardwareMap.dcMotorController.get("shooter controller");


        shooter_motor1 = hardwareMap.dcMotor.get("shooter");
        shooter_motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);//when applying zero power to the shooter, coast
        shooter_motor1.setDirection(DcMotor.Direction.REVERSE);

        //ThirdOmniWheel = (PWMOutputController) hardwareMap.pwmOutput.get("3rdWheel");


        shooter_motor2 = hardwareMap.dcMotor.get("motor_4");
        shooter_motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);//when applying zero power to the shooter, coast

        //these 2 motors have the encoders for the omni wheels plugged into them (we can't do speed control with them)
        collector_motor = hardwareMap.dcMotor.get("collector");
        dead_motor = hardwareMap.dcMotor.get("aim");
        collector_motor.setDirection(DcMotor.Direction.REVERSE);

        deployWheelServo1 = hardwareMap.servo.get("servo_1");
        deployWheelServo2 = hardwareMap.servo.get("servo_5");

        CapBallMotor1 = hardwareMap.dcMotor.get("lift_1");
        CapBallMotor2 = hardwareMap.dcMotor.get("lift_2");
        CapBallMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        CapBallMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        aim_servo1 = hardwareMap.servo.get("servo_2");
        triggerServo = hardwareMap.servo.get("servo_6");

        triggerServo.setPosition(0.5f);

        aim_servo1.setPosition(0.0f);

        beacon_servo_1=hardwareMap.servo.get("servo_3");
        beacon_servo_2=hardwareMap.servo.get("servo_4");



        //WheelRightOT = hardwareMap.analogInput.get("WheelRightOT");
        //WheelLeftOT = hardwareMap.analogInput.get("WheelLeftOT");


        triggerSensor = hardwareMap.touchSensor.get("triggerSensor");


        aimServoPositionMaster = 0.0f;
        beacon_servo_1.setPosition(Globals.beacon_servo1_on);
        beacon_servo_2.setPosition(Globals.beacon_servo2_on);



        //Autonomous is supposed to set the x and y coordinates of the goal,
        // but if it wasn't run, they will be -1, so we will default to the blue side
        goalPositionX = HalffieldLength + (float) Math.cos(Math.toRadians(-45.0f))*15.5f*2.54f;
        goalPositionY = HalffieldLength + (float) Math.sin(Math.toRadians(-45.0f))*15.5f*2.54f;



        //ir_seeker = hardwareMap.irSeekerSensor.get("ir_seeker");


        //SOMEDAY.....
        //touch_l = hardwareMap.touchSensor.get("touch_l");
        //touch_r = hardwareMap.touchSensor.get("touch_r");
        deployWheelServo1.setPosition(Globals.deployWheelServo1Up);
        deployWheelServo2.setPosition(Globals.deployWheelServo2Up);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    public static float worldXAutonomous;
    public static float worldYAutonomous;
    public static float worldAngleAutonomous;
    public static float goalXPosAutonomous;
    public static float goalYPosAutonomous;
    public static boolean blueside;
    public boolean firstInitLoop = true;
    @Override
    public void init_loop() {



        ///////Set our positions to what they were set in autonomous
        if(firstInitLoop){
            if(!Float.isNaN(worldXAutonomous)){
                worldXPosition = worldXAutonomous;
                worldYPosition = worldYAutonomous;
                worldAngle_rad = worldAngleAutonomous;
                goalPositionX = goalXPosAutonomous;
                goalPositionY = goalYPosAutonomous;
                firstInitLoop = false;
            }

        }
        wheelLeftLast = dead_motor.getCurrentPosition();
        wheelRightLast= collector_motor.getCurrentPosition();

        aim_servo1.setPosition(0.0f);


    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    public long firstXPressTime;


    public long fireStarttime;
    float goalPosX_withOffset = 0.0f;
    float goalPosY_withOffset = 0.0f;


    public enum motorStates{
        normal,
        slow,
        takingShot;
    }
    public long lastNormalTime = -1;
    public long waitShotTime = 500;

    public motorStates motorState = motorStates.normal;


    public float radiansToCenterDelta = -1.0f;
    public boolean ballLoaded = false;
    boolean g1Left_bumper_down = false;
    @Override
    public void loop() {
        FtcRobotControllerActivity.dimmer.longBright();
        ballLoaded = gamepad2.dpad_left;
        radiansToCenterDelta = Math.abs(AngleWrap(worldAngle_rad-((float) angleToCenter_rad-
                (float) Math.toRadians(180.0f))));
        if(motorState == motorStates.normal){
            lastNormalTime = SystemClock.uptimeMillis();
            wheelLeft.setPower(motorPowerL);
            wheelRight.setPower(motorPowerR);

            if((DistanceToTargetCentimeters <= 127.0f) && ballLoaded )
            {   motorState = motorStates.slow; }
        }
        if(motorState == motorStates.takingShot){
            wheelLeft.setPower(0);
            wheelRight.setPower(0);
            if(SystemClock.uptimeMillis()-lastNormalTime > waitShotTime){
                motorState = motorStates.normal;
            }
        }

        if(motorState == motorStates.slow){

            float motorPowerLFixed = motorPowerL;
            float motorPowerRFixed = motorPowerR;

            lastNormalTime = SystemClock.uptimeMillis();
            float percentThere = radiansToCenterDelta/(float) Math.toRadians(60.0f);
            if( percentThere < 0.3f) percentThere *= 0.5f;
            if(percentThere<0.15f){percentThere = 0.15f;}
            if(Math.abs(motorPowerL) > Math.abs(motorPowerR) && Math.abs(motorPowerL) > percentThere){
                if(motorPowerL > 0){
                    motorPowerLFixed = percentThere;
                }else{
                    motorPowerLFixed = -percentThere;
                }
                if(motorPowerR != 0){
                    motorPowerRFixed = motorPowerLFixed*(motorPowerR/motorPowerL);
                }
            }
            if(Math.abs(motorPowerR) > Math.abs(motorPowerL) && Math.abs(motorPowerR) > percentThere){
                if(motorPowerR > 0){
                    motorPowerRFixed = percentThere;
                }else{
                    motorPowerRFixed = -percentThere;
                }
                if(motorPowerL != 0){
                    motorPowerLFixed = motorPowerRFixed*(motorPowerL/motorPowerR);
                }
            }

            wheelRight.setPower(motorPowerRFixed);
            wheelLeft.setPower(motorPowerLFixed);

            if((DistanceToTargetCentimeters > 127.0f) || !ballLoaded )
            {   motorState = motorStates.normal;    }
        }

        if((radiansToCenterDelta <= Math.toRadians(4)) && (motorState == motorStates.slow) ){
            if(ballLoaded){
                motorState = motorStates.takingShot;
                triggerStage = trigStates.state_fire;
            }
        }
        telemetry.addLine()
                .addData("X: ",worldXPosition)
                .addData(" Y: ",worldYPosition)
                .addData(" ∠: ",Math.toDegrees(worldAngle_rad));


        telemetry.addLine()
            .addData("GoalX",goalPositionX)
            .addData("GoalY",goalPositionY);


        telemetry.addData("Shooter Speed:",Globals.target_shooter_speed);

        telemetry.addLine()
                .addData("Manual Adjs:","")
                .addData("X: ", goalPositionX_offset)
                .addData("Y: ", goalPositionY_offset);
        telemetry.addData("AimServo: ", aimServoPositionMaster);




        if(gamepad1.left_bumper){
            if(!g1Left_bumper_down){
                blueside = !blueside;
            }
            g1Left_bumper_down = true;
        }else{
            g1Left_bumper_down = false;
        }






        /*
        telemetry.addLine()
                .addData("Red", colorSensor1.red())
                .addData("Blue",colorSensor1.blue())
                .addData("Green",colorSensor1.green());
        */

        //telemetry.addData("AimServoPosition" ,aimServoPositionMaster);
        goalPosX_withOffset = goalPositionX + goalPositionX_offset;
        goalPosY_withOffset = goalPositionY + goalPositionY_offset;

        //calculate the distance to the target with the distance formula
        DistanceToTargetCentimeters = (float) Math.sqrt(Square(goalPosX_withOffset-worldXPosition)+Square(goalPosY_withOffset-worldYPosition));

        //telemetry.addData("Shooter Position",aimServoPosition1);
        //telemetry.addData("Shooter Speed", Globals.target_shooter_speed);
        if(gamepad2.dpad_up){
            Globals.target_shooter_speed+=0.00003f;
        }
        if(gamepad2.dpad_down){
            Globals.target_shooter_speed-=0.00003f;
        }



        float tempPos1;
        float tempPos2;
        if(gamepad1.right_trigger >= 0.2f){
            tempPos1 = Globals.deployWheelServo1Down;
        }else{
            tempPos1 = Globals.deployWheelServo1Up;
        }
        if(gamepad1.left_trigger >= 0.2f){
            tempPos2 = Globals.deployWheelServo2Down;
        }else{
            tempPos2 = Globals.deployWheelServo2Up;
        }

        if(LSA_aligning_on_line){
            tempPos1 = Globals.deployWheelServo1Up + 0.4f;
        }

        deployWheelServo1.setPosition(tempPos1);
        deployWheelServo2.setPosition(tempPos2);












        if(triggerStage == trigStates.state_reload_begin) {
            triggerServo.setPosition(0.0f);
            triggerStage = trigStates.state_reload_busy;
        }
        if(triggerStage == trigStates.move){
            triggerServo.setPosition(0.0f);
        }
        if(triggerStage == trigStates.stop){
            triggerServo.setPosition(0.5f);
        }
        if(triggerStage == trigStates.state_reload_busy) {
            if(triggerSensor.isPressed()){
                triggerServo.setPosition(0.5f);
                triggerStage = trigStates.state_ready;
            }
        }
        if(triggerStage == trigStates.state_fire){
            //only fire if we are reloaded
            if(triggerSensor.isPressed()){
                fireStarttime = SystemClock.uptimeMillis();
                triggerStage = trigStates.state_firing;
                //fired = true;
            }
            triggerServo.setPosition(0.0f);
        }
        if(triggerStage == trigStates.state_firing){
            //wait_for_rev_up for the touch sensor to be unpresseed, and commence reload.
            if(SystemClock.uptimeMillis()-fireStarttime > 500){
                triggerStage = trigStates.stop;

            }
        }
        if(triggerStage == trigStates.state_ready){
            if(triggerSensor.isPressed()){
                triggerServo.setPosition(0.5f);
            }else{
                triggerServo.setPosition(0.0f);
            }
        }





        CapBallMotorFunction();
        OverRideReset();
        Movement();
        Collector();
        StevenInputTargetAngle();


        ShooterSpeedCalculations();
        servo_movement();
        PositioningCalculations();
        AutoAimCalculations();
        DisplaySpeed();
        PeterAdjustVortexPos();






        if(gamepad1.right_bumper){
            SpeedScale = 1.0f;
        }else{

            SpeedScale = 0.65f;
        }



        telemetry.update();
    }




    boolean gamepad1dpadLeftDown = false;
    boolean gamepad1dpadRightDown = false;
    boolean gamepad1dpadDownDown = false;
    boolean gamepad1dpadUpDown = false;
    float vortexAdjustPosScale = 5.0f;

    float goalPositionX_offset = 0.0f;
    float goalPositionY_offset = 0.0f;

    public void PeterAdjustVortexPos(){
        //Give the ability to trash these
        if(gamepad1.y){
            goalPositionX_offset=0.0f;
            goalPositionY_offset = 0.0f;
        }
        if(gamepad1.x){
            if(gamepad1.dpad_left){
                if(!gamepad1dpadLeftDown){
                    goalPositionX_offset -= vortexAdjustPosScale;
                }
                gamepad1dpadLeftDown = true;
            }else{
                gamepad1dpadLeftDown = false;
            }

            if(gamepad1.dpad_right){
                if(!gamepad1dpadRightDown){
                    goalPositionX_offset += vortexAdjustPosScale;
                }
                gamepad1dpadRightDown = true;
            }else{
                gamepad1dpadRightDown = false;
            }



            if(gamepad1.dpad_up){
                if(!gamepad1dpadUpDown){
                    goalPositionY_offset += vortexAdjustPosScale;
                }
                gamepad1dpadUpDown = true;
            }else{
                gamepad1dpadUpDown = false;
            }

            if(gamepad1.dpad_down){
                if(!gamepad1dpadDownDown){
                    goalPositionY_offset -= vortexAdjustPosScale;
                }
                gamepad1dpadDownDown = true;
            }else{
                gamepad1dpadDownDown = false;
            }

        }

    }






    //our last imput to our file for debugging
    public long lastDataRecord = 0;

    public void RecordDataInFile(){

        if(lastDataRecord == 0){

            //DbgLog.msg("\nMY: what up, this is a log\n");
        }


        /*
        if(SystemClock.uptimeMillis()-lastDataRecord > 100){
            lastDataRecord = SystemClock.uptimeMillis();

            float blueReading = (float) Math.random();
            float redReading = (float) Math.random();
            float greenReading = (float) Math.random();

            ps.println(blueReading + "," + redReading + "," + greenReading);
        }
        if(gamepad1.x){
            ps.close();
            try {
                outputStream.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        */
    }








    public float shooterSpeedRealL = 0.0f;
    public float shooterSpeedRealR = 0.0f;
    public long shooterLeftLastPosition = 0;
    public long shooterRightLastPosition = 0;
    public long lastShooterSpeedUpdate = 0;
    public void DisplaySpeed(){
        if(SystemClock.uptimeMillis()-lastShooterSpeedUpdate >= 100){

            float shooter_motor1_position = shooter_motor1.getCurrentPosition();
            long curr_time = SystemClock.uptimeMillis();
            //DbgLog.msg("*S_03,"+ ","+curr_time+","+shooter_motor1_position);


            shooterSpeedRealL = (shooter_motor1.getCurrentPosition()-shooterLeftLastPosition)/1000f;
            shooterSpeedRealR = (shooter_motor2.getCurrentPosition()-shooterRightLastPosition)/1000f;

            shooterLeftLastPosition = shooter_motor1.getCurrentPosition();
            shooterRightLastPosition = shooter_motor2.getCurrentPosition();


            lastShooterSpeedUpdate = curr_time;


        }
        //telemetry.addData("Speed Ls REAL", shooterSpeedRealL);
        //telemetry.addData("Speed Rs REAL", shooterSpeedRealR);
    }





    public float CapBall1EngagedStartingPosition = -1.0f;
    public float CapBall2EngagedStartingPosition = -1.0f;
    public boolean CapBallDeployed = false;

    public long CapBallDeployTime = -1;
    public void CapBallMotorFunction(){


        if(gamepad2.right_trigger > 0.5f && !CapBallEngaged){
            CapBallMotor1.setPower(0.5f);
            CapBallMotor2.setPower(-0.5f);
            CapBallEngaged = true;
            CapBall1EngagedStartingPosition = CapBallMotor1.getCurrentPosition();
            CapBall2EngagedStartingPosition = CapBallMotor2.getCurrentPosition();
            aimServoPositionSteven = 1.0f;
        }

        if(CapBallEngaged && !CapBallDeployed){
            if(Math.abs(CapBallMotor1.getCurrentPosition())-Math.abs(CapBall1EngagedStartingPosition) > 25.0f){
                CapBallMotor1.setPower(0.0f);
                CapBallMotor2.setPower(0.0f);
                //////////Give Steven normal control only once he releases the trigger
                if(gamepad2.right_trigger < 0.05f){
                    CapBallDeployed = true;
                }
            }

        }





        boolean goingUpwards = gamepad2.right_trigger>=gamepad2.left_trigger;
        float CapBallMotor1CurrPosition = CapBallMotor1.getCurrentPosition();
        float CapBallMotor2CurrPosition = CapBallMotor2.getCurrentPosition();
        //float CapBallMotor2CurrPosition = CapBallMotor2.getCurrentPosition();
        float CapBallMotor1Delta = Math.abs(CapBallMotor1CurrPosition-CapBall1EngagedStartingPosition);
        //float CapBallMotor2Delta = Math.abs(CapBallMotor1CurrPosition-CapBall2EngagedStartingPosition);

        if(CapBallDeployed){
            float CapBallMotor1SpeedScale = 1.0f;
            float CapBallMotor2SpeedScale = 1.0f;
            if(goingUpwards){
                CapBallMotor1SpeedScale = 1.0f;
                CapBallMotor2SpeedScale = 1.0f;
            }else{
                CapBallMotor1SpeedScale = 0.9f;
                CapBallMotor2SpeedScale = 0.9f;
            }

            if(CapBallMotor1Delta <= 1700 && !goingUpwards){
                CapBallMotor1SpeedScale = 0.4f;
                CapBallMotor2SpeedScale = 0.4f;
            }

            if(CapBallMotor1Delta > 6800 && goingUpwards){
                CapBallMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                CapBallMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                CapBallMotor1SpeedScale = 0.3f;
                CapBallMotor2SpeedScale = 0.3f;
            }
            if(!goingUpwards){
                CapBallMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                CapBallMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if(!goingUpwards && CapBallMotor1CurrPosition<CapBall1EngagedStartingPosition){
                CapBallMotor1SpeedScale = 0.0f;
            }
            if(!goingUpwards && CapBallMotor2CurrPosition>CapBall2EngagedStartingPosition){
                CapBallMotor2SpeedScale = 0.0f;
            }
            if(!gamepad2.right_bumper){
                CapBallMotor1.setPower((gamepad2.right_trigger-(gamepad2.left_trigger*0.6f)) * CapBallMotor1SpeedScale);
                CapBallMotor2.setPower(((-gamepad2.right_trigger) + (gamepad2.left_trigger*0.6f)) * CapBallMotor2SpeedScale);
            }else{
                CapBallMotor1.setPower(gamepad2.right_trigger * CapBallMotor1SpeedScale);
            }
            //telemetry.addData("CapBallLift1",CapBallMotor1.getCurrentPosition());
            //telemetry.addData("CapBallLift2",CapBallMotor2.getCurrentPosition());
        }

    }







    public long firstSpecificationClick = -1;
    public boolean specifyButtonPressed = false;
    private void StevenInputTargetAngle() {
        if(gamepad2.left_stick_button){
            float AngleOfLeftStick = (float) Math.atan2(-gamepad2.left_stick_y,gamepad2.left_stick_x);//angle = opposite over adjacent
            if(SystemClock.uptimeMillis()-firstSpecificationClick < 400){
                AngleOfLeftStick = (float) Math.round(AngleOfLeftStick/Math.toRadians(45.0f)) * (float) Math.toRadians(45.0f);
            }
            goalPositionX = HalffieldLength + (float) Math.cos(AngleOfLeftStick)*15.5f*2.54f;
            goalPositionY = HalffieldLength + (float) Math.sin(AngleOfLeftStick)*15.5f*2.54f;

            specifyButtonPressed = true;
        }else{
            if(specifyButtonPressed){
                firstSpecificationClick = SystemClock.uptimeMillis();
            }
            specifyButtonPressed = false;
        }
    }

    private void StevenEstimation() {
        if(gamepad1.left_bumper){
            //if this is the first update of steven pressing the left bumper
            if(!Gamepad2LeftBumperDown){


                //invert the var StevenIsEstimating
                if(StevenIsEstimating){
                    StevenIsEstimating = false;
                    //this is the first update of Steven finishing his estimation
                    //multiply the number of tiles he entered by the length of each tile to get the number of inches

                    StevenEstimation *= 23.6f;
                    DistanceToTargetInches= StevenEstimation;

                }else{
                    StevenIsEstimating = true;
                    //start his estimation off at 0
                    StevenEstimation = 0.0f;
                }


            }
            Gamepad2LeftBumperDown = true;
        }
        else{
            Gamepad2LeftBumperDown = false;

        }




        if(StevenIsEstimating){
            if(gamepad1.dpad_up){
                //if this is the first update of steven pressing the up button
                if(!Gamepad2DpadUpDown){
                    StevenEstimation+=1;//add one tile's worth of length to Steven's estimation
                }
                Gamepad2DpadUpDown = true;
            }
            else{
                Gamepad2DpadUpDown = false;
            }


            if(gamepad1.dpad_right){
                //if this is the first update of steven pressing the right button
                if(!Gamepad2DpadRightDown){
                    StevenEstimation+=0.5f;//add one half tile's worth of length to Steven's estimation
                }
                Gamepad2DpadRightDown = true;
            }
            else{
                Gamepad2DpadRightDown = false;
            }

            if(gamepad1.dpad_down){
                //if this is the first update of steven pressing the down button
                if(!Gamepad2DpadDownDown){
                    StevenEstimation+=0.25f;//add one tile's worth of length to Steven's estimation
                }
                Gamepad2DpadDownDown = true;
            }
            else{
                Gamepad2DpadDownDown = false;
            }

            if(gamepad1.dpad_left){
                //if this is the first update of steven pressing the left button
                if(!Gamepad2DpadLeftDown){
                    StevenEstimation+=0.125f;//add one tile's worth of length to Steven's estimation
                }
                Gamepad2DpadLeftDown = true;
            }
            else{
                Gamepad2DpadLeftDown = false;
            }
        }



    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public float Square(float number){
        return number*number;
    }

    float lastSpeedR;
    float lastSpeedL;
    float motorPowerL;
    float motorPowerR;
    long lastMoveUpdateTime = 0;
    private void Movement(){

        //this is the amount of turning inputed by the gamepad
        float gamepad_turning_fine = gamepad1.left_stick_x;

        float speed_forward_user = gamepad1.right_stick_y;
        float netSpeed_user = speed_forward_user;
        //there are 2 gears going from the motor, so polarity is inverted
        netSpeed_user *= -1;
        //////////////////////////////////////////////////////////////////////////////////

        //we will start the target speeds at whatever the netspeed is
        //to turn, we will simply decrement powers of the motors if need be
        float TargetSpeedR = netSpeed_user;
        float TargetSpeedL = netSpeed_user;


        //if turning is positive, slow the right motor down to turn to the right
        //else slow the left motor down to turn to the left
        if(gamepad_turning_fine<0){
            TargetSpeedR= netSpeed_user * (1-(1.0f * Math.abs(gamepad_turning_fine)));
            TargetSpeedL = netSpeed_user * (1.0f + Math.abs(gamepad_turning_fine));
        }
        else{
            TargetSpeedL = netSpeed_user * (1-(1.0f * Math.abs(gamepad_turning_fine)));
            TargetSpeedR = netSpeed_user * (1.0f + Math.abs(gamepad_turning_fine));
        }
        TargetSpeedR = Range.clip(TargetSpeedR, -1, 1);
        TargetSpeedL = Range.clip(TargetSpeedL, -1, 1);


        TargetSpeedR *= -1f * SpeedScale;
        TargetSpeedL *= -1f * SpeedScale;


        //inverting the movement direction if right stick is pressed
        if(!Gamepad1RightStickDown){
            if(gamepad1.right_stick_button){
                movementInversed = !movementInversed;
                Gamepad1RightStickDown = true;
            }
        }else{
            if(!gamepad1.right_stick_button){
                Gamepad1RightStickDown = false;
            }
        }


        ///Make the powers lag behind in order to avoid slipage
        //TargetSpeedL = wheelLeftLastPower + (0.1f * (TargetSpeedL-wheelLeftLastPower));
        //TargetSpeedR = wheelRightLastPower + (0.1f * (TargetSpeedR-wheelRightLastPower));


        /*
        float accelLimit = 0.0025f * (1+ Math.abs(wheelLeftLastPower-wheelRightLastPower * 3));
        if(Math.abs(wheelLeftLastPower-TargetSpeedL)/(SystemClock.uptimeMillis()-lastMoveUpdateTime) > accelLimit){
            if(TargetSpeedL-wheelLeftLastPower < 0) {
                TargetSpeedL = wheelLeftLastPower - accelLimit;
            }
        }
        if(Math.abs(wheelRightLastPower-TargetSpeedR)/(SystemClock.uptimeMillis()-lastMoveUpdateTime) > accelLimit){
            if(TargetSpeedR-wheelRightLastPower < 0){
                TargetSpeedR = wheelRightLastPower - accelLimit;
            }
        }
        lastMoveUpdateTime = SystemClock.uptimeMillis();

        wheelLeftLastPower = TargetSpeedL;
        wheelRightLastPower = TargetSpeedR;
        */










        if(!gamepad1.a && !gamepad1.b){
            if(!Gamepad2Left_BumperDown){//don't be applying two powers at once
                if(!movementInversed){
                    motorPowerR = TargetSpeedR;
                    motorPowerL = TargetSpeedL;
                }else{
                    motorPowerR = -TargetSpeedL;
                    motorPowerL = -TargetSpeedR;
                }

            }
        }else{
            motorPowerL = -0.2f;
            motorPowerR = -0.2f;
        }

    }


    private void Collector(){
        if(gamepad2.y){
            ydown = true;
        }else{
            if(ydown){
                if(ypressed){
                    ypressed = false;
                }else{
                    ypressed = true;
                }

            }
            ydown = false;
        }
        if(ypressed){
            collector_motor.setPower(Globals.collector_power);
        }
        else{
            collector_motor.setPower(0);
        }
        if(gamepad2.right_bumper){
            collector_motor.setPower(-Globals.collector_power);
        }
    }

    private void ShooterSpeedCalculations(){
        if(gamepad2.b){
            if(!Gamepad2BDown){
                wheelsSpinning = !wheelsSpinning;
                Gamepad2BDown = true;
            }
        }else{
            Gamepad2BDown = false;
        }




        if(wheelsSpinning){
            if(!isAccelerating && !DoneAccelerating){//if this is the first update that the bbutton was down..
                //initialize our clock
                lastAccelUpdate = SystemClock.uptimeMillis();
                //Don't do speed control when accelerating
                shooter_motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shooter_motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                isAccelerating = true;
                isDecelerating = false;

                target_shooter_power = 0.20f;//accelerate to this and once done,

            }

        }
        else{

            if(isAccelerating){//if this is the first update of deceleration
                isDecelerating = true;
                //For deceleration, (or not moving) we will not use encoders
                shooter_motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shooter_motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //remember the current time in last decel update
                lastDecelUpdate = SystemClock.uptimeMillis();

                isAccelerating = false;//you are not accelerating
                DoneAccelerating = false;
            }

        }

        if(isAccelerating){
            //if you have not reached maximum power, accelerate the wheels slowly, for 4 seconds based on TIME
            if(shooter_current_power < target_shooter_power){
                shooter_current_power += ((SystemClock.uptimeMillis()-lastAccelUpdate)/3000.0f);
                //remember the time of this update for the next calculation
                lastAccelUpdate = SystemClock.uptimeMillis();
                //since we just made a power change, update power, and clip it for safety
                shooter_current_power = Range.clip(shooter_current_power,-1.0f,1.0f);
                shooter_motor1.setPower(shooter_current_power);
                shooter_motor2.setPower(shooter_current_power);
            }
            else{//we are done accelerating
                DoneAccelerating = true;

                //change over to speed control, because you are at target power
                shooter_motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shooter_motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //apply target speed
                shooter_motor1.setPower(Globals.target_shooter_speed);
                shooter_motor2.setPower(Globals.target_shooter_speed);

            }
        }
        if(isDecelerating){

            shooter_current_power -= (SystemClock.uptimeMillis()-lastDecelUpdate)/3000.0f;//decelerate at a rate that would take 3 seconds to decelerate from 1.0 to 0.0
            lastDecelUpdate = SystemClock.uptimeMillis();

            if(shooter_current_power <= 0.0f){
                isDecelerating = false;//you are done decelerating
                shooter_motor1.setPower(0);
                shooter_motor2.setPower(0);
                shooter_current_power = 0.0f;
            }

            else{

                //apply target power
                shooter_motor1.setPower(shooter_current_power);
                shooter_motor2.setPower(shooter_current_power);
            }
        }
    }

    public boolean fireButtonDown = false;


    public float trigger_servo_position_neutral = 0.49f;
    public float trigger_servo_position_activated = 0.0f;
    public float trigger_servo_current_position = trigger_servo_position_neutral;
    public long last_servo_loop_update = -1;
    //boolean fired = false;
    public void servo_movement(){
        aimServoPositionSteven-=(gamepad2.right_stick_y/150.0f);
        aimServoPositionSteven= Range.clip(aimServoPositionSteven,0.0f,1.0f);
        //beacon_servo_1.setPosition(aimServoPositionSteven);
        //telemetry.addData("Point to center servo %", aimServoPositionSteven);
        if(gamepad2.a || gamepad2.dpad_left)//auto aim the shooter
        {
            //calculate the Aim based on the curve fit we did
            float calculatedAim =  Globals.shooterConstant - (Globals.shooterSlope * DistanceToTargetCentimeters);//0.00178026f = old
            //to shoot straight up:
            telemetry.addData("calculatedAim=",calculatedAim);

            aimServoPositionMaster = calculatedAim;
            /*if(triggerStage == trigStates.state_ready){

            }else{
                if(triggerStage != trigStates.state_reload_busy && triggerStage != trigStates.state_fire
                        && triggerStage != trigStates.state_firing){
                    triggerStage = trigStates.state_reload_begin;
                }
            }
            */

        }
        else{
            /*
            if(fired){
                triggerStage = trigStates.stop;
            }
            fired = false;
            */
            if(gamepad2.right_stick_button){
                aimServoPositionMaster = Globals.aimLoadPosition;
                aimServoPositionSteven = Globals.aimLoadPosition;
            }else{
                aimServoPositionMaster = aimServoPositionSteven;
            }

        }



        aimServoPositionMaster = Range.clip(aimServoPositionMaster,0.0f,1.0f);

        //telemetry.addData("Aim Servo Position:", aimServoPositionMaster);

        if(gamepad2.left_bumper){
            aimServoPositionMaster = 0.677f;
        }

        aim_servo1.setPosition(aimServoPositionMaster);

        //telemetry.addData("trigger Stage",triggerStage);

        long currentTimeMilis = SystemClock.uptimeMillis();
        //////////////////TRIGGER SERVO///////////////////////////////////////////////////////////////////////////////////////////////
        if(gamepad2.x){
            trigger_servo_current_position += (((currentTimeMilis-last_servo_loop_update)/0.5f)
                    *(trigger_servo_position_activated-trigger_servo_position_neutral));
            if(trigger_servo_current_position <= trigger_servo_position_activated)
            {trigger_servo_current_position=trigger_servo_position_activated;}



            if(SystemClock.uptimeMillis()-firstXPressTime < 500 && !fireButtonDown){
                triggerStage = trigStates.state_reload_begin;
            }else{
                if(triggerStage != trigStates.state_reload_busy && triggerStage != trigStates.state_firing){
                    triggerStage = trigStates.state_fire;
                }
            }
            firstXPressTime = SystemClock.uptimeMillis();
            fireButtonDown = true;
        }else{
            if(triggerStage != trigStates.state_reload_busy && triggerStage != trigStates.state_firing && triggerStage != trigStates.state_ready){
                triggerStage = trigStates.state_reload_begin;
            }
            fireButtonDown = false;
        }
        last_servo_loop_update = currentTimeMilis;


        if(gamepad2.dpad_right){
            beacon_servo_1.setPosition(Globals.beacon_servo1_off);
            beacon_servo_2.setPosition(Globals.beacon_servo2_off);
        }else{
            //beacon_servo_1.setPosition((Math.toDegrees(AngleWrap(((+(worldAngle_rad-(float) Math.PI) -(float) angleToCenter_rad)))))/205.0f+0.5224f );
            //telemetry.addData("AngleToCenter", angleToCenter_rad);
            beacon_servo_1.setPosition(Globals.beacon_servo1_on);
            beacon_servo_2.setPosition(Globals.beacon_servo2_on);
        }
        /*
        if(gamepad2.dpad_left){
            beacon_servo_2.setPosition(Globals.beacon_servo2_off);
        }else{
            beacon_servo_2.setPosition(Globals.beacon_servo2_on);
        }
        */

    }

    private void OverRideReset(){

        if(!gamepad1.x){
            if(gamepad1.dpad_up){
                worldXPosition = HalffieldLength;
                worldYPosition=fieldLength-(robotLength-worldStartingPositionY);
                worldAngle_rad = (float) Math.toRadians(90.0f);


                goalPositionX_offset=0.0f;
                goalPositionY_offset = 0.0f;
            }

            if(gamepad1.dpad_down){
                worldXPosition = HalffieldLength;
                worldYPosition=(robotLength-worldStartingPositionY);
                worldAngle_rad = (float) Math.toRadians(270.0f);

                goalPositionX_offset=0.0f;
                goalPositionY_offset = 0.0f;
            }
            if(gamepad1.dpad_left){
                worldXPosition = robotLength-worldStartingPositionY;
                worldYPosition=HalffieldLength;
                worldAngle_rad = (float) Math.toRadians(180.0f);

                goalPositionX_offset=0.0f;
                goalPositionY_offset = 0.0f;
            }
            if(gamepad1.dpad_right){
                worldXPosition = fieldLength-(robotLength-worldStartingPositionY);
                worldYPosition=HalffieldLength;
                worldAngle_rad = (float) Math.toRadians(0.0f);

                goalPositionX_offset=0.0f;
                goalPositionY_offset = 0.0f;
            }
        }


        //telemetry.addData("LSA Reading 1:",LSA_Sensor1.getRawLightDetected());
        //telemetry.addData("LSA Reading 2:", LSA_Sensor2.getRawLightDetected());
        //telemetry.addData("Aligning On line?",LSA_aligning_on_line);
        //telemetry.addData("First Sensor Went First",LSA_first_sensor_went_first);
        telemetry.addData("Blueside?",blueside);



        if(gamepad1.a || gamepad1.b){
            boolean seeingRed_lightsensor1 = LSA_Sensor1.getRawLightDetected()> Globals.red_line_intensity;
            boolean seeingRed_lightsensor2 = LSA_Sensor2.getRawLightDetected()> Globals.red_line_intensity;

            //If this is the first update we are crossing the line with the first sensor
            if(seeingRed_lightsensor1 && !LSA_aligning_on_line&&LSA_done){
                LSA_done = false;


                LSA_x1 = worldXPosition;
                LSA_y1 = worldYPosition;


                LSA_aligning_on_line = true;
                LSA_first_sensor_went_first = true;
            }
            //If this is the first update we are crossing the line with the second sensor
            if(seeingRed_lightsensor2 && !LSA_aligning_on_line&&LSA_done){
                LSA_done = false;
                LSA_x1 = worldXPosition;
                LSA_y1 = worldYPosition;
                LSA_aligning_on_line = true;
                LSA_first_sensor_went_first = false;
            }
            //if we are done aligning and we are not done the entire process, we are done
            if(!LSA_done && !LSA_aligning_on_line && !seeingRed_lightsensor1 && !seeingRed_lightsensor2){
                LSA_done = true;
            }

            //if this is the first time the other sensor has crossed the line (we can now align or position)
            if((LSA_aligning_on_line && seeingRed_lightsensor2 && LSA_first_sensor_went_first && !LSA_done) ||
                    (LSA_aligning_on_line && seeingRed_lightsensor1 && !LSA_first_sensor_went_first && !LSA_done)){
                LSA_aligning_on_line = false;
                LSA_x2 = worldXPosition;
                LSA_y2 = worldYPosition;
                //calculate how far we went
                float LSA_distanceTraveled = (float) Math.sqrt(Square(LSA_x2-LSA_x1)+Square(LSA_y2-LSA_y1));


                if(gamepad1.a){
                    if(LSA_first_sensor_went_first){

                        if(blueside){
                            worldAngle_rad = (float)
                                    (Math.toRadians(135.0f)-
                                            Math.atan2(LSA_distanceTraveled,
                                                    Globals.LSA_distance_between_sensors));
                        }else{
                            worldAngle_rad = (float)
                                    (Math.toRadians(45.0f)-
                                            Math.atan2(LSA_distanceTraveled,
                                                    Globals.LSA_distance_between_sensors));
                        }

                    }else{
                        if(blueside){
                            worldAngle_rad = (float)
                                    (Math.toRadians(135.0f)+
                                            Math.atan2(LSA_distanceTraveled,
                                                    Globals.LSA_distance_between_sensors));
                        }else{
                            worldAngle_rad = (float)
                                    (Math.toRadians(45.0f)+
                                            Math.atan2(LSA_distanceTraveled,
                                                    Globals.LSA_distance_between_sensors));
                        }

                    }
                }else{
                    if(LSA_first_sensor_went_first){
                        if(blueside){
                            worldAngle_rad = (float)
                                    (Math.toRadians(315.0f)-
                                            Math.atan2(LSA_distanceTraveled,
                                                    Globals.LSA_distance_between_sensors));
                        }else{
                            worldAngle_rad = (float)
                                    (Math.toRadians(225.0f)-
                                            Math.atan2(LSA_distanceTraveled,
                                                    Globals.LSA_distance_between_sensors));
                        }

                    }else{
                        if(blueside){
                            worldAngle_rad = (float)
                                    (Math.toRadians(315.0f)+
                                            Math.atan2(LSA_distanceTraveled,
                                                    Globals.LSA_distance_between_sensors));
                        }else{
                            worldAngle_rad = (float)
                                    (Math.toRadians(225.0f)+
                                            Math.atan2(LSA_distanceTraveled,
                                                    Globals.LSA_distance_between_sensors));
                        }

                    }
                }


                worldAngle_rad = AngleWrap(worldAngle_rad);


                calculateNewPosition();

            }
        }else{
            LSA_aligning_on_line = false;
            LSA_done = true;
        }
        /*
        telemetry.addData("angle_robotToSensor_relative",Math.toDegrees(angle_robotToSensor_relative));
        telemetry.addData("angle_robotToSensor_absolute",Math.toDegrees(angle_robotToSensor_absolute));
        telemetry.addData("distance_robotToSensor",distance_robotToSensor);
        telemetry.addData("sensor_relative_to_robot_y",sensor_relative_to_robot_y);
        telemetry.addData("sensor_relative_to_robot_x",sensor_relative_to_robot_x);
        telemetry.addData("predicted_distance_to_corner",predicted_distance_to_corner);
        telemetry.addData("predicted_angle_to_corner_from_line",Math.toDegrees(predicted_angle_to_corner_from_line));
        telemetry.addData("calculated_distance_from_bottom_corner",calculated_distance_from_bottom_corner);
        telemetry.addData("sensor_worldXPosition",sensor_worldXPosition);
        telemetry.addData("sensor_worldYPosition",sensor_worldXPosition);
        */

    }
    float LSA_x1 = 0.0f;
    float LSA_y1 = 0.0f;
    float LSA_x2 = 0.0f;
    float LSA_y2 = 0.0f;
    boolean LSA_aligning_on_line=false;
    boolean LSA_first_sensor_went_first = false;
    boolean LSA_done = true;


    float angle_robotToSensor_relative = 0.0f;
    float angle_robotToSensor_absolute = 0.0f;
    float distance_robotToSensor = 0.0f;
    float sensor_relative_to_robot_y = 0.0f;
    float sensor_relative_to_robot_x = 0.0f;
    float sensor_worldXPosition = 0.0f;
    float sensor_worldYPosition = 0.0f;
    float sensor_predicted_pos_y = 0.0f;
    float sensor_predicted_pos_x = 0.0f;
    float predicted_distance_to_corner = 0.0f;
    float predicted_angle_to_corner = 0.0f;
    float predicted_angle_to_corner_from_line = 0.0f;
    float calculated_distance_from_corner = 0.0f;
    float calculated_distance_from_bottom_corner = 0.0f;



    public void calculateNewPosition(){
        float LSA_sensor_x_relative_final = Globals.LSA_sensor_x_relative;
        if(LSA_first_sensor_went_first){
            LSA_sensor_x_relative_final = -Globals.LSA_sensor_x_relative;
        }



        angle_robotToSensor_relative =
                (float) Math.toRadians(90.0f)- (float) Math.atan2(Globals.LSA_sensor_y_relative,
                LSA_sensor_x_relative_final);
        angle_robotToSensor_absolute = angle_robotToSensor_relative+worldAngle_rad;
        distance_robotToSensor = (float)
                Math.sqrt(Square(Globals.LSA_sensor_y_relative)+Square(LSA_sensor_x_relative_final));


        sensor_relative_to_robot_y = (float)
                Math.sin(angle_robotToSensor_absolute)*distance_robotToSensor;
        sensor_relative_to_robot_x = (float)
                Math.cos(angle_robotToSensor_absolute)*distance_robotToSensor;
        sensor_predicted_pos_y = sensor_relative_to_robot_y
                + worldYPosition;
        sensor_predicted_pos_x = sensor_relative_to_robot_x
                + worldXPosition;


        predicted_distance_to_corner = (float) Math.sqrt(Square(fieldLength-sensor_predicted_pos_x)
                +Square(fieldLength-sensor_predicted_pos_y));
        if(!blueside){
            predicted_distance_to_corner = (float) Math.sqrt(Square(sensor_predicted_pos_x)
                    +Square(fieldLength-sensor_predicted_pos_y));
        }

        predicted_angle_to_corner = (float) Math.atan2(fieldLength-sensor_predicted_pos_x,
                fieldLength-sensor_predicted_pos_y);
        if(!blueside){
            predicted_angle_to_corner = (float) Math.atan2(sensor_predicted_pos_x,
                    fieldLength-sensor_predicted_pos_y);
        }
        predicted_angle_to_corner_from_line =
                predicted_angle_to_corner-(float) Math.toRadians(45.0f);

        calculated_distance_from_corner = predicted_distance_to_corner*
                (float) Math.cos(predicted_angle_to_corner_from_line);
        calculated_distance_from_bottom_corner = (fieldLength*1.414213f)-calculated_distance_from_corner;



        sensor_worldXPosition = calculated_distance_from_bottom_corner/1.414213f;
        sensor_worldYPosition = sensor_worldXPosition;

        if(!blueside){
            sensor_worldXPosition = fieldLength-sensor_worldXPosition;
        }


        worldXPosition = sensor_worldXPosition-sensor_relative_to_robot_x;
        worldYPosition = sensor_worldYPosition-sensor_relative_to_robot_y;





    }







    public void PositioningCalculations(){
        float balanceScalingFactor =
                1.0f;

        //float wheelRightCurrent = WheelRightOT_position_tracked;
        //float wheelLeftCurrent = WheelLeftOT_position_tracked;

        float wheelLeftCurrent = dead_motor.getCurrentPosition()/balanceScalingFactor;
        float wheelRightCurrent= collector_motor.getCurrentPosition()*balanceScalingFactor;
        //telemetry.addData("wheel Left Pos:",dead_motor.getCurrentPosition());
        //telemetry.addData("wheel Right Pos:",collector_motor.getCurrentPosition());

        float wheelLeftDelta = (wheelLeftCurrent - wheelLeftLast)/moveScalingFactor;
        float wheelRightDelta = (wheelRightCurrent - wheelRightLast)/moveScalingFactor;


        //if(!(Math.abs(wheelLeftDelta) < 0.25f && Math.abs(wheelRightDelta) < 0.25f)){
        float robotWidth = 38.62f;//39.2247 OLD in centimeters// this is effectively the turning scale factor

        float Angleincrement = (wheelRightDelta-wheelLeftDelta)/robotWidth;
        worldAngle_rad += Angleincrement;
        //telemetry.addData("WorldAngle:", Math.toDegrees(worldAngle_rad));
        float Distance = (wheelRightDelta+wheelLeftDelta)/2.0f;
        worldXPosition += Math.cos(worldAngle_rad) * Distance;
        worldYPosition += Math.sin(worldAngle_rad) * Distance;


        wheelLeftLast = wheelLeftCurrent;
        wheelRightLast = wheelRightCurrent;


        worldAngle_rad = AngleWrap(worldAngle_rad);








        ReadEncodersOT();

    }



    public void ReadEncodersOT(){
        ////////////////////////////PositionTrackers///////////////////////////////
        if(SystemClock.uptimeMillis()- All_telemetry_trackers_lastUpdate > All_telemetry_trackers_pollingRate){
            All_telemetry_trackers_lastUpdate = SystemClock.uptimeMillis();
            // Get the current reading
            //WheelRightOT_curr_sensor_reading = Math.abs((float) WheelRightOT.getVoltage()/ (float) WheelRightOT.getMaxVoltage());
            //WheelLeftOT_curr_sensor_reading = Math.abs((float) WheelLeftOT.getVoltage()/ (float) WheelLeftOT.getMaxVoltage());

            float WheelRightOT_delta_temp = WheelRightOT_curr_sensor_reading - WheelRightOT_last_sensor_reading;
            float WheelLeftOT_delta_temp = WheelLeftOT_curr_sensor_reading - WheelLeftOT_last_sensor_reading;

            float WheelRightOT_delta_final = WheelRightOT_delta_temp;
            float WheelLeftOT_delta_final = WheelLeftOT_delta_temp;


            if(Math.abs(WheelRightOT_delta_temp) <= All_telemetry_trackers_wrapAroundThreshold && WheelRightOT_WrapingAround){
                WheelRightOT_WrapingAround = false;
                WheelRightOT_delta_temp = WheelRightOT_curr_sensor_reading - WheelRightOT_lastvalidreading;
                if(WheelRightOT_delta_temp > 0.5f){
                    WheelRightOT_delta_final -=1.0f;
                }
                if(WheelRightOT_delta_temp <-0.5f){
                    WheelRightOT_delta_final +=1.0f;
                }
            }
            if(Math.abs(WheelLeftOT_delta_temp) <= All_telemetry_trackers_wrapAroundThreshold && WheelLeftOT_WrapingAround){
                WheelLeftOT_WrapingAround = false;
                WheelLeftOT_delta_temp = WheelLeftOT_curr_sensor_reading - WheelLeftOT_lastvalidreading;
                if(WheelLeftOT_delta_temp > 0.5f){
                    WheelLeftOT_delta_final -=1.0f;
                }
                if(WheelLeftOT_delta_temp <-0.5f){
                    WheelLeftOT_delta_final +=1.0f;
                }
            }

            if(Math.abs(WheelRightOT_delta_temp) >= All_telemetry_trackers_wrapAroundThreshold && !WheelRightOT_WrapingAround){
                WheelRightOT_WrapingAround =true;
                WheelRightOT_lastvalidreading = WheelRightOT_last_sensor_reading;
            }
            if(Math.abs(WheelLeftOT_delta_temp) >= All_telemetry_trackers_wrapAroundThreshold && !WheelLeftOT_WrapingAround){
                WheelLeftOT_WrapingAround =true;
                WheelLeftOT_lastvalidreading = WheelLeftOT_last_sensor_reading;
            }

            if(!WheelRightOT_WrapingAround){
                WheelRightOT_position_tracked += WheelRightOT_delta_final;
            }
            if(!WheelLeftOT_WrapingAround){
                WheelLeftOT_position_tracked += WheelLeftOT_delta_final;
            }

            WheelRightOT_last_sensor_reading = WheelRightOT_curr_sensor_reading;
            WheelLeftOT_last_sensor_reading = WheelLeftOT_curr_sensor_reading;
        }
        //telemetry.addData("WheelRightOT RAW:", WheelRightOT_curr_sensor_reading)
        //        .addData("WheelRightOT Tracked:", WheelRightOT_position_tracked);
        //telemetry.addData("WheelLeftOT RAW:", WheelLeftOT_curr_sensor_reading)
        //        .addData("WheelLeftOT Tracked:", WheelLeftOT_position_tracked);
    }



    double angleToCenter_rad = 0;
    private void AutoAimCalculations(){


        int quadrant=0;
        if(worldYPosition <= goalPosY_withOffset && worldXPosition<=goalPosX_withOffset){
            quadrant = 1;
        }
        if(worldYPosition >= goalPosY_withOffset && worldXPosition<=goalPosX_withOffset){
            quadrant = 2;
        }
        if(worldYPosition >= goalPosY_withOffset && worldXPosition>=goalPosX_withOffset){
            quadrant = 3;
        }
        if(worldYPosition <= goalPosY_withOffset && worldXPosition>=goalPosX_withOffset){
            quadrant = 4;
        }


        if(quadrant == 1){
            angleToCenter_rad = Math.atan((goalPosY_withOffset-worldYPosition)/(goalPosX_withOffset-worldXPosition));
        }
        if(quadrant == 2){
            angleToCenter_rad = -Math.atan((worldYPosition-goalPosY_withOffset)/(goalPosX_withOffset-worldXPosition));
        }
        if(quadrant == 3){
            angleToCenter_rad = Math.PI + Math.atan((worldYPosition-goalPosY_withOffset)/(worldXPosition-goalPosX_withOffset));
        }
        if(quadrant == 4){
            angleToCenter_rad =Math.PI - Math.atan((goalPosY_withOffset-worldYPosition)/(worldXPosition-goalPosX_withOffset));
        }
        //wrap around the angleToCenter_rad
        angleToCenter_rad =AngleWrap((float) angleToCenter_rad);


        if(gamepad1.x){
            if(!Gamepad2Left_BumperDown){
                rightWheelStartingPosition = dead_motor.getCurrentPosition();
                leftWheelStartingPosition = collector_motor.getCurrentPosition();
            }
            Gamepad2Left_BumperDown = true;




            if(gamepad1.x){
                if(!Gamepad2Left_BumperDown){
                    rightWheelStartingPosition = dead_motor.getCurrentPosition();
                    leftWheelStartingPosition = collector_motor.getCurrentPosition();
                }
                Gamepad2Left_BumperDown = true;

                MoveToAngle((float) angleToCenter_rad- (float) Math.toRadians(180.0f),(float) Math.toRadians(10.0f),0.5f,false);


            }else{
                Gamepad2Left_BumperDown = false;
            }



            //MoveToAngle((float) angleToCenter_rad- (float) Math.toRadians(180.0f),(float) Math.toRadians(10.0f),0.6f,0,true);



        }else{
            Gamepad2Left_BumperDown = false;
        }

    }


    float rightWheelStartingPosition = 0.0f;
    float leftWheelStartingPosition = 0.0f;


    public float AngleWrap(float angle){
        while (angle<-Math.PI){
            angle += 2*Math.PI;
        }
        while (angle>Math.PI){
            angle -= 2*Math.PI;
        }
        return angle;
    }
    private void MoveToAngle(float angle,float maxangle,float maxspeed,boolean maintain){

        float wheelLeftPower = 0;
        float wheelRightPower = 0;


        float howMuchToTurn = AngleWrap(angle-worldAngle_rad);



        int wheel = -1;

        if(AngleWrap((float) (worldAngle_rad-(angleToCenter_rad-Math.toRadians(180.0f))))
                > 0.0f){
            wheel = 0;
        }else{
            wheel = 1;
        }


        if(wheel == 0){
            wheelLeftPower = -(howMuchToTurn/maxangle)*maxspeed;
            if(!(Math.abs(howMuchToTurn) > Math.toRadians(20.0f))){
                wheelLeftPower /= 100.0f;
            }
            if(wheelLeftPower > 0.0f){
                wheelLeftPower += 0.3f;//add a minimum power
            }else{
                wheelLeftPower -= 0.3f;//add a minimum power
            }



            if(maintain){
                if(dead_motor.getCurrentPosition() < rightWheelStartingPosition){
                    wheelRightPower = -0.05f;
                }else{
                    wheelRightPower = 0.05f;
                }
            }

        }
        if(wheel == 1){
            wheelRightPower= (howMuchToTurn/maxangle)*maxspeed;

            if(!(Math.abs(howMuchToTurn) > Math.toRadians(20.0f))){
                wheelRightPower /= 100.0f;
            }


            if(wheelRightPower > 0.0f){
                wheelRightPower += 0.3f;//add a minimum power
            }else{
                wheelRightPower -= 0.3f;//add a minimum power
            }


            if(maintain){
                if(collector_motor.getCurrentPosition() < leftWheelStartingPosition){
                    wheelLeftPower = -0.05f;
                }else{
                    wheelLeftPower = 0.05f;
                }
            }


        }

        if(Math.abs(howMuchToTurn) <= Math.toRadians(1.0f)){
            wheelLeftPower = 0.0f;
            wheelRightPower = 0.0f;
        }





        float maxpower = Math.abs(maxspeed);

        //telemetry.addData("left Applied Power", wheelLeftPower);
        //telemetry.addData("right Applied Power", wheelRightPower);
        //telemetry.addData("howMuchToTurn",howMuchToTurn);

        motorPowerL = (Range.clip(wheelLeftPower,-maxpower,maxpower));
        motorPowerR = (Range.clip(wheelRightPower,-maxpower,maxpower));

    }

    private void UserAlignment(){

        ////////////////////////////////////////Alignment//////////////////////
        if(gamepad1.y) {//if gamepad1 says to align by pressing the "y" button]
            /////////////////////////////////////////////////////Y alignments////////////////////////////////////////////////////////////////////////////////
            //done
            if (worldYPosition < HalffieldLength && Math.toDegrees(worldAngle_rad) > 45.0f && Math.toDegrees(worldAngle_rad) < 135.0f) { //Aligning forwards on top wall
                worldYPosition = robotLength-worldStartingPositionY;
                worldAngle_rad = (float) Math.toRadians(90.0f);
            }

            //done
            if (worldYPosition < HalffieldLength && Math.toDegrees(worldAngle_rad) > 225.0f && Math.toDegrees(worldAngle_rad) < 315.0f) { //Aligning shooter Forwards on bottom wall
                worldYPosition = worldStartingPositionY;
                worldAngle_rad = (float) Math.toRadians(270.0f);
            }

            //done
            if (worldYPosition > HalffieldLength && Math.toDegrees(worldAngle_rad) > 45.0f && Math.toDegrees(worldAngle_rad) < 135.0f) { //Aligning Forwards on top wall
                worldYPosition = fieldLength - worldStartingPositionY;
                worldAngle_rad = (float) Math.PI / 2;
            }

            //done
            if (worldYPosition > HalffieldLength && Math.toDegrees(worldAngle_rad) > 225.0f && Math.toDegrees(worldAngle_rad) < 315.0f) { //Aligning Backwards on top wall
                worldYPosition = fieldLength-(robotLength-worldStartingPositionY);
                worldAngle_rad = (float) Math.PI * 1.5f;
            }

            //////////////////////////////////////////////////////////// x alignments /////////////////////////////////////////////////////////////////////

            //done
            if (worldXPosition < HalffieldLength && Math.toDegrees(worldAngle_rad) > 135.0f && Math.toDegrees(worldAngle_rad) < 225.0f) { //Aligning Backwards on Left wall
                worldXPosition = worldStartingPositionY;
                worldAngle_rad = (float) Math.toRadians(180.0f);
            }

            //done
            if (worldXPosition < HalffieldLength && (Math.toDegrees(worldAngle_rad) > 315.0f || Math.toDegrees(worldAngle_rad) < 45.0f)) { //Aligning Forwards on Left wall
                worldXPosition = robotLength-worldStartingPositionY;
                worldAngle_rad =  0.0f;
            }

            //done
            if (worldXPosition > HalffieldLength && Math.toDegrees(worldAngle_rad) > 135.0f && Math.toDegrees(worldAngle_rad) < 225.0f) { //Aligning forwards on right wall
                worldXPosition = fieldLength-(robotLength-worldStartingPositionY);
                worldAngle_rad = (float) Math.PI;
            }

            //done
            if (worldXPosition > HalffieldLength && (Math.toDegrees(worldAngle_rad) > 315.0f || Math.toDegrees(worldAngle_rad) < 45.0f)){ //Aligning Backwards on right wall
                worldXPosition = fieldLength-worldStartingPositionY;
                worldAngle_rad = 0.0f;
            }
        }
    }

    private void AuxiliaryShooterConstantAdjustment(){
        telemetry.addData("Shooter constant = ", shooterConstant);
        if(gamepad2.dpad_right){
            if(!left_bumper_down){
                shooterConstant -= 0.005f;
                left_bumper_down = true;
            }
        }else{
            left_bumper_down = false;
        }
        if(gamepad2.dpad_right){
            if(!right_bumper_down){
                shooterConstant += 0.005f;
                right_bumper_down = true;
            }
        }else{
            right_bumper_down = false;
        }

    }
}
