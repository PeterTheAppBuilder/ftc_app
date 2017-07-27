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
package OldRobot;

import android.os.SystemClock;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import OldRobot.Globals;
import OldRobot.MyTeleOp;

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

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Shoot 2, End on Middle", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
@Disabled

public class TwoS_middle_end extends OpMode
{
    ////////////////////DEFINE VARIABLES//////////////////////////////////////


    public float DistanceToTargetInches=0.0f;
    public float DistanceToTargetCentimeters=0.0f;

    public DcMotor wheelRight;
    public DcMotor wheelLeft;
    public DcMotor shooter_motor1;
    public DcMotor shooter_motor2;
    public DcMotor collector_motor;
    public DcMotor dead_motor;


    public Servo aim_servo1;//the first aim servo position, it is controlled by aimServoPositionMaster
    public DcMotorController ShooterEncoder;
    public DcMotorController OmniWheelEncoders;
    public Servo triggerServo;
    public Servo beacon_servo_1;
    public Servo beacon_servo_2;
    public OpticalDistanceSensor beacon_line_sensor;
    public Servo deployWheelServo1;
    public Servo deployWheelServo2;
    public ColorSensor colorSensor1;
    public ColorSensor colorSensor2;

    public TouchSensor triggerSensor;

    public float capBallDeployPosition = 0.95f;
    public Servo capBallDeploy;


    public float aimServoPosition1;//the aiming servo position
    public float aimServoPosition2;//the second aiming servo position.
    public float aimServoPositionMaster = 0.223f;

    public long blockStartTime = 0;


    boolean isAccelerating = false;//if you are currently accelerating
    boolean isDecelerating = false;//if you are currently decelerating
    boolean DoneAccelerating = false;



    int beacon_color = 0;



    boolean wheelsSpinning = false;


    long lastDecelUpdate = 0;//the time of the last update in the decel algorithm
    long lastAccelUpdate = 0;//the time of the last update in the accel algorithm

    public float moveScalingFactor = 45.423f;//when you divide the encoder values by this it returns cm
    public float SpeedScale = 0.4f;//percent of total speed to be maximum speez
    public float target_shooter_power = 0;//the speed of the shooter when it accelerates
    public float shooter_current_power = 0;//the current speed the shooter is at
    public float aimServoPosition= Globals.shooterVertical;//the aiming servo position. It starts at max

    public float angleSaved = -1.0f;


    float beaconReleasePoint = 152.0f;

    public float worldAngle_rad = AngleWrap((float) Math.toRadians(45));//start the world angle at 90 degrees (upward)

    public float fieldLength = 358.775f;//the length of the feild in cm
    public float HalffieldLength = fieldLength/2.0f;//the length of half field
    float robotLength = 44.196f;//robot height in centimeters


    //the robot doesn't start in the exact corner, so measurements needed to be taken
    public float worldXPosition;
    public float worldYPosition;



    //the position of the goal in centimeters
    public float goalPositionX;
    public float goalPositionY;

    public boolean goForThirdBall = false;


    public float wheelLeftLast = 0;
    public float wheelRightLast = 0;

    public float wheelLeftLastPower = 0.0f;
    public float wheelRightLastPower = 0.0f;

    public long shooterStartTime = 0;//the time in milis that the shooter was started

    public float aimServoLoadPosition = 0.68f;




    private enum progStates {
        initializeStage,
        start_shooter,
        reload1,
        move_before_shoot,
        wait_for_rev_up,
        aim1,
        waitforaim1,
        fire1,
        waitforfire1,
        reload2,
        waitforreload2,
        aim2,
        waitforaim2,
        fire2,
        waitforlastfire,
        move_to_platform,
        finished;

        public progStates getNext() {
            return this.ordinal() < progStates.values().length - 1
                    ? progStates.values()[this.ordinal() + 1]
                    : null;
        }

        public int getStateNum() {
            return this.ordinal();
        }
    }


    public progStates programStage = progStates.initializeStage ;//the part of the program we are at







    public enum trigStates {
        stop,
        state_reload_begin,
        state_reload_busy,
        state_fire,
        state_firing
    }

    public trigStates triggerStage = trigStates.stop;


    public boolean StageFinished = true;
    public boolean triggerStageFinished = true;

    float blockStartingX = 0.0f;
    float blockStartingY = 0.0f;
    float blockStartingAngle = 0.0f;

    double angleToCenter = 0;

    float shooterServoK = 0.65f;

    int shotsFired = 0;

    boolean blueside = true;

    float drivetoBeacon2Speed = 0.2f;



    //LegacyModule legacyModule;
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    ModernRoboticsI2cRangeSensor rangeSensor1;
    ModernRoboticsI2cRangeSensor rangeSensor2;


    @Override
    public void init() {



        beacon_line_sensor=(ModernRoboticsAnalogOpticalDistanceSensor) hardwareMap.get("LSA_Sensor1");
        beacon_line_sensor.enableLed(true);

        colorSensor2 = hardwareMap.colorSensor.get("colorSensorBlue");
        colorSensor2.setI2cAddress( I2cAddr.create8bit(0x3a));
        colorSensor2.enableLed(false);

        colorSensor1 = hardwareMap.colorSensor.get("colorSensorRed");
        colorSensor1.setI2cAddress( I2cAddr.create8bit(0x3c));
        colorSensor1.enableLed(false);

        triggerSensor = hardwareMap.touchSensor.get("triggerSensor");

        wheelRight = hardwareMap.dcMotor.get("motor_1");
        wheelLeft = hardwareMap.dcMotor.get("motor_2");
        wheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        wheelLeft.setDirection(DcMotor.Direction.REVERSE);
        wheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        OmniWheelEncoders = hardwareMap.dcMotorController.get("collector and aim");

        ShooterEncoder = hardwareMap.dcMotorController.get("shooter controller");

        shooter_motor1 = hardwareMap.dcMotor.get("shooter");
        shooter_motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);//when applying zero power to the shooter, coast
        shooter_motor1.setDirection(DcMotor.Direction.REVERSE);

        shooter_motor2 = hardwareMap.dcMotor.get("motor_4");
        shooter_motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);//when applying zero power to the shooter, coast

        //these 2 motors have the encoders for the omni wheels plugged into them (we can't do speed control with them)
        collector_motor = hardwareMap.dcMotor.get("collector");
        dead_motor = hardwareMap.dcMotor.get("aim");
        collector_motor.setDirection(DcMotor.Direction.REVERSE);



        aim_servo1 = hardwareMap.servo.get("servo_2");
        triggerServo=hardwareMap.servo.get("servo_6");
        beacon_servo_1=hardwareMap.servo.get("servo_3");
        beacon_servo_2=hardwareMap.servo.get("servo_4");
        deployWheelServo1 =hardwareMap.servo.get("servo_1");
        deployWheelServo2 = hardwareMap.servo.get("servo_5");
        capBallDeploy=hardwareMap.servo.get("servo_5");
        capBallDeploy.setPosition(capBallDeployPosition);

        aim_servo1.setPosition(Globals.startAimPosition);




        lastDecelUpdate = 0;//the time of the last update in the decel algorithm
        lastAccelUpdate = 0;//the time of the last update in the accel algorithm
        SpeedScale = 0.4f;//percent of total speed to be maximum speez
        target_shooter_power = 0;//the speed of the shooter when it accelerates
        shooter_current_power = 0;//the current speed the shooter is at










        wheelLeftLast = dead_motor.getCurrentPosition();
        wheelRightLast= collector_motor.getCurrentPosition();

        //the position of the goal in centimeters
        goalPositionX = HalffieldLength + (float) Math.cos(Math.toRadians(-45.0f))*15.5f*2.54f;
        goalPositionY = HalffieldLength + (float) Math.sin(Math.toRadians(-45.0f))*15.5f*2.54f;


        shooterStartTime = 0;//the time in milis that the shooter was started

        aimServoLoadPosition = 0.68f;





        StageFinished = true;

        blockStartingX = 0.0f;
        blockStartingY = 0.0f;
        blockStartingAngle = 0.0f;

        angleToCenter = 0;

        shooterServoK = 0.65f;

        shotsFired = 0;
        Aim_and_trigger_servo_movement();
        deployWheelServo1.setPosition(Globals.deployWheelServo1Up);
        deployWheelServo2.setPosition(Globals.deployWheelServo2Up);

        beacon_servo_1.setPosition(Globals.beacon_servo1_off);
        beacon_servo_2.setPosition(Globals.beacon_servo2_off);


    }


    float snipingServo1Position = 0.0f;
    float snipingServo2Position = 1.0f;

    long waitTime = 5000;
    boolean dpad_up = false;
    boolean dpad_down = false;


    @Override
    public void init_loop() {

        telemetry.addData("Wait For: " + waitTime/1000 + " " + "Seconds","");

        aim_servo1.setPosition(Globals.startAimPosition);


        //Give the user the option to adjust the starting wait time
        if(gamepad1.dpad_up){
            if(!dpad_up){
                waitTime += 1000;
                dpad_up = true;
            }
        }else{
            dpad_up = false;
        }

        if(gamepad1.dpad_down){
            if(!dpad_down){
                if(waitTime > 0){
                    waitTime -= 1000;
                }

                dpad_down = true;
            }
        }else{
            dpad_down = false;
        }


        telemetry.addData("intensity",beacon_line_sensor.getRawLightDetected());


        telemetry.addData("color sensor 1", colorSensor1.blue());
        telemetry.addData("color sensor 2", colorSensor2.blue());

        triggerServo.setPosition(0.5f);//stop the servo

        if(gamepad1.a){
            //set our position to red
            blueside = false;
        }
        if(gamepad1.b){
            blueside=true;//we will start on blue
        }

        if(gamepad1.left_bumper){
            goForThirdBall = true;
        }
        if(gamepad1.right_bumper){
            goForThirdBall = false;
        }

        if(goForThirdBall){
            telemetry.addData("GOING FOR THIRD BALL!","");
        }else{
            telemetry.addData("NOT GOING FOR THIRD BALL","");
        }



        if(blueside){
            telemetry.addData("We are on the BLUE side","");
        }else{
            telemetry.addData("we are on the RED side","");
        }



        String colors = String.format("Ls1 b: %.1f Ls1 r: %.1f Ls2 b: %.1f Ls2 r %.1f", (float) colorSensor1.blue(), (float) colorSensor1.red(),(float) colorSensor2.blue(), (float) colorSensor2.red());
        telemetry.addData(colors, "");
        String XandY = String.format("X Pos: %.1f Y Pos: %.1f", worldXPosition,worldYPosition);

        telemetry.addData("worldAngle_rad",Math.toDegrees(worldAngle_rad));


        wheelLeftLast = dead_motor.getCurrentPosition();
        wheelRightLast= collector_motor.getCurrentPosition();



    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {


        runtime.reset();
    }

    float csr1 =-1.0f;
    float csb1 = -1.0f;
    float csg1 = -1.0f;



    boolean reloaded = false;

    float leftWheelStartingPosition = -1.0f;
    float rightWheelStartingPosition = -1.0f;

    public void initializeStateVariables(){
        StageFinished = false;
        //initialization
        blockStartingX = worldXPosition;
        blockStartingY = worldYPosition;
        leftWheelStartingPosition = collector_motor.getCurrentPosition();
        rightWheelStartingPosition = dead_motor.getCurrentPosition();
        blockStartingAngle = worldAngle_rad;
        blockStartTime = SystemClock.uptimeMillis();
    }

    public void initializeMasterVariables(){
        StageFinished = true;

        aimServoPosition= Globals.aimLoadPosition;//the aiming servo position. It starts at max

        isAccelerating = false;//if you are currently accelerating
        isDecelerating = false;//if you are currently decelerating
        DoneAccelerating = false;
        wheelsSpinning = false;



        goalPositionY = HalffieldLength + (float) Math.sin(Math.toRadians(-45.0f))*15.5f*2.54f;
        if(blueside){
            goalPositionX = HalffieldLength + (float) Math.cos(Math.toRadians(-45.0f))*15.5f*2.54f;

            worldXPosition=fieldLength- 260.0f;
            worldYPosition=42.0f;
            worldAngle_rad = AngleWrap((float) Math.toRadians(45.0f));

        }else{
            goalPositionX = HalffieldLength + (float) Math.cos(Math.toRadians(-135.0f))*15.5f*2.54f;

            worldXPosition= 260.0f;
            worldYPosition = 42.0f;
            worldAngle_rad = AngleWrap((float) Math.toRadians(-45.0f));
        }

        aimServoPositionMaster= 0.2174f;
        triggerStage = trigStates.stop;

        StageFinished = true;
        programStage = programStage.getNext();

    }





    public void activateCollector(){
        collector_motor.setPower(-0.8f);
    }
    public void stopCollector(){
        collector_motor.setPower(0.0f);
    }




    @Override
    public void loop() {


        if(programStage == progStates.finished){
            aimServoPositionMaster = 0.0f;
            beacon_servo_1.setPosition(Globals.beacon_servo1_on);
            beacon_servo_2.setPosition(Globals.beacon_servo2_on);
        }


        //calculate the distance to the target with the distance formula
        DistanceToTargetCentimeters = (float) Math.sqrt(Square(goalPositionX-worldXPosition)+Square(goalPositionY-worldYPosition));
        DistanceToTargetInches = DistanceToTargetCentimeters/2.54f;
        DistanceToTargetInches -= 5.0f;//subtract a bit to get it in the center


        telemetry.addData("", String.format("Stage %d, x: %.1f, y: %.1f, angle %.1f",programStage.getStateNum(),worldXPosition,worldYPosition, worldAngle_rad));

        ///display the color
        telemetry.addLine()
                .addData("color = ", beacon_color)
                .addData("red",csr1)
                .addData("blue",csb1);
        telemetry.update();

        telemetry.addData("programStage",programStage);
        PositioningCalculations();
        AutoAimCalculations();
        ShooterSpeedCalculations();
        Aim_and_trigger_servo_movement();



        telemetry.addData("X",worldXPosition);
        telemetry.addData("Y", worldYPosition);
        telemetry.addData("Angle",Math.toDegrees(worldAngle_rad));


        telemetry.addData("Stage", programStage);





        if(triggerStage == trigStates.stop){
            //stop the servo
            triggerServo.setPosition(0.5f);
        }
        if(triggerStage == trigStates.state_reload_begin) {
            triggerServo.setPosition(0.2f);

            triggerStage = trigStates.state_reload_busy;

        }

        if(triggerStage == trigStates.state_reload_busy) {
            if(triggerSensor.isPressed()){

                triggerServo.setPosition(0.5f);
                reloaded = true;
            }else{
                reloaded = false;
                triggerServo.setPosition(0.2f);
            }
        }
        if(triggerStage == trigStates.state_fire) {
            triggerServo.setPosition(0.0f);
            if(triggerSensor.isPressed()){
                //only advance to stage firing if the trigger sensor is pressed and ready.
                triggerStage = trigStates.state_firing;
            }
        }
        if(triggerStage == trigStates.state_firing){
            //wait_for_rev_up for the touch sensor to be unpresseed, and commence reload.
            if(!triggerSensor.isPressed()){
                triggerStage = trigStates.state_reload_begin;
            }
        }





        if(programStage == progStates.initializeStage){
            initializeMasterVariables();

        }




        boolean triggerUndeployed1 = false;




        //////START THE BEAST////////////
        if(programStage == progStates.start_shooter){
            if(StageFinished){//if this is the first update of this block
                initializeStateVariables();
                wheelsSpinning = true;
                StageFinished = true;
                programStage =programStage.getNext();
            }
        }
        //////////RELOAD
        if(programStage == progStates.reload1 || programStage == progStates.reload2){
            if(StageFinished){//if this is the first update of this block
                initializeStateVariables();
                triggerStage = trigStates.state_reload_begin;
                aimServoPositionMaster = Globals.aimLoadPosition;
                programStage = programStage.getNext();
                StageFinished= true;
            }
        }
        if(programStage == progStates.move_before_shoot){
            if(StageFinished){
                initializeStateVariables();
            }
            move(50.0f,false,0.5f,0.2f,true,0.0f);
        }


        if(programStage == progStates.waitforreload2){
            wait(5000);
        }

        if(programStage == progStates.wait_for_rev_up){
            wait((int) waitTime);
        }










        //////////YOTO YAIM
        if(programStage == progStates.aim1 || programStage == progStates.aim2){
            if(StageFinished){//if this is the first update of this block
                initializeStateVariables();

                //calculate the Aim based on the curve fit we did
                float calculatedAim =  Globals.shooterConstant-(Globals.shooterSlope*DistanceToTargetCentimeters);
                telemetry.addData("calculatedAim=",calculatedAim);
                aimServoPositionMaster = calculatedAim;

                programStage=programStage.getNext();
                StageFinished = true;
            }
        }


        //wait_for_rev_up for the servo to tilt

        if(programStage == progStates.waitforaim1 ||
                programStage == progStates.waitforaim2) {
            wait(1000);
        }


        //////////FIRE!!!!!!!!!!!!!!!!!!!!!!!!!!!!//////////////////////////////////////////////////////////////////////////////
        if(programStage == progStates.fire1 || programStage == progStates.fire2){
            if(StageFinished){//if this is the first update of this block
                initializeStateVariables();
                triggerStage = trigStates.state_fire;

                StageFinished = true;
                if(programStage == progStates.fire2 && !goForThirdBall){
                    programStage = progStates.waitforlastfire;
                }else{
                    programStage = programStage.getNext();
                }

            }
        }

        if(programStage == progStates.waitforfire1 || programStage == progStates.waitforlastfire){
            wait(600);
        }



        /*
        if(programStage == progStates.turn_to_platform){
            if(StageFinished){
                initializeStateVariables();
                triggerStage = trigStates.stop;
                stopCollector();
                wheelsSpinning = false;

            }
            if(blueside){
                float xDifference = (HalffieldLength+30.5f)-worldXPosition;
                float yDifference = (HalffieldLength+0.0f)-worldYPosition;
                float angleToPlatform = (float) Math.atan2(yDifference,xDifference);
                MoveToAngleAbsolute(0.0f,(float) Math.toRadians(15.0f), 0.11f,0,true,true,programStage.getNext());
            }else{
                float xDifference = (HalffieldLength-30.5f)-worldXPosition;
                float yDifference = (HalffieldLength+0.0f)-worldYPosition;
                float angleToPlatform = (float) Math.atan2(yDifference,xDifference);
                telemetry.addData("XDiff",xDifference);
                telemetry.addData("angleToPlatform",Math.toDegrees(angleToPlatform));

                MoveToAngleAbsolute( (float) Math.toRadians(180.0f-0.0f),(float) Math.toRadians(15.0f), 0.11f,1,true,true,programStage.getNext());
            }
        }
        if(programStage == progStates.move_to_platform){
            if(StageFinished){
                initializeStateVariables();

                blockStartingAngle = (float) Math.toRadians(0.0f);

            }
            double howMuchToMove;
            if(blueside){
                howMuchToMove = Math.sqrt(Square((HalffieldLength+30.5f)-blockStartingX) +
                        Square((HalffieldLength)-blockStartingY));
            }else{
                howMuchToMove = Math.sqrt(Square((HalffieldLength-30.5f)-blockStartingX) +
                        Square((HalffieldLength)-blockStartingY));

            }
            move((float) howMuchToMove+10.0f,false,0.5f,0.0f,true,0.0f);
        }
        */



        if(programStage == progStates.move_to_platform){
            if(StageFinished){
                initializeStateVariables();

                triggerStage = trigStates.stop;

            }
            double howMuchToMove;
            if(blueside){
                howMuchToMove = Math.sqrt(Square((HalffieldLength+30.5f)-blockStartingX) +
                        Square((HalffieldLength)-blockStartingY));
            }else{
                howMuchToMove = Math.sqrt(Square((HalffieldLength-30.5f)-blockStartingX) +
                        Square((HalffieldLength-30.5f)-blockStartingY));

            }
            move((float) howMuchToMove-50.0f,false,0.5f,0.2f,true,0.0f);
        }



        MyTeleOp.worldAngleAutonomous = worldAngle_rad;
        MyTeleOp.worldXAutonomous = worldXPosition;
        MyTeleOp.worldYAutonomous = worldYPosition;
        MyTeleOp.goalXPosAutonomous = goalPositionX;
        MyTeleOp.goalYPosAutonomous = goalPositionY;



    }






    public void Aim_and_trigger_servo_movement(){

        aimServoPositionMaster = Range.clip(aimServoPositionMaster,0.0f,1.0f);
        aim_servo1.setPosition(aimServoPositionMaster);
    }



    public void wait(int miliseconds){
            if(StageFinished){//if this is the first update of this block
                initializeStateVariables();
            }
            if(SystemClock.uptimeMillis()-blockStartTime > miliseconds){
                programStage = programStage.getNext();
                StageFinished = true;
            }
    }

    float distance_traveled = 0.0f;
    public void move(float target_distance,boolean direction,float speed,float dither,boolean Break,float decel_dist) {

        if(target_distance < 0){
            direction = !direction;
        }
        distance_traveled = (float) Math.sqrt(Square(worldXPosition-blockStartingX)+Square(worldYPosition-blockStartingY));

        telemetry.addData("target distance",target_distance/2.54f);

        float tempWorldAngle = worldAngle_rad;
        tempWorldAngle = AngleWrap(tempWorldAngle);

        if(     (decel_dist > 0) &&
                (Math.abs(target_distance)-Math.abs(distance_traveled) < decel_dist )) {
            speed = ((decel_dist-(Math.abs(target_distance) - Math.abs(distance_traveled))) / decel_dist)
                    * 0.7f * speed + (0.3f * speed);
        }

        //if we still need to go forwards more
        if(Math.abs(distance_traveled) < Math.abs(target_distance)){
            if(!direction){
                if(tempWorldAngle > blockStartingAngle){
                    wheelLeft.setPower(1.0f*speed);
                    wheelRight.setPower((1.0f-dither)*speed);
                }else{
                    wheelLeft.setPower((1.0f-dither)*speed);
                    wheelRight.setPower(1.0f*speed);
                }
            }
            else{
                if(tempWorldAngle < blockStartingAngle){
                    wheelLeft.setPower(-1.0f*speed);
                    wheelRight.setPower((-1.0f+dither)*speed);
                }else{
                    wheelLeft.setPower((-1.0f+dither)*speed);
                    wheelRight.setPower(-1.0f*speed);

                }
            }
        }else{//if we have finished moving
            programStage = programStage.getNext();
            StageFinished = true;
            if(Break){
                wheelRight.setPower(0.0f);
                wheelLeft.setPower(0.0f);
            }

        }
    }


    public void moveCurve(float target_curve_distance,float speedL, float speedR,float equalize,boolean Break) {

        if(target_curve_distance < 0){
            float speedLtemp = speedL;
            speedL = -speedR;
            speedR = -speedLtemp;
        }
        float rightDelta = dead_motor.getCurrentPosition()-rightWheelStartingPosition;
        float leftDelta = collector_motor.getCurrentPosition()-leftWheelStartingPosition;

        distance_traveled = (leftDelta + rightDelta)/(2*moveScalingFactor);

        float percentGoalLeft = speedL/speedR;

        float currentPercentLeft = leftDelta/rightDelta;




        float currentSpeedLeft = speedL;
        float currentSpeedRight = speedR;


        if(currentPercentLeft < percentGoalLeft){
            currentSpeedLeft += equalize * (speedL/Math.abs(speedL));
            currentSpeedRight -= equalize * (speedR/Math.abs(speedR));
        }else{
            currentSpeedLeft -= equalize * (speedL/Math.abs(speedL));
            currentSpeedRight += equalize * (speedR/Math.abs(speedR));
        }



        //if we still need to go forwards more
        if(Math.abs(distance_traveled) < Math.abs(target_curve_distance)){

            wheelLeft.setPower(currentSpeedLeft);
            wheelRight.setPower(currentSpeedRight);


        }else{//if we have finished moving
            programStage = programStage.getNext();
            StageFinished = true;
            if(Break){
                wheelRight.setPower(0.0f);
                wheelLeft.setPower(0.0f);
            }

        }
        telemetry.addData("Left wheel motor speed",wheelLeft.getPower());
        telemetry.addData("Right wheel motor speed",wheelRight.getPower());


    }






    public void moveWithUltrasonic(float target_distance,boolean direction,float speed,float distanceAwayFromWall) {

        if(target_distance < 0){
            direction = !direction;
        }
        distance_traveled = (float) Math.sqrt(Square(worldXPosition-blockStartingX)+Square(worldYPosition-blockStartingY));

        telemetry.addData("target distance",target_distance/2.54f);

        float tempWorldAngle = worldAngle_rad;
        tempWorldAngle = AngleWrap(tempWorldAngle);


        //if we still need to go forwards more
        if(Math.abs(distance_traveled) < Math.abs(target_distance)){
            if(direction){
                if(rangeSensor1.cmUltrasonic() < distanceAwayFromWall){
                    wheelRight.setPower(1.0f*speed);
                    wheelLeft.setPower(0.6f*speed);
                }else{
                    wheelRight.setPower(0.6f*speed);
                    wheelLeft.setPower(1.0f*speed);
                }
            }
            else{
                if(rangeSensor1.cmUltrasonic() > distanceAwayFromWall){
                    wheelRight.setPower(-1.0f*speed);
                    wheelLeft.setPower(-0.6f*speed);
                }else{
                    wheelRight.setPower(-0.6f*speed);
                    wheelLeft.setPower(-1.0f*speed);
                }
            }
        }else{//if we have finished moving
            programStage =programStage.getNext();
            StageFinished = true;
            wheelRight.setPower(0.0f);
            wheelLeft.setPower(0.0f);
        }
    }
    public float AngleWrap(float angle){
        while (angle<-Math.PI){
            angle += 2*Math.PI;
        }
        while (angle>Math.PI){
            angle -= 2*Math.PI;
        }
        return angle;
    }

    @Override
    public void stop() {
    }

    public float Square(float number){
        return number*number;
    }

    private void ShooterSpeedCalculations(){

        if(wheelsSpinning){
            if(!isAccelerating && !DoneAccelerating){//if this is the first update that the bbutton was down..
                //initialize our clock
                lastAccelUpdate = SystemClock.uptimeMillis();
                //Don't do speed control when accelerating
                shooter_motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shooter_motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                isAccelerating = true;
                isDecelerating = false;

                target_shooter_power = 0.2f;//accelerate to this and once done,

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


    private void PositioningCalculations(){

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
    }

    private void AutoAimCalculations(){
        int quadrant=0;
        if(worldYPosition <= goalPositionY && worldXPosition<=goalPositionX){
            quadrant = 1;
        }
        if(worldYPosition >= goalPositionY && worldXPosition<=goalPositionX){
            quadrant = 2;
        }
        if(worldYPosition >= goalPositionY && worldXPosition>=goalPositionX){
            quadrant = 3;
        }
        if(worldYPosition <= goalPositionY && worldXPosition>=goalPositionX){
            quadrant = 4;
        }



        if(quadrant == 1){
            angleToCenter = Math.atan((goalPositionY-worldYPosition)/(goalPositionX-worldXPosition));
        }
        if(quadrant == 2){
            angleToCenter = -Math.atan((worldYPosition-goalPositionY)/(goalPositionX-worldXPosition));
        }
        if(quadrant == 3){
            angleToCenter = Math.PI + Math.atan((worldYPosition-goalPositionY)/(worldXPosition-goalPositionX));
        }
        if(quadrant == 4){
            angleToCenter =Math.PI - Math.atan((goalPositionY-worldYPosition)/(worldXPosition-goalPositionX));
        }
        //wrap around the angleToCenter_rad
        if(angleToCenter >= Math.PI * 2.0f){
            angleToCenter -= Math.PI * 2.0f;
        }
        if(angleToCenter < 0){
            angleToCenter += Math.PI * 2.0f;
        }

        //telemetry.addData("", String.format("Ang to Goal %.1f, World Ang %.1f",(float)Math.toDegrees(angleToCenter_rad),(float) Math.toDegrees(worldAngle_rad)));
    }
    private void MoveToAngleAbsolute(float angle,float maxangle,float maxspeed,int wheel,boolean maintain,boolean Break,progStates advanceState){

        float wheelLeftPower = 0;
        float wheelRightPower = 0;


        float howMuchToTurn = AngleWrap(angle- worldAngle_rad);



        if(wheel == 0){
            wheelLeftPower = -(howMuchToTurn/maxangle)*maxspeed;

            if(wheelLeftPower > 0.0f){
                wheelLeftPower += 0.05f;//add a minimum power
            }else{
                wheelLeftPower -= 0.05f;//add a minimum power
            }



            if(maintain){
                if(dead_motor.getCurrentPosition() < rightWheelStartingPosition){
                    wheelRightPower = -0.15f;
                }else{
                    wheelRightPower = 0.15f;
                }
            }else{
                wheelRightPower = 0.0f;
            }

        }
        if((wheel == 1) || (wheel == 2)){
            wheelRightPower= (howMuchToTurn/maxangle)*maxspeed;
            if(wheelRightPower > 0.0f){
                wheelRightPower += 0.05f;//add a minimum power
            }else{
                wheelRightPower -= 0.05f;//add a minimum power
            }


            if(maintain){
                if(collector_motor.getCurrentPosition() < leftWheelStartingPosition){
                    wheelLeftPower = -0.15f;
                }else{
                    wheelLeftPower = 0.15f;
                }
            }else{
                wheelLeftPower = 0.0f;
            }

            if( wheel == 2){
                wheelLeftPower = -1.0f * wheelRightPower;
            }

        }




        float maxpower = Math.abs(maxspeed);


        wheelLeft.setPower(Range.clip(wheelLeftPower,-maxpower,maxpower));
        wheelRight.setPower(Range.clip(wheelRightPower,-maxpower,maxpower));


        telemetry.addData("How much to turn:",Math.toDegrees(howMuchToTurn));
        //if we are within one degree of our target angle, finish and move to the next thing and brake
        if(Math.abs(howMuchToTurn)<=Math.toRadians(1.0f)){
            if(Break){
                wheelLeft.setPower(0.0f);
                wheelRight.setPower(0.0f);
            }



            programStage = advanceState;
            StageFinished = true;
        }
    }

}
