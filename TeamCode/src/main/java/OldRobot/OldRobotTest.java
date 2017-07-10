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
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.opencv.core.Point;

import OldRobot.Globals;
import ftc.pathfinder.pathFinder;
import ftc.vision.FrameGrabber;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "MyTeleOp 2.5", group = "Iterative Opmode")
// @Autonomous(...) is the other common choice

public class OldRobotTest extends OpMode {

    public enum progStates {

        /*
        turn_to_center_vortex,
        move_before_shooting,
        reloadn1,
        waitforreloadn1,
        aimn1,
        waitforaimn1,
        firen1,
        waitforfiren1,
        reloadn2,
        waitforreloadn2,
        aimn2,
        waitforaimn2,
        firen2,
        */
        waitforfire,

        searchingForBalls,
        lockingIteration1,
        lockingIteration2,
        pathFollowingStage,
        driveBackABit,
        pointToGoal,
        driveToProximity,
        ////Shooting Sequence:
        reload,
        waitforreload,
        aim,
        waitforaim,
        fire;


        public progStates getNext() {
            return this.ordinal() < progStates.values().length - 1
                    ? progStates.values()[this.ordinal() + 1]
                    : null;
        }
    }
    ////////////////////DEFINE VARIABLES//////////////////////////////////////
    private float closestScanBallAngle;
    private float closestScanBallDistance;
    public progStates programStage = progStates.searchingForBalls;//progStates.turn_to_center_vortex;
    public boolean StageFinished = true;
    public float DistanceToTargetCentimeters = 0.0f;
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
    public Servo deployWheelServo1;
    public Servo deployWheelServo2;
    public DcMotorController ShooterEncoder;
    public DcMotorController OmniWheelEncoders;
    public Servo triggerServo;
    public TouchSensor triggerSensor;
    boolean isAccelerating = false;//if you are currently accelerating
    boolean isDecelerating = false;//if you are currently decelerating
    boolean DoneAccelerating = false;
    boolean wheelsSpinning = false;
    long lastDecelUpdate = 0;//the time of the last update in the decel algorithm
    long lastAccelUpdate = 0;//the time of the last update in the accel algorithm
    public final float moveScalingFactor = 45.423f;//when you divide the encoder values by this it returns cm
    public float target_shooter_power = 0;//the speed of the shooter when it accelerates
    public float shooter_current_power = 0;//the current speed the shooter is at
    public float aimServoPositionMaster = 0.0f;//the master position
    public boolean ypressed = false;
    public boolean ydown = false;
    public float worldStartingPositionX = 23.18f;
    public float worldStartingPositionY = 8.5f;
    //the robot doesn't start in the exact corner, so measurements needed to be taken
    public float worldXPosition = worldStartingPositionX;
    public float worldYPosition = worldStartingPositionY;
    public static float worldAngle_rad = (float) Math.PI / 2.0f;//start the world angle at 90 degrees (upward)
    long All_telemetry_trackers_lastUpdate = 0;
    public static final int All_telemetry_trackers_pollingRate = 5;
    public static final float All_telemetry_trackers_wrapAroundThreshold = 0.3f;
    float WheelRightOT_curr_sensor_reading = 0.0f;
    float WheelRightOT_last_sensor_reading = 0.0f;
    float WheelRightOT_position_tracked = 0.0f;
    float WheelRightOT_lastvalidreading = 0.0f;
    boolean WheelRightOT_WrapingAround = false;
    float WheelLeftOT_curr_sensor_reading = 0.0f;
    float WheelLeftOT_last_sensor_reading = 0.0f;
    float WheelLeftOT_position_tracked = 0.0f;
    float WheelLeftOT_lastvalidreading = 0.0f;
    boolean WheelLeftOT_WrapingAround = false;
    public float fieldLength = 358.775f;//the length of the feild in cm
    public float HalffieldLength = fieldLength / 2.0f;//the length of half field
    float robotLength = 44.5f;//robot height in centimeters
    //the position of the goal in centimeters
    public float goalPositionX = HalffieldLength + (float) Math.cos(Math.toRadians(-45.0f)) * 15.5f * 2.54f;
    public float goalPositionY = HalffieldLength + (float) Math.sin(Math.toRadians(-45.0f)) * 15.5f * 2.54f;
    public float wheelLeftLast = 0;
    public float wheelRightLast = 0;
    public enum trigStates {
        stop,
        state_reload_begin,
        state_reload_busy,
        state_fire,
        state_firing
    }
    float rightWheelStartingPosition = 0.0f;
    float leftWheelStartingPosition = 0.0f;
    public static double angleToTurnVision_rad;
    public static double currentLockAngleToVision_rad;
    public static double currentLockYPositionToRobot;
    static double ballYRelativeToRobot;
    static double ballXRelativeToRobotCenter;
    double ballXRelativeToCenterPercent = 0.0;
    final float screenXSize = 800.0f;
    final float screenYSize = 480.0f;
    public boolean finishedLockingTarget = false;

    public void acquireNewTarget() {
        finishedLockingTarget = true;
    }

    boolean foundBall = false;

    public void newTestBallFinderFunction() {

        if (FrameGrabber.bestBallX != 0 && !FrameGrabber.uptoDate) {
            foundBall = true;
            int ballX = FrameGrabber.bestBallX;
            int ballY = FrameGrabber.bestBallY;

            ballYRelativeToRobot = 16.40639 + ((140765900 - 16.40639) / (1 + Math.pow((ballY / 0.002833515), 1.330388)));
            ballYRelativeToRobot *= 2.54;
            double cameraViewSizeAtY = (1.259259 * ballYRelativeToRobot) + 23.85185;
            double cameraCenterViewSizeAtY = cameraViewSizeAtY / 2.0;
            ballXRelativeToCenterPercent = (ballX - (screenXSize / 2.0f)) / (screenXSize / 2.0f);
            ballXRelativeToRobotCenter = ballXRelativeToCenterPercent * cameraCenterViewSizeAtY;


            float phonePositionRelativeToRobotOriginX = -4.5f;
            ballXRelativeToRobotCenter += phonePositionRelativeToRobotOriginX;


            //calculate angle
            angleToTurnVision_rad = worldAngle_rad + (Math.atan2(ballYRelativeToRobot, ballXRelativeToRobotCenter) - Math.toRadians(90.0f));


            FrameGrabber.uptoDate = true;


        } else {
            foundBall = false;
        }
        if (finishedLockingTarget) {//only update the target angle if we have finished locking on
            if (foundBall) {
                currentLockAngleToVision_rad = angleToTurnVision_rad;
                currentLockYPositionToRobot = ballYRelativeToRobot;
                finishedLockingTarget = false;
            }
        }


        telemetry.addData("Angle Vision", Math.toDegrees(currentLockAngleToVision_rad));
    }

    public ColorSensor colorSensor1;
    public trigStates triggerStage = trigStates.stop;
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    public ModernRoboticsI2cRangeSensor wallDetector;
    @Override
    public void init() {


        programStage = progStates.searchingForBalls;


        Globals.target_shooter_speed = Globals.target_shooter_speed_base;//cool
        goalPositionX_offset = 0.0f;
        goalPositionY_offset = 0.0f;

        telemetry.setMsTransmissionInterval(10);
        telemetry.setAutoClear(true);
        ydown = false;
        ypressed = false;


        colorSensor1 = hardwareMap.colorSensor.get("colorSensorRed");
        colorSensor1.setI2cAddress(I2cAddr.create8bit(0x3c));
        colorSensor1.enableLed(false);


        LSA_Sensor1 = (ModernRoboticsAnalogOpticalDistanceSensor) hardwareMap.get("LSA_Sensor1");
        LSA_Sensor1.enableLed(true);
        LSA_Sensor2 = (ModernRoboticsAnalogOpticalDistanceSensor) hardwareMap.get("LSA_Sensor2");
        LSA_Sensor2.enableLed(true);


        wallDetector = (ModernRoboticsI2cRangeSensor) hardwareMap.get("wallDetector");


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

        beacon_servo_1 = hardwareMap.servo.get("servo_3");
        beacon_servo_2 = hardwareMap.servo.get("servo_4");


        triggerSensor = hardwareMap.touchSensor.get("triggerSensor");


        aimServoPositionMaster = 0.0f;
        beacon_servo_1.setPosition(Globals.beacon_servo1_off);
        beacon_servo_2.setPosition(Globals.beacon_servo2_off);


        //Autonomous is supposed to set the x and y coordinates of the goal,
        // but if it wasn't run, they will be -1, so we will default to the blue side
        goalPositionX = HalffieldLength + (float) Math.cos(Math.toRadians(-45.0f)) * 15.5f * 2.54f;
        goalPositionY = HalffieldLength + (float) Math.sin(Math.toRadians(-45.0f)) * 15.5f * 2.54f;


        deployWheelServo1.setPosition(Globals.deployWheelServo1Up);
        deployWheelServo2.setPosition(Globals.deployWheelServo2Up);
    }
    public static boolean blueside = true;
    @Override
    public void init_loop() {
        StageFinished = true;


        wheelLeftLast = dead_motor.getCurrentPosition();
        wheelRightLast = collector_motor.getCurrentPosition();

        aim_servo1.setPosition(Globals.aimLoadPosition);

        if (gamepad1.a) {
            blueside = false;
            FrameGrabber.blueside = false;
        }
        FrameGrabber.usingVision = true;


        triggerServo.setPosition(0.5f);//stop the servo


    }
    @Override
    public void start() {
        runtime.reset();
    }
    float goalPosX_withOffset = 0.0f;
    float goalPosY_withOffset = 0.0f;
    public float moveToTarget() {
        if (AngleWrap(worldAngle_rad - (float) currentLockAngleToVision_rad) > 0) {
            wheelLeft.setPower(0.0f);
            wheelRight.setPower(-0.3f);
        } else {
            wheelRight.setPower(0.0f);
            wheelLeft.setPower(-0.3f);
        }
        //return the amount we still have left for the state machine
        return Math.abs(AngleWrap(worldAngle_rad - (float) currentLockAngleToVision_rad));
    }

    public void stopMotors() {
        wheelLeft.setPower(0.0f);
        wheelRight.setPower(0.0f);
    }


    public long blockStartTime = 0;
    float blockStartingX = 0.0f;
    float blockStartingY = 0.0f;
    float blockStartingAngle = 0.0f;

    public void initializeStateVariables() {
        StageFinished = false;
        //initialization
        blockStartingX = worldXPosition;
        blockStartingY = worldYPosition;
        leftWheelStartingPosition = collector_motor.getCurrentPosition();
        rightWheelStartingPosition = dead_motor.getCurrentPosition();
        blockStartingAngle = worldAngle_rad;
        blockStartTime = SystemClock.uptimeMillis();
    }


    boolean targetBallAcquired = false;
    boolean motorsStopped = false;

    public void activateCollector() {
        collector_motor.setPower(Globals.collector_power);
    }

    boolean reloaded = false;


    public void Aim_and_trigger_servo_movement() {

        aimServoPositionMaster = Range.clip(aimServoPositionMaster, 0.0f, 1.0f);
        aim_servo1.setPosition(aimServoPositionMaster);
    }

    float distance_traveled = 0.0f;
    int wheelLeftLastReading = -1;
    int wheelRightLastReading = -1;
    long lastSpeedReadTime = 0;
    boolean hadToManeuverAroundCenter = false;

    public void triggerStateMachine() {
        if (triggerStage == trigStates.stop) {
            //stop the servo
            triggerServo.setPosition(0.5f);
        }
        if (triggerStage == trigStates.state_reload_begin) {
            triggerServo.setPosition(0.0f);
            triggerStage = trigStates.state_reload_busy;
        }
        if (triggerStage == trigStates.state_reload_busy) {
            if (triggerSensor.isPressed()) {
                triggerServo.setPosition(0.5f);
                reloaded = true;
            } else {
                reloaded = false;
                triggerServo.setPosition(0.0f);
            }
        }
        if (triggerStage == trigStates.state_fire) {
            triggerServo.setPosition(0.0f);
            if (triggerSensor.isPressed()) {
                //only advance to stage firing if the trigger sensor is pressed and ready.
                triggerStage = trigStates.state_firing;
            }
        }
        if (triggerStage == trigStates.state_firing) {
            //wait_for_rev_up for the touch sensor to be unpresseed, and commence reload.
            if (!triggerSensor.isPressed()) {
                triggerStage = trigStates.state_reload_begin;
            }
        }
    }

    public void wait(int miliseconds) {
        if (StageFinished) {//if this is the first update of this block
            initializeStateVariables();
        }
        if (SystemClock.uptimeMillis() - blockStartTime > miliseconds) {
            programStage = programStage.getNext();
            StageFinished = true;
        }
    }

    public void move(float target_distance, boolean direction, float speed, float dither, boolean Break, float decel_dist) {

        if (target_distance < 0) {
            direction = !direction;
        }
        distance_traveled = (float) Math.sqrt(Square(worldXPosition - blockStartingX) + Square(worldYPosition - blockStartingY));

        telemetry.addData("target distance", target_distance / 2.54f);

        float tempWorldAngle = worldAngle_rad;
        tempWorldAngle = AngleWrap(tempWorldAngle);

        if ((decel_dist > 0) &&
                (Math.abs(target_distance) - Math.abs(distance_traveled) < decel_dist)) {
            speed = (1.0f - ((decel_dist - (Math.abs(target_distance) - Math.abs(distance_traveled))) / decel_dist))
                    * 0.7f * speed + (0.3f * speed);
        }

        //if we still need to go forwards more
        if (Math.abs(distance_traveled) < Math.abs(target_distance)) {
            if (!direction) {
                if (AngleWrap(tempWorldAngle - blockStartingAngle) > 0.0f) {
                    wheelLeft.setPower(1.0f * speed);
                    wheelRight.setPower((1.0f - dither) * speed);
                } else {
                    wheelLeft.setPower((1.0f - dither) * speed);
                    wheelRight.setPower(1.0f * speed);
                }
            } else {
                if (AngleWrap(tempWorldAngle - blockStartingAngle) < 0.0f) {
                    wheelLeft.setPower(-1.0f * speed);
                    wheelRight.setPower((-1.0f + dither) * speed);
                } else {
                    wheelLeft.setPower((-1.0f + dither) * speed);
                    wheelRight.setPower(-1.0f * speed);

                }
            }
        } else {//if we have finished moving
            if(programStage != progStates.pathFollowingStage){
                programStage = programStage.getNext();
                StageFinished = true;
            }else{//if this is the mini path-finding state machine
                //reset the path-finding state machine to look for the next point
                goToBallStage++;
                goToBallStepStage = 0;
                goToBallStepStageFinished = true;
            }
            if (Break) {
                wheelRight.setPower(0.0f);
                wheelLeft.setPower(0.0f);
            }

        }
    }



    //path-following state machine vars/////////////////////////////
    //this state-machine variable records the point index we are trying to go to
    int goToBallStage = 0;
    //this state-machine variable records if we are currently turning to the step point
    //or moving. 0 = turning, and 1 = moving
    int goToBallStepStage = 0;
    //allows us to initialize variables inside the mini state machine for one update
    boolean goToBallStepStageFinished = true;
    double ballXAbsolute;
    double ballYAbsolute;

    //the path follower calculates the angle to each step point and needs
    //to remember it between turns and moves
    double AngleToStepPoint = 0.0;

    //records distance to the step point calculated in the turn before the move
    //to step point
    private double distanceToStepPoint;



    public void mainStateMachine() {
        /*
        if (programStage == progStates.turn_to_center_vortex) {
            if (StageFinished) {
                aimServoPositionMaster = Globals.aimLoadPosition;
                triggerStage = trigStates.state_reload_begin;
                wheelsSpinning = true;
                goalPositionY = HalffieldLength + (float) Math.sin(Math.toRadians(-45.0f)) * 15.5f * 2.54f;
                if (blueside) {
                    goalPositionX = HalffieldLength + (float) Math.cos(Math.toRadians(-45.0f)) * 15.5f * 2.54f;
                    worldXPosition = 119;
                    worldYPosition = 36;
                    worldAngle_rad = (float) Math.toRadians(-90.0f);
                } else {
                    goalPositionX = HalffieldLength + (float) Math.cos(Math.toRadians(-135.0f)) * 15.5f * 2.54f;
                    worldXPosition = fieldLength - 119;
                    worldYPosition = 36;
                    worldAngle_rad = (float) Math.toRadians(-90.0f);
                }


                initializeStateVariables();
            }
            if (blueside) {
                MoveToAngleAbsolute((float) angleToCenter_rad - (float) Math.toRadians(180.0f),
                        (float) Math.toRadians(8.0f), 0.17f, 0, true, true, programStage.getNext());
            } else {
                MoveToAngleAbsolute((float) angleToCenter_rad - (float) Math.toRadians(180.0f),
                        (float) Math.toRadians(8.0f), 0.17f, 1, true, true, programStage.getNext());
            }

        }

        if (programStage == progStates.move_before_shooting) {
            if (StageFinished) {
                initializeStateVariables();
                blockStartingAngle = (float) angleToCenter_rad - (float) Math.toRadians(180.0f);
                activateCollector();
            }
            move(30.0f, false, 0.3f, 0.5f, true, 10.0f);
        }

        */

        Aim_and_trigger_servo_movement();



        telemetry.addLine()
                .addData("Ballx", FrameGrabber.bestBallX)
                .addData("Bally", FrameGrabber.bestBallY);
        telemetry.addLine()
                .addData("BallXAbsolute", ballXRelativeToRobotCenter)
                .addData("BallYAbsolute", ballYRelativeToRobot)
                .addData("Size", FrameGrabber.bestBallRadius);
        telemetry.addData("wallDetectorReading", wallDetector.cmUltrasonic());

        newTestBallFinderFunction();





        if (programStage == progStates.searchingForBalls) {
            if (StageFinished) {
                goalPositionY = HalffieldLength + (float) Math.sin(Math.toRadians(-45.0f)) * 15.5f * 2.54f;
                if (blueside) {
                    goalPositionX = HalffieldLength + (float) Math.cos(Math.toRadians(-45.0f)) * 15.5f * 2.54f;
                    worldXPosition = 119;
                    worldYPosition = 36;
                    worldAngle_rad = (float) Math.toRadians(-90.0f);
                } else {
                    goalPositionX = HalffieldLength + (float) Math.cos(Math.toRadians(-135.0f)) * 15.5f * 2.54f;
                    worldXPosition = fieldLength - 119;
                    worldYPosition = 36;
                    worldAngle_rad = (float) Math.toRadians(-90.0f);
                }


                aimServoPositionMaster = Globals.aimLoadPosition;
                initializeStateVariables();
            }
            if (blueside) {
                //turn in place until you find a ball
                wheelLeft.setPower(0.09f);
                wheelRight.setPower(-0.09f);
            } else {
                //turn in place until you find a ball
                wheelLeft.setPower(-0.09f);
                wheelRight.setPower(0.09f);
            }
            if (foundBall) {
                programStage = programStage.getNext();
                StageFinished = true;
            }
        }

        if (programStage == progStates.lockingIteration1) {
            if (StageFinished) {
                initializeStateVariables();
                targetBallAcquired = false;

                stopMotors();//keep spinning for a bit to make sure the ball is in view
                motorsStopped = true;
            }


            if (SystemClock.uptimeMillis() - blockStartTime > 250 && !targetBallAcquired) {
                acquireNewTarget();
                targetBallAcquired = true;
            }


            if (targetBallAcquired) {
                float deltaMove = moveToTarget();
                //once we get withing 1 degree of the ball
                if (deltaMove < Math.toRadians(1.0f)) {
                    stopMotors();
                    programStage = programStage.getNext();
                    StageFinished = true;
                }
            }
        }


        if (programStage == progStates.lockingIteration2) {
            if (StageFinished) {
                initializeStateVariables();
                targetBallAcquired = false;
            }
            if (SystemClock.uptimeMillis() - blockStartTime > 250 && !targetBallAcquired) {
                acquireNewTarget();
                targetBallAcquired = true;
            }

            if (targetBallAcquired) {
                float deltaMove = moveToTarget();
                //once we get withing 1 degree of the ball
                if (deltaMove < Math.toRadians(1.0f)) {
                    stopMotors();
                    programStage = programStage.getNext();
                    StageFinished = true;
                }
            }
        }


        if (programStage == progStates.pathFollowingStage) {
            if (StageFinished) {
                stopMotors();
                initializeStateVariables();

                //our position is our percentage across the field multiplied
                // by the path-finding resolution
                double myPositionX = (worldXPosition/fieldLength)*24;
                double myPositionY = (worldYPosition/fieldLength)*24;


                float phoneAngle = (float) currentLockAngleToVision_rad;//AngleWrap(worldAngle_rad-(float) Math.toRadians(180.0f));
                ballXAbsolute = Math.cos(phoneAngle)*currentLockYPositionToRobot;
                ballYAbsolute = Math.sin(phoneAngle)*currentLockYPositionToRobot;

                ballXAbsolute += worldXPosition;
                ballYAbsolute += worldYPosition;
                //ballXAbsolute += Math.cos(worldAngle_rad)*17*2.54;
                //ballYAbsolute += Math.sin(worldAngle_rad)*17*2.54;

                ballXAbsolute = (ballXAbsolute/fieldLength)*24;
                ballYAbsolute = (ballYAbsolute/fieldLength)*24;
                telemetry.addLine().addData("BallXAbs",ballXAbsolute)
                .addData("BallYAbs",ballYAbsolute);
                telemetry.addLine().addData("MyXField",myPositionX)
                        .addData("MyYField",myPositionY);
                telemetry.addData("Phone Angle",Math.toDegrees(phoneAngle));


                pathFinder.calcPath((int) Math.round(myPositionX),
                        (int) Math.round(myPositionY),
                        (int) Math.round( ballXAbsolute ),
                        (int) Math.round(ballYAbsolute) );


            }

            //Execute the path-following state machine only if the stage is less than the number
            //of steps and the steps have been computed in the pathfinder
            if(goToBallStage < pathFinder.m_filteredSteps.size() && pathFinder.m_filteredSteps != null){
                if(goToBallStepStage == 0){
                    if(goToBallStepStageFinished){
                        goToBallStepStageFinished = false;
                    }



                    Point thisStep = pathFinder.m_filteredSteps.get(goToBallStage);
                    //calculate the x position of the pathfinder's step by dividing it by the
                    //pathfinder's field size and multiplying it by our field size

                    double thisStepX = (thisStep.x/24)*fieldLength;
                    double thisStepY = (1-(thisStep.y/24))*fieldLength;



                    //of course when calculating angles, we need the step position relative to us
                    double deltaXPoint = thisStepX-worldXPosition;
                    double deltaYPoint = thisStepY-worldYPosition;
                    AngleToStepPoint = Math.atan2(deltaYPoint,deltaXPoint);

                    distanceToStepPoint = Math.sqrt(Square((float) deltaXPoint)+Square((float) deltaYPoint));



                    //move to the angle calculated but don't advance the state since we are in a mini state machine
                    //the turn function will increment our goToBallStepStage variable to go to the next stage
                    MoveToAngleAbsolute((float) AngleToStepPoint,(float) Math.toRadians(50.0f),0.15f,2,false,true,programStage);
                }
                if(goToBallStepStage == 1){
                    if(goToBallStepStageFinished){
                        initializeStateVariables();
                        goToBallStepStageFinished = false;
                    }
                    move((float) distanceToStepPoint,true,0.5f,0.3f,true,30.0f);
                }
            }
            //if we have completed all the stages in path following, stop the motors
            if(goToBallStage == pathFinder.m_filteredSteps.size()){
                stopMotors();
            }
        }

        if (programStage == progStates.driveBackABit) {
            if (StageFinished) {
                initializeStateVariables();
            }
            move(10.0f, false, 0.3f, 0.5f, true, 10.0f);

        }
        if (programStage == progStates.pointToGoal) {
            if (StageFinished) {
                initializeStateVariables();
            }
            MoveToAngleAbsolute((float) angleToCenter_rad - (float) Math.toRadians(180.0f),
                    (float) Math.toRadians(8.0f), 0.17f, 0, true, true, programStage.getNext());
        }
        float distance_to_proximity;
        if (programStage == progStates.driveToProximity) {
            if (StageFinished) {
                initializeStateVariables();
                blockStartingAngle = (float) angleToCenter_rad - (float) Math.toRadians(180.0f);
            }
            distance_to_proximity = (float) Math.sqrt(Square(worldXPosition - goalPositionX) +
                    Square(worldYPosition - goalPositionY));

            telemetry.addData("distance to goal", distance_to_proximity);
            move(666666, false, 0.3f, 0.5f, true, 10.0f);
            if (distance_to_proximity < 150) {
                stopMotors();
                programStage = programStage.getNext();
                StageFinished = true;
            }

        }


        //////////RELOAD
        if (programStage == progStates.reload ){//|| programStage == progStates.reloadn1 || programStage == progStates.reloadn2) {
            if (StageFinished) {//if this is the first update of this block
                initializeStateVariables();
                triggerStage = trigStates.state_reload_begin;
                aimServoPositionMaster = Globals.aimLoadPosition;
                programStage = programStage.getNext();
                StageFinished = true;
            }
        }
        if (programStage == progStates.waitforreload){// || programStage == progStates.waitforreloadn1 || programStage == progStates.waitforreloadn2) {
            wait(1500);
        }


        //////////YOTO YAIM
        if (programStage == progStates.aim ){//|| programStage == progStates.aimn1 || programStage == progStates.aimn2) {
            if (StageFinished) {//if this is the first update of this block
                initializeStateVariables();

                //calculate the Aim based on the curve fit we did
                float calculatedAim = Globals.shooterConstant - (Globals.shooterSlope * DistanceToTargetCentimeters);
                telemetry.addData("calculatedAim=", calculatedAim);
                aimServoPositionMaster = calculatedAim;

                programStage = programStage.getNext();
                StageFinished = true;
            }
        }

        if (programStage == progStates.waitforaim ){//||
                //programStage == progStates.waitforaimn1 ||
                //programStage == progStates.waitforaimn2) {
            wait(500);
        }


        //////////FIRE!!!!!!!!!!!!!!!!!!!!!!!!!!!!//////////////////////////////////////////////////////////////////////////////
        if (programStage == progStates.fire) {
            if (StageFinished) {//if this is the first update of this block
                initializeStateVariables();
                triggerStage = trigStates.state_fire;
                StageFinished = true;
                programStage = programStage.waitforfire;
            }
        }
        //////////FIRE!!!!!!!!!!!!!!!!!!!!!!!!!!!!//////////////////////////////////////////////////////////////////////////////
        /*if (programStage == progStates.firen1 || programStage == progStates.firen2) {
            if (StageFinished) {//if this is the first update of this block
                initializeStateVariables();
                triggerStage = trigStates.state_fire;
                StageFinished = true;
                programStage = programStage.getNext();
            }
        }
        */

        if (programStage == progStates.waitforfire){// || programStage == progStates.waitforfiren1) {
            wait(700);
        }
    }

    public void debugDisplayText() {

        //telemetry.addData("programStage", programStage);

        telemetry.addLine()
                .addData("X: ", worldXPosition)
                .addData(" Y: ", worldYPosition)
                .addData(" ∠: ", Math.toDegrees(worldAngle_rad));




        //telemetry.addLine()
                //.addData("GoalX", goalPositionX)
                //.addData("GoalY", goalPositionY);


        //telemetry.addData("Shooter Speed:", Globals.target_shooter_speed);

        /*
        telemetry.addLine()
                .addData("Manual Adjs:", "")
                .addData("X: ", goalPositionX_offset)
                .addData("Y: ", goalPositionY_offset);
        telemetry.addData("AimServo: ", aimServoPositionMaster);
        */

        telemetry.update();
    }


    @Override
    public void loop() {
        triggerStateMachine();
        mainStateMachine();
        debugDisplayText();
        ShooterSpeedCalculations();
        PositioningCalculations();
        AutoAimCalculations();
    }

    float goalPositionX_offset = 0.0f;
    float goalPositionY_offset = 0.0f;
    @Override
    public void stop() {
    }

    public float Square(float number) {
        return number * number;
    }


    private void ShooterSpeedCalculations() {

        if (wheelsSpinning) {
            if (!isAccelerating && !DoneAccelerating) {//if this is the first update that the bbutton was down..
                //initialize our clock
                lastAccelUpdate = SystemClock.uptimeMillis();
                //Don't do speed control when accelerating
                shooter_motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shooter_motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                isAccelerating = true;
                isDecelerating = false;

                target_shooter_power = 0.20f;//accelerate to this and once done,

            }

        } else {

            if (isAccelerating) {//if this is the first update of deceleration
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

        if (isAccelerating) {
            //if you have not reached maximum power, accelerate the wheels slowly, for 4 seconds based on TIME
            if (shooter_current_power < target_shooter_power) {
                shooter_current_power += ((SystemClock.uptimeMillis() - lastAccelUpdate) / 3000.0f);
                //remember the time of this update for the next calculation
                lastAccelUpdate = SystemClock.uptimeMillis();
                //since we just made a power change, update power, and clip it for safety
                shooter_current_power = Range.clip(shooter_current_power, -1.0f, 1.0f);
                shooter_motor1.setPower(shooter_current_power);
                shooter_motor2.setPower(shooter_current_power);
            } else {//we are done accelerating
                DoneAccelerating = true;

                //change over to speed control, because you are at target power
                shooter_motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shooter_motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //apply target speed
                shooter_motor1.setPower(Globals.target_shooter_speed);
                shooter_motor2.setPower(Globals.target_shooter_speed);

            }
        }
        if (isDecelerating) {

            shooter_current_power -= (SystemClock.uptimeMillis() - lastDecelUpdate) / 3000.0f;//decelerate at a rate that would take 3 seconds to decelerate from 1.0 to 0.0
            lastDecelUpdate = SystemClock.uptimeMillis();

            if (shooter_current_power <= 0.0f) {
                isDecelerating = false;//you are done decelerating
                shooter_motor1.setPower(0);
                shooter_motor2.setPower(0);
                shooter_current_power = 0.0f;
            } else {

                //apply target power
                shooter_motor1.setPower(shooter_current_power);
                shooter_motor2.setPower(shooter_current_power);
            }
        }
    }


    public void PositioningCalculations() {
        float balanceScalingFactor =
                1.0f;

        //float wheelRightCurrent = WheelRightOT_position_tracked;
        //float wheelLeftCurrent = WheelLeftOT_position_tracked;

        float wheelLeftCurrent = dead_motor.getCurrentPosition() / balanceScalingFactor;
        float wheelRightCurrent = collector_motor.getCurrentPosition() * balanceScalingFactor;
        //telemetry.addData("wheel Left Pos:",dead_motor.getCurrentPosition());
        //telemetry.addData("wheel Right Pos:",collector_motor.getCurrentPosition());

        float wheelLeftDelta = (wheelLeftCurrent - wheelLeftLast) / moveScalingFactor;
        float wheelRightDelta = (wheelRightCurrent - wheelRightLast) / moveScalingFactor;


        //if(!(Math.abs(wheelLeftDelta) < 0.25f && Math.abs(wheelRightDelta) < 0.25f)){
        float robotWidth = 38.62f;//39.2247 OLD in centimeters// this is effectively the turning scale factor

        float Angleincrement = (wheelRightDelta - wheelLeftDelta) / robotWidth;
        worldAngle_rad += Angleincrement;
        //telemetry.addData("WorldAngle:", Math.toDegrees(worldAngle_rad));
        float Distance = (wheelRightDelta + wheelLeftDelta) / 2.0f;
        worldXPosition += Math.cos(worldAngle_rad) * Distance;
        worldYPosition += Math.sin(worldAngle_rad) * Distance;


        wheelLeftLast = wheelLeftCurrent;
        wheelRightLast = wheelRightCurrent;


        worldAngle_rad = AngleWrap(worldAngle_rad);


        ReadEncodersOT();

        goalPosX_withOffset = goalPositionX + goalPositionX_offset;
        goalPosY_withOffset = goalPositionY + goalPositionY_offset;

        //calculate the distance to the target with the distance formula
        DistanceToTargetCentimeters = (float) Math.sqrt(Square(goalPosX_withOffset - worldXPosition) + Square(goalPosY_withOffset - worldYPosition));

    }


    public void ReadEncodersOT() {
        ////////////////////////////PositionTrackers///////////////////////////////
        if (SystemClock.uptimeMillis() - All_telemetry_trackers_lastUpdate > All_telemetry_trackers_pollingRate) {
            All_telemetry_trackers_lastUpdate = SystemClock.uptimeMillis();
            // Get the current reading
            //WheelRightOT_curr_sensor_reading = Math.abs((float) WheelRightOT.getVoltage()/ (float) WheelRightOT.getMaxVoltage());
            //WheelLeftOT_curr_sensor_reading = Math.abs((float) WheelLeftOT.getVoltage()/ (float) WheelLeftOT.getMaxVoltage());

            float WheelRightOT_delta_temp = WheelRightOT_curr_sensor_reading - WheelRightOT_last_sensor_reading;
            float WheelLeftOT_delta_temp = WheelLeftOT_curr_sensor_reading - WheelLeftOT_last_sensor_reading;

            float WheelRightOT_delta_final = WheelRightOT_delta_temp;
            float WheelLeftOT_delta_final = WheelLeftOT_delta_temp;


            if (Math.abs(WheelRightOT_delta_temp) <= All_telemetry_trackers_wrapAroundThreshold && WheelRightOT_WrapingAround) {
                WheelRightOT_WrapingAround = false;
                WheelRightOT_delta_temp = WheelRightOT_curr_sensor_reading - WheelRightOT_lastvalidreading;
                if (WheelRightOT_delta_temp > 0.5f) {
                    WheelRightOT_delta_final -= 1.0f;
                }
                if (WheelRightOT_delta_temp < -0.5f) {
                    WheelRightOT_delta_final += 1.0f;
                }
            }
            if (Math.abs(WheelLeftOT_delta_temp) <= All_telemetry_trackers_wrapAroundThreshold && WheelLeftOT_WrapingAround) {
                WheelLeftOT_WrapingAround = false;
                WheelLeftOT_delta_temp = WheelLeftOT_curr_sensor_reading - WheelLeftOT_lastvalidreading;
                if (WheelLeftOT_delta_temp > 0.5f) {
                    WheelLeftOT_delta_final -= 1.0f;
                }
                if (WheelLeftOT_delta_temp < -0.5f) {
                    WheelLeftOT_delta_final += 1.0f;
                }
            }

            if (Math.abs(WheelRightOT_delta_temp) >= All_telemetry_trackers_wrapAroundThreshold && !WheelRightOT_WrapingAround) {
                WheelRightOT_WrapingAround = true;
                WheelRightOT_lastvalidreading = WheelRightOT_last_sensor_reading;
            }
            if (Math.abs(WheelLeftOT_delta_temp) >= All_telemetry_trackers_wrapAroundThreshold && !WheelLeftOT_WrapingAround) {
                WheelLeftOT_WrapingAround = true;
                WheelLeftOT_lastvalidreading = WheelLeftOT_last_sensor_reading;
            }

            if (!WheelRightOT_WrapingAround) {
                WheelRightOT_position_tracked += WheelRightOT_delta_final;
            }
            if (!WheelLeftOT_WrapingAround) {
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

    private void AutoAimCalculations() {
        int quadrant = 0;
        if (worldYPosition <= goalPosY_withOffset && worldXPosition <= goalPosX_withOffset) {
            quadrant = 1;
        }
        if (worldYPosition >= goalPosY_withOffset && worldXPosition <= goalPosX_withOffset) {
            quadrant = 2;
        }
        if (worldYPosition >= goalPosY_withOffset && worldXPosition >= goalPosX_withOffset) {
            quadrant = 3;
        }
        if (worldYPosition <= goalPosY_withOffset && worldXPosition >= goalPosX_withOffset) {
            quadrant = 4;
        }


        if (quadrant == 1) {
            angleToCenter_rad = Math.atan((goalPosY_withOffset - worldYPosition) / (goalPosX_withOffset - worldXPosition));
        }
        if (quadrant == 2) {
            angleToCenter_rad = -Math.atan((worldYPosition - goalPosY_withOffset) / (goalPosX_withOffset - worldXPosition));
        }
        if (quadrant == 3) {
            angleToCenter_rad = Math.PI + Math.atan((worldYPosition - goalPosY_withOffset) / (worldXPosition - goalPosX_withOffset));
        }
        if (quadrant == 4) {
            angleToCenter_rad = Math.PI - Math.atan((goalPosY_withOffset - worldYPosition) / (worldXPosition - goalPosX_withOffset));
        }
        //wrap around the angleToCenter_rad
        angleToCenter_rad = AngleWrap((float) angleToCenter_rad);
    }
    public static float AngleWrap(float angle) {
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }


    private void MoveToAngleAbsolute(float angle, float maxangle, float maxspeed, int wheel, boolean maintain, boolean Break, progStates advanceState) {

        float wheelLeftPower = 0;
        float wheelRightPower = 0;


        float howMuchToTurn = AngleWrap(angle - worldAngle_rad);


        if (wheel == 0) {
            wheelLeftPower = -(howMuchToTurn / maxangle) * maxspeed;

            if (wheelLeftPower > 0.0f) {
                wheelLeftPower += 0.05f;//add a minimum power
            } else {
                wheelLeftPower -= 0.05f;//add a minimum power
            }


            if (maintain) {
                if (dead_motor.getCurrentPosition() < rightWheelStartingPosition) {
                    wheelRightPower = -0.15f;
                } else {
                    wheelRightPower = 0.15f;
                }
            } else {
                wheelRightPower = 0.0f;
            }

        }
        if ((wheel == 1) || (wheel == 2)) {
            wheelRightPower = (howMuchToTurn / maxangle) * maxspeed;
            if (wheelRightPower > 0.0f) {
                wheelRightPower += 0.05f;//add a minimum power
            } else {
                wheelRightPower -= 0.05f;//add a minimum power
            }


            if (maintain) {
                if (collector_motor.getCurrentPosition() < leftWheelStartingPosition) {
                    wheelLeftPower = -0.15f;
                } else {
                    wheelLeftPower = 0.15f;
                }
            } else {
                wheelLeftPower = 0.0f;
            }

            if (wheel == 2) {
                wheelLeftPower = -1.0f * wheelRightPower;
            }

        }


        float maxpower = Math.abs(maxspeed);


        wheelLeft.setPower(Range.clip(wheelLeftPower, -maxpower, maxpower));
        wheelRight.setPower(Range.clip(wheelRightPower, -maxpower, maxpower));


        //telemetry.addData("How much to turn:", Math.toDegrees(howMuchToTurn));
        //if we are within one degree of our target angle, finish and move to the next thing and brake
        if (Math.abs(howMuchToTurn) <= Math.toRadians(1.0f)) {
            if (Break) {
                wheelLeft.setPower(0.0f);
                wheelRight.setPower(0.0f);
            }


            programStage = advanceState;
            StageFinished = true;

            //if we are in the path following stage, incrememnt that state-machine variable
            if(programStage == progStates.pathFollowingStage){
                goToBallStepStage++;
                goToBallStepStageFinished = true;
                initializeStateVariables();
            }
        }
    }
}
