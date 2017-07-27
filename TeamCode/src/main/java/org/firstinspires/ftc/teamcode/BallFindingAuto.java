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

import android.os.AsyncTask;
import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Point;

import java.text.DecimalFormat;

import RobotUtilities.Tracker;
import ftc.pathfinder.pathFinderNative;
import ftc.vision.FrameGrabber;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "BallFindingAuto", group = "yee")
public class BallFindingAuto extends OpMode {


    public double fieldLength = 358.775;//the length of the feild in cm
    public double HalffieldLength = fieldLength / 2.0;//the length of half field

    public enum progStates {


        backup,
        searchingForBalls,
        lockingIteration1,
        lockingIteration2,
        pathFollowingStage;


        public progStates getNext() {
            return this.ordinal() < progStates.values().length - 1
                    ? progStates.values()[this.ordinal() + 1]
                    : null;
        }
    }

    DecimalFormat df = new DecimalFormat("#.00");
    /*///////////////////////////////VARS////////////////////////////*/
    BNO055IMU imu;

    public final float turnScale = 0.6f;
    DcMotor motorTL;
    DcMotor motorTR;
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motor_shooter;
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
    /*/////////////////////////////////////////////////////////////////*/

    @Override
    public void init() {
        motorTL = hardwareMap.dcMotor.get("motorTL");
        motorTR = hardwareMap.dcMotor.get("motorTR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motor_shooter = hardwareMap.dcMotor.get("shooter");
        motor_shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        door_servo = hardwareMap.servo.get("door");


        telemetry.setMsTransmissionInterval(10);
    }
  @Override
  public void init_loop() {

  }


    @Override
    public void start() {
        programStartTime = SystemClock.uptimeMillis();


        //////////reset encoders///////////////
        tracker_l.reset();
        tracker_r.reset();
        tracker_a.reset();
        wheelLeftLast = tracker_l.getPos();
        wheelRightLast= -tracker_r.getPos();
        ///////////////////////////////////////


        worldXPosition = 119;
        worldYPosition = 23;
        worldAngle_rad = Math.toRadians(-90.0f);
        targetAngle = Math.toRadians(0);

        DbgLog.msg("ProgramStart");
    }







    double GyroAngle = 0;
    Orientation angles;
    long programStartTime = 0;
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

    public void lock_currentBall() {
        finishedLockingTarget = true;
    }

    boolean foundBall = false;
    public static double unlockedBall_absolute_angle;
    public static double lockedBall_absolute_angle;
    public static double lockedBall_distanceAway;
    //the distance from the robot to the unlocked ball
    public static double unlockedBall_distanceAway;
    static double ballYRelativeToRobot;
    static double ballXRelativeToRobotCenter;
    double ballXRelativeToCenterPercent = 0.0;
    final double screenXSize = 800;
    final double screenYSize = 480;
    public boolean finishedLockingTarget = false;
    public void vision_findBalls() {

        telemetry.addData("CurrentlyLocking",finishedLockingTarget);

        if(FrameGrabber.bestBallX != 0){
            foundBall = true;
        }else{
            foundBall = false;
        }

        if (foundBall && !FrameGrabber.uptoDate) {
            int ballX = FrameGrabber.bestBallX;
            int ballY = FrameGrabber.bestBallY;
            ballY = 480-ballY;

            ballYRelativeToRobot = 182417400 + ((89.93381 - 182417400) / (1 + Math.pow((ballY / 4440.877), 5.265578)));



            double cameraViewSizeAtY = (1.1 * ballYRelativeToRobot) + 23.85185;
            double cameraCenterViewSizeAtY = cameraViewSizeAtY / 2.0;

            ballXRelativeToCenterPercent = (ballX - (screenXSize / 2.0f)) / (screenXSize / 2.0f);
            ballXRelativeToRobotCenter = ballXRelativeToCenterPercent * cameraCenterViewSizeAtY;

            float phonePositionRelativeToRobotOriginX = -4;
            ballXRelativeToRobotCenter += phonePositionRelativeToRobotOriginX;


            //calculate angle
            unlockedBall_absolute_angle = worldAngle_rad + (Math.atan2(ballYRelativeToRobot, ballXRelativeToRobotCenter)-Math.toRadians(90));
            unlockedBall_distanceAway = Math.sqrt(Square(ballYRelativeToRobot)+Square(ballXRelativeToRobotCenter));


            FrameGrabber.uptoDate = true;
        }
        telemetry.addData("Ball Raw:",FrameGrabber.bestBallY);
        telemetry.addLine()
                .addData("BestBall: ang: ", Math.toDegrees(unlockedBall_absolute_angle))
                .addData("dist: ",unlockedBall_distanceAway);



        //this is a variable set by the user that once set to true, will remember the current unlocked
        //ball's coordinates and save them. The prefixes change from unlockedBall to lockedBall.
        if (finishedLockingTarget ){//|| programStage == progStates.pathFollowingStage) {
            if (foundBall) {
                lockedBall_absolute_angle = unlockedBall_absolute_angle;
                lockedBall_distanceAway = unlockedBall_distanceAway;
                finishedLockingTarget = false;
                calcBallPosAbs();
            }
        }
    }

    double moveToTargetSpeed = 0.06;
    public double moveToTarget() {
        //simply move left or right depending on where the ball is
        //this will dither around the center point of the ball
        //probs want a proportional algorithm later
        if (AngleWrap(worldAngle_rad - lockedBall_absolute_angle) < 0) {
            movement_turn = moveToTargetSpeed;
        } else {
            movement_turn = -moveToTargetSpeed;
        }
        //return the amount we still have left for the state machine
        return Math.abs(AngleWrap(worldAngle_rad - lockedBall_absolute_angle));
    }







    long blockStartTime = 0;
    double blockStartingX = 0.0f;
    double blockStartingY = 0.0f;
    double blockStartingAngle = 0.0f;
    public void initializeStateVariables() {
        StageFinished = false;
        //initialization
        blockStartingX = worldXPosition;
        blockStartingY = worldYPosition;
        blockStartingAngle = worldAngle_rad;
        blockStartTime = SystemClock.uptimeMillis();
    }


    progStates programStage = progStates.backup;
    boolean StageFinished = true;
    boolean targetBallAcquired = false;






    //path-following state machine vars/////////////////////////////
    //this state-machine variable records the point index we are trying to go to
    int goToBallStage = 0;

    double ballXAbsolute;
    double ballYAbsolute;

    //the path follower calculates the angle to each step point and needs
    //to remember it between turns and moves
    double AngleToStepPoint = 0.0;

    //records distance to the step point calculated in the turn before the move
    //to step point
    private double distanceToStepPoint;





    int ballUpdates = 0;
    public void calcBallPosAbs(){
        //the ball may not be perfectly centered on the screen, so calculate it's
        //angle relative to the phone and it's actual distance away formed by this
        //triangle
        double ballXAbsoluteTemp = Math.cos(lockedBall_absolute_angle)* lockedBall_distanceAway;
        double ballYAbsoluteTemp = Math.sin(lockedBall_absolute_angle)* lockedBall_distanceAway;
        ballXAbsoluteTemp += worldXPosition;
        ballYAbsoluteTemp += worldYPosition;

        if(ballXAbsoluteTemp > fieldLength-5){
            ballXAbsoluteTemp = fieldLength-5;
        }
        if(ballXAbsoluteTemp < 5){
            ballXAbsoluteTemp = 5;
        }
        if(ballYAbsoluteTemp > fieldLength-5){
            ballYAbsoluteTemp = fieldLength -5;
        }
        if(ballYAbsoluteTemp < 5){
            ballYAbsoluteTemp = 5;
        }
        //If we are pathfollowing an already calculated path and the position of the ball is really far away
        //from the last calculation, don't update it, because it is likely a different ball
        boolean okayToUpdateBallPos = true;
        if(programStage == progStates.pathFollowingStage){

            double thisUpdateDistDelta = Math.sqrt(
                    Square(ballXAbsoluteTemp-ballXAbsolute)+
                    Square(ballYAbsoluteTemp-ballYAbsolute));

            if(pathFinderNative.m_filteredSteps != null){
                if(thisUpdateDistDelta > 100 || distanceToBall < 40){
                    okayToUpdateBallPos = false;
                }
            }

        }
        if(okayToUpdateBallPos){

            ballUpdates++;
            ballXAbsolute = ballXAbsoluteTemp;
            ballYAbsolute = ballYAbsoluteTemp;
            FrameGrabber.bestBallX_abs = ballXAbsolute;
            FrameGrabber.bestBallY_abs = ballYAbsolute;
        }


    }




    pathFinderNative m_pathFinder = new pathFinderNative();
    public void MainStateMachine() {

        if(programStage == progStates.backup){
            if(StageFinished){
                initializeStateVariables();
                movement_y = -0.2;
            }
            if(worldYPosition > 50){
                movement_y = 0;
                StageFinished = true;
                programStage = programStage.getNext();
            }
        }


        double searchingForBallsSpeed = -0.2;
        //this state turns until it sees a ball and then moves to locking iterations
        if (programStage == progStates.searchingForBalls) {
            //initialize our position when this state starts
            if (StageFinished) {
                initializeStateVariables();
                //start turning and searching for balls
                movement_turn = searchingForBallsSpeed;
            }
            if (foundBall) {
                //move on but don't stop the motors
                //this will do a slight over-turn however,
                //that will make sure the ball is directly in view
                programStage = programStage.getNext();
                StageFinished = true;
            }
        }

        if (programStage == progStates.lockingIteration1) {
            if (StageFinished) {
                initializeStateVariables();
                //we haven't acquired our target until we turn for a little longer and confirm that
                //the ball is still visible.
                targetBallAcquired = false;
            }


            //after 250 milliseconds of overtime, try to re-find the target and confirm that it
            //is still there. It should be more centered in the camera's view now
            if (SystemClock.uptimeMillis() - blockStartTime > 250 && !targetBallAcquired) {
                lock_currentBall();
                targetBallAcquired = true;
            }


            //now that we have aquired our target ball, turn towards it
            if (targetBallAcquired && foundBall) {
                double deltaMove = moveToTarget();
                //once we get withing 1 degree of the ball
                if (deltaMove < Math.toRadians(0.5)) {
                    movement_turn=0;
                    programStage = programStage.getNext();
                    StageFinished = true;
                }
            }
        }


        if (programStage == progStates.lockingIteration2) {
            if (StageFinished) {
                initializeStateVariables();
                //we haven't acquired our target until we turn for a little longer and confirm that
                //the ball is still visible.
                targetBallAcquired = false;
            }


            if (!targetBallAcquired) {
                lock_currentBall();
                targetBallAcquired = true;
            }


            //now that we have aquired our target ball, turn towards it
            if (targetBallAcquired&&foundBall) {
                double deltaMove = moveToTarget();
                //once we get withing 1 degree of the ball
                if (deltaMove < Math.toRadians(0.5)) {
                    movement_turn=0;
                    programStage = programStage.getNext();
                    StageFinished = true;
                }
            }
        }

        ///////////////////////////////////////////PATH FOLLOWING STAGE/////////////////////////////
        telemetry.addData("ballupdates:",ballUpdates);
        if (programStage == progStates.pathFollowingStage) {
            //only start when we have locked and computed the ball
            if (StageFinished && !finishedLockingTarget) {
                initializeStateVariables();
                ballUpdates = 0;


                //calculate the path in the background. It is a new path, so the last param is 1
                m_pathFinder.execute(worldXPosition, worldYPosition, ballXAbsolute, ballYAbsolute, fieldLength,1.0);
                //also initialize our step stage to zero since this is the first computation of the path
                goToBallStage=0;
            }

            //update the ball's position every update
            lock_currentBall();

            double distance_from_center = Math.sqrt(Square(worldXPosition-HalffieldLength)+Square(worldYPosition-HalffieldLength));



            //we don't want any exceptions of trying to follow a path that doesn't exist
            if(pathFinderNative.m_filteredSteps != null){
                //update the pathfinder if we have a new ball position once the current one
                //has finished computing
                if(foundBall && m_pathFinder.getStatus()== AsyncTask.Status.FINISHED&&
                        distanceToBall > 30 && distance_from_center > 50){

                    m_pathFinder = new pathFinderNative();

                    //calculate the updated path in the background. It is not a new path, so the last param is 0
                    m_pathFinder.execute(worldXPosition, worldYPosition, ballXAbsolute, ballYAbsolute, fieldLength,0.0);
                }
                //if the pathFinder has finished computing, reset our step count, because we have
                //only just began to follow this new path. Also, we are now up to date, so set that
                //variable to true so that we don't do this again until uptodate is set to false
                if(!pathFinderNative.upToDate){
                    goToBallStage = 0;
                    pathFinderNative.upToDate = true;
                }

                //Execute the path-following state machine only if the stage is less than the number
                //of steps and the steps have been computed in the pathfinder
                if(goToBallStage < pathFinderNative.m_filteredSteps.size() ){

                    //get the current step position and store it in some variables
                    Point thisStep = pathFinderNative.m_filteredSteps.get(goToBallStage);
                    double thisStepX = thisStep.x;
                    double thisStepY = thisStep.y;


                    //the last step matters a lot because it is the final approach.
                    //therefore, we want the full resolution of the initial scan here
                    //because the position of the ball in the pathfinder is at a low resolution
                    //therefore, set it to the actual, un-rounded position initially calculated
                    if(goToBallStage == pathFinderNative.m_filteredSteps.size()-1){
                        thisStepX = ballXAbsolute;
                        thisStepY = ballYAbsolute;
                    }


                    FrameGrabber.step_x = thisStepX;
                    FrameGrabber.step_y = thisStepY;

                    //of course when calculating angles, we need the step position relative to us
                    double deltaXPoint = thisStep.x-worldXPosition;
                    double deltaYPoint = thisStep.y-worldYPosition;
                    AngleToStepPoint = Math.atan2(deltaYPoint,deltaXPoint);

                    distanceToStepPoint = Math.sqrt(Square(deltaXPoint)+Square(deltaYPoint));



                    //we can move faster if we are farther than 150cm away from the ball.
                    if(distanceToBall > 150){
                        goToPosition(thisStepX,thisStepY,0.3,0.2);
                    }else{
                        goToPosition(thisStepX,thisStepY,0.3,0.2);
                    }

                }
                //if we have completed all the stages in path following or we have come
                //within 20 centimeters of the exact ball position, stop the motors.
                if(goToBallStage == pathFinderNative.m_filteredSteps.size() || distanceToBall<20){
                    movement_x = 0;
                    movement_y = 0;
                    movement_turn = 0;
                }
            }
        }
    }
    private double Square(double x) {
        return x*x;
    }




    private void goToPosition(double x, double y, double speed,double turnToPointSpeed){

        double angleToPoint = Math.atan2(y-worldYPosition,x-worldXPosition);

        double relativeAngle = AngleWrap(angleToPoint-worldAngle_rad);


        movement_y = (Math.cos(relativeAngle)) * speed;
        movement_x = -(Math.sin(relativeAngle)) * speed;

        double absAngleToBall = AngleWrap(Math.atan2(ballYAbsolute-worldYPosition,ballXAbsolute-worldXPosition));
        double angleToBall = AngleWrap(absAngleToBall-worldAngle_rad);


        double absAngleTo90 = (Math.round(absAngleToBall/Math.toRadians(90)) * Math.toRadians(90));
        double angleTo90 = AngleWrap(absAngleTo90-worldAngle_rad);



        //if(distanceToBall > 160){
            //movement_turn = ((relativeAngle)/Math.toRadians(45))*turnToPointSpeed;
        //}else{
        //}
        boolean isWallBall = false;
        if(ballXAbsolute > fieldLength-30 || ballXAbsolute < 30 || ballYAbsolute > fieldLength-30 || ballYAbsolute < 30){
            isWallBall = true;
        }
        if(pathFinderNative.m_filteredSteps != null){
            if(goToBallStage == pathFinderNative.m_filteredSteps.size()-1 && isWallBall){
                angleToBall = angleTo90;
            }
        }
        movement_turn = ((angleToBall)/Math.toRadians(45))*turnToPointSpeed;




        if(Math.sqrt(Square(x-worldXPosition)+Square(y-worldYPosition)) < 5){
            movement_x = 0;
            movement_y = 0;
            movement_turn =0;
            goToBallStage++;
        }
    }




    double distanceToBall;
    @Override
    public void loop() {
        telemetry.addData("bestBallRadius: ", FrameGrabber.bestBallRadius);
        //finishedLockingTarget = true;
        vision_findBalls();
        distanceToBall = Math.sqrt(
                Square(worldXPosition-ballXAbsolute)+
                        Square(worldYPosition-ballYAbsolute));


        MainStateMachine();
        ApplyMovement();
        ReadEncodersOT();
        PositioningCalculations();

        shooter();
        trigger();

    }

    public final double moveScalingFactor = 18.2667;//when you divide the encoder values by this it returns cm
    public double worldXPosition=0;
    public double worldYPosition=0;
    public double worldAngle_rad = 0;//start the world angle at 0 degrees

    public double wheelLeftLast = 0;
    public double wheelRightLast = 0;
    public double wheelAuxLast = 0;

    public double fastestSpeed = 0;

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

    long All_telemetry_trackers_lastUpdate =0;
    public static final int All_telemetry_trackers_pollingRate = 0;
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




    public double targetAngle = Math.toRadians(90.0f);

    public void ApplyMovement(){
      motorTL.setPower(-movement_y+movement_turn-movement_x);
      motorBL.setPower(-movement_y+movement_turn+movement_x);
      motorBR.setPower(-movement_y-movement_turn-movement_x);
      motorTR.setPower(-movement_y-movement_turn+movement_x);
    }

    boolean shooter_master = false;
    boolean shooter_reving = false;
    long shooter_rev_start_time = 0;
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
    double shooter_current_speed;
    public static final double shooter_max_speed = 0.62;
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
    public enum trigStates {
        secureDown,
        stopped,
        engaged;
        public trigStates getNext() {
            return this.ordinal() < trigStates.values().length - 1
                    ? trigStates.values()[this.ordinal() + 1]
                    : null;
        }
    }
    trigStates triggerStage = trigStates.secureDown;
    double door_servo_current_pos = 0;
    boolean fire = false;
    long fireStartTime = 0;
    public void fire(){
        //if during the last update, we were not firing, initialize
        if(!fire){
            fireStartTime = SystemClock.uptimeMillis();
        }
        fire = true;
        triggerStage = trigStates.engaged;
    }
    public void stopFiring(){
        fire = false;
    }
    long secureDown_start_time = 0;
    public void trigger(){
        telemetry.addData("triggerStage: ", triggerStage);
        if(gamepad1.x){
            fire();
        }else{
            stopFiring();
        }
        if(triggerStage == trigStates.engaged){
            door_servo_current_pos = 1;
            if(SystemClock.uptimeMillis()-fireStartTime >= 1000 && !fire){
                triggerStage = trigStates.secureDown;
                secureDown_start_time = SystemClock.uptimeMillis();
            }
        }


        if(triggerStage == trigStates.secureDown){
            door_servo_current_pos = 0;
            if(SystemClock.uptimeMillis()-secureDown_start_time > 800){
                triggerStage = trigStates.stopped;
            }
        }
        if(triggerStage == trigStates.stopped){
            door_servo_current_pos = 0.5;
        }

        door_servo.setPosition(door_servo_current_pos);
    }

}
