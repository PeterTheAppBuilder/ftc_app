package org.firstinspires.ftc.teamcode;

/**
 * Created by GlutenFree on 1/24/2017.
 */

public class Globals {
    /*

    To DO:
    -Make collector backwards control
    -change manual adjust to be holding left bumper and abxy as dpad
     */


    public static final float LSA_sensor_x_relative = 37.0f/2.0f;
    public static final float LSA_sensor_y_relative = 21.5f;


    public static final float LSA_distance_between_sensors = 37.0f;
    public static final float red_line_intensity = 0.6f;
    public static final float blue_line_intensity = 0.21f;


    public static final float leftCurveSpeed = -0.32f;
    public static final float rightCurveSpeed = -0.4f;
    public static float light_read_speed = 0.13f;
    public static final float curveToCenterSpeedL = 0.28f*2.0f;
    public static final float curveToCenterSpeedR = 0.20f*2.0f;

    public static final int servo_press_time = 500;
    public static final float ready_press_speed = 0.25f;
    public static final float driveRedBeacon2Speed = 0.3f;

    public static final float press_red_beacon2_speed = 0.2f;

    public static final float collector_power = -1.0f;

    public static final float startAimPosition = 0.6f;

    public static final float aimLoadPosition = 0.5024f;

    //Calculated aim = shooter constant - (shooterSlope * distance to target)
    public static final float shooterConstant = 0.82f;
    public static final float shooterSlope = 0.0012f;


    public static final float target_shooter_speed_base = 0.32f;
    public static float target_shooter_speed = target_shooter_speed_base;//cool

    public static final float beacon_servo1_on = 1.0f;
    public static final float beacon_servo2_on = 0.0f;
    public static final float beacon_servo1_off = 0.0f;
    public static final float beacon_servo2_off = 1.0f;

    public static float shooterVertical = 0.7691f;

    public static final float deployWheelServo1Up = 0.0f;
    public static final float deployWheelServo1Down = 1.0f;
    public static final float deployWheelServo2Up = 1.0f;
    public static final float deployWheelServo2Down = 0.0f;

}
