package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Created by Administrator on 7/9/2017.
 */

public class Tracker {
    AnalogInput m_encoder;
    public static final double All_telemetry_trackers_wrapAroundThreshold = 0.3;
    public static final double limit_top = 1;
    public static final double limit_bot = 0;

    double resetReading = 0.0f;
    double currentReading = 0.0f;
    double lastReading = 0.0f;
    double lastValidReading = 0.0f;

    int num_of_wraparounds = 0;

    boolean wrapingAround = false;




    public Tracker(AnalogInput encoder){
        m_encoder = encoder;
        reset();
    }

    public void reset(){
        num_of_wraparounds = 0;
        resetReading = getCurrentReading();
        currentReading = getCurrentReading();
        lastReading = getCurrentReading();
        lastValidReading = getCurrentReading();
    }

    private double getCurrentReading(){
        return Math.abs(m_encoder.getVoltage()/ m_encoder.getMaxVoltage());
    }

    public void update(){
        currentReading = getCurrentReading();
        double delta_temp = currentReading-lastReading;
        if(Math.abs(delta_temp) > All_telemetry_trackers_wrapAroundThreshold){
            wrapingAround = true;
        }else{
            if(wrapingAround){
                wrapingAround = false;
                if(currentReading-lastValidReading >0){
                    num_of_wraparounds --;
                }else{
                    num_of_wraparounds ++;
                }
            }

            lastValidReading = currentReading;
        }



        lastReading = currentReading;
    }
    public double getPos(){
        return (num_of_wraparounds+((lastValidReading-resetReading)-limit_bot)/(limit_top-limit_bot));
    }
    public double getRAW(){
        return currentReading;
    }
}
