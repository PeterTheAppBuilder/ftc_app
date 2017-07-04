package ftc.vision;

import android.content.Context;
import android.hardware.Camera;
import android.hardware.camera2.CameraMetadata;
import android.hardware.camera2.CaptureRequest;
import android.util.AttributeSet;

import org.opencv.android.JavaCameraView;

import static android.hardware.Camera.Parameters.FOCUS_MODE_FIXED;

/**
 * Created by Administrator on 6/12/2017.
 */

public class cameraSettings extends JavaCameraView {
    public cameraSettings(Context context, AttributeSet attrs) {
        super(context, attrs);
    }


    // Setup the camera
    public void turnOffAutoExposure() {
        //mCamera = Camera.open(mCameraIndex());
        //Camera camera = mCamera;
        ;
        /*
        try{
        }catch(Exception e){

        }*/

        if(mCamera!=null){
            mCamera.autoFocus(new Camera.AutoFocusCallback(){
                @Override
                public void onAutoFocus(boolean b, Camera camera){};
            });
            Camera.Parameters params = mCamera.getParameters();
            params.setAutoExposureLock(true);
            mCamera.setParameters(params);

        }



    }
}
