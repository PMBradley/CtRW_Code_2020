package org.firstinspires.ftc.teamcode.hardware.vision.OpenCV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


public class Vision_OpenCV_ExternalCam {

    public OpenCvCamera webcam;
    CustomPipeline pipeline;

    private String device_name = "Webcam 1";

    public static final int CAMERA_RESOLUTION_X = 640; // the camera x resolution dimensions that will be used
    public static final int CAMERA_RESOLUTION_Y = 480; // the camera y resolution dimensions that will be used
    private static final OpenCvCameraRotation CAMERA_ROTATION = OpenCvCameraRotation.UPSIDE_DOWN; // which direction is up for the camera

    private boolean cameraStreaming = false;

    public Vision_OpenCV_ExternalCam(HardwareMap hardwareMap){ // most basic constructor, just inits the camera, using the default device name
        initWebcam(hardwareMap); // init the webcam
    }
    public Vision_OpenCV_ExternalCam(HardwareMap hardwareMap, String device_name){ // next most basic constructor, inits the camera, but sets a custom device name
        this.device_name = device_name; // set the class's device name to the passed in device name

        initWebcam(hardwareMap); // init the webcam
    }
    public Vision_OpenCV_ExternalCam(HardwareMap hardwareMap, CustomPipeline new_pipeline) { // a constructor that also sets a new pipeline, uses the defualt device name

        initWebcam(hardwareMap); // init the webcam
        setWebcamPipeine(new_pipeline); // set the camera to use the input pipeline
    }
    public Vision_OpenCV_ExternalCam(HardwareMap hardwareMap, String device_name, CustomPipeline new_pipeline){ // most complex constructor, uses a custom device name and sets up the pipeline
        this.device_name = device_name; // set the class's device name to the passed in device name

        initWebcam(hardwareMap); // init the webcam
        setWebcamPipeine(new_pipeline); // set the camera to use the input pipeline
    }


    public void initWebcam(HardwareMap hardwareMap){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, device_name), cameraMonitorViewId);

        if (pipeline == null) { // if the pipeline is null, use the first ever pipeline (But know that it will not likely function as wanted)
            pipeline = new RingStackHeightPipeline(); // TODO: create a template pipeline to use in place of the first one here
        }

        webcam.setPipeline(pipeline); // make sure the pipeline is set

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        // webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDevice();
        startWebcamStreaming();

        FtcDashboard.getInstance().startCameraStream(webcam, 0); // init the dashboard stream (0 max fps means no limit)
    }

    public void setWebcamPipeine(CustomPipeline new_pipeline){
        pipeline = new_pipeline;
        webcam.setPipeline(pipeline);
    }


    public String getOutput(){ // the whole point of the class, this is where you finally access and get an output
        return pipeline.getAnalysis();
    }


    public void startWebcamStreaming(){
        webcam.startStreaming(CAMERA_RESOLUTION_X, CAMERA_RESOLUTION_Y, CAMERA_ROTATION); // start the streaming process
        cameraStreaming = true; // update the flag
    }
    public void stopWebcamStreaming(){
        webcam.stopStreaming(); // stop the streaming process
        cameraStreaming = false; // update the flag
    }
    public boolean isCameraStreaming(){
        return cameraStreaming;
    }

}
