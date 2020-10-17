package org.firstinspires.ftc.teamcode.hardware.vision.OpenCV;

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


    public Vision_OpenCV_ExternalCam(HardwareMap hardwareMap){ // most basic constructor, just inits the camera, using the default device name
        initWebcam(hardwareMap); // init the webcam
    }
    public Vision_OpenCV_ExternalCam(HardwareMap hardwareMap, String device_name){ // next most basic constructor, inits the camera, but sets a custom device name
        this.device_name = device_name; // set the class's device name to the passed in device name

        initWebcam(hardwareMap); // init the webcam
    }
    public Vision_OpenCV_ExternalCam(HardwareMap hardwareMap, CustomPipeline new_pipeline) { // a constructor that also sets a new pipeline, uses the defualt device name
        setWebcamPipeine(new_pipeline); // set the camera to use the input pipeline

        initWebcam(hardwareMap); // init the webcam
    }
    public Vision_OpenCV_ExternalCam(HardwareMap hardwareMap, String device_name, CustomPipeline new_pipeline){ // most complex constructor, uses a custom device name and sets up the pipeline
        setWebcamPipeine(new_pipeline); // set the camera to use the input pipeline

        this.device_name = device_name; // set the class's device name to the passed in device name

        initWebcam(hardwareMap); // init the webcam
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
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT); // start the streaming process
    }

    public void setWebcamPipeine(CustomPipeline new_pipeline){
        pipeline = new_pipeline;
        webcam.setPipeline(pipeline);
    }


    public String getOutput(){ // the whole point of the class, this is where you finally access and get an output
        return pipeline.getAnalysis();
    }

}
