package org.firstinspires.ftc.teamcode.hardware.vision.OpenCV;

import org.opencv.core.Mat;

public class DefaultPipeline extends CustomPipeline{


    @Override
    public Mat processFrame(Mat input) {
        // Do any processing of the frame here

        return input;
    }

    @Override
    public String getAnalysis(){
        return "This is a default pipeline, I don't know what you were expecting.";
    }
}
