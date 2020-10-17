package org.firstinspires.ftc.teamcode.hardware.vision.OpenCV;

import org.openftc.easyopencv.OpenCvPipeline;



// This class exists to support flexible Pipeline types from year to year
// - makes it so that only the Pipeline being passed into Vision_OpenCV's constructor has to be changed, not the class itself
// Meaning that from year to year, all that has to be done is a new pipeline class has to be created and used, no modifications to Vision_OpenCV are required
// (feel free to tho, assuming you know what you are doing :)

public abstract class CustomPipeline extends OpenCvPipeline {

    public String getAnalysis(){ // @Override this function in the abstracted version
        return "ERROR: NO DATA TO RETURN - PLEASE CREATE A PIPELINE CLASS THAT EXTENDS THIS CLASS, USING THE PIPELINE_TEMPLATE CLASS AS A BASE.";
    }
}
