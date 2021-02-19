package org.firstinspires.ftc.teamcode.hardware.vision.OpenCV;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


@Config
public class FarRingStackHeightPipeline extends CustomPipeline{
    /*
     * An enum to defined for use with ring count
     */
    public enum RingAmount
    {
        NONE,
        ONE,
        MOST
    }

    public static double min_avg = 137; // tune this value to get consistent color comparisons (all color values lower than it will be seen as a place where a ring is)

    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar ORANGE = new Scalar(255, 153, 0);
    static final Scalar BLACK = new Scalar(0, 0, 0);

    /*
     * The core values which define the location and size of the sample regions
     */
    public static Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(128, 195);
    public static Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(128,230);
    public static int REGION_WIDTH = 90;
    public static int REGION_HEIGHT = 10;

    /*
     * Points which actually define the sample region rectangles, derived from above values
     *
     * Example of how points A and B work to define a rectangle
     *
     *   ------------------------------------
     *   | (0,0) Point A                    |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                  Point B (70,50) |
     *   ------------------------------------
     *
     */
    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);


    /*
     * Working variables
     */
    Mat region1_Cb, region2_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    int avg1, avg2;



    // Volatile since accessed by OpMode thread w/o synchronization
    private volatile RingAmount amount = RingAmount.NONE;

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */
    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 0);
    }

    @Override
    public void init(Mat firstFrame)
    {
        /*
         * We need to call this in order to make sure the 'Cb'
         * object is initialized, so that the submats we make
         * will still be linked to it on subsequent frames. (If
         * the object were to only be initialized in processFrame,
         * then the submats would become delinked because the backing
         * buffer would be re-allocated the first time a real frame
         * was crunched)
         */
        inputToCb(firstFrame);

        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */
        region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
    }

    @Override
    public Mat processFrame(Mat input)
    {
        /*
         * Overview of what we're doing:
         *
         * We first convert to YCrCb color space, from RGB color space.
         * Why do we do this? Well, in the RGB color space, chroma and
         * luma are intertwined. In YCrCb, chroma and luma are separated.
         * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
         * are Y, the luma channel (which essentially just a B&W image), the
         * Cr channel, which records the difference from red, and the Cb channel,
         * which records the difference from blue. Because chroma and luma are
         * not related in YCrCb, vision code written to look for certain values
         * in the Cr/Cb channels will not be severely affected by differing
         * light intensity, since that difference would most likely just be
         * reflected in the Y channel.
         *
         * After we've converted to YCrCb, we extract just the 2nd channel, the
         * Cb channel. We do this because rings are bright orange and contrast
         * STRONGLY on the Cb channel against everything else.
         *
         * We then look at two regions: the region that a single ring would be,
         * and a region that a top ring would be. We get the average values from
         * those regions, then compare that average to the minimum average. If
         * the measured average is greater than or equal to the minimum average
         * value, we assume that there is a ring in that space. If there is a
         * ring in the top space, we know there is a full stack. If there is a
         * ring in the one ring space and none at the top space, we know there
         * is only one ring. If there are no rings at all, then there are no rings.
         *
         * We also draw rectangles on the screen showing where the sample regions
         * are, for visualization. Additionally, if there are
         *
         * In order for this whole process to work correctly, each sample region
         * should be positioned in a solid color region of the top and bottom rings,
         * and be small enough such that only the ring is sampled, and not any of the
         * surroundings.
         */

        /*
         * Get the Cb channel of the input frame after conversion to YCrCb
         */
        inputToCb(input);

        /*
         * Compute the average pixel value of each submat region. We're
         * taking the average of a single channel buffer, so the value
         * we need is at index 0. We could have also taken the average
         * pixel value of the 3-channel image, and referenced the value
         * at index 2 here.
         */
       /* full_avg1 = Core.mean(region1_Cb);
        full_avg2 = Core.mean(region2_Cb);*/

        avg1 = (int) Core.mean(region1_Cb).val[0];
        avg2 = (int) Core.mean(region2_Cb).val[0];

        /*
         * Draw a rectangle showing sample region 1 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLACK, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 2 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region2_pointA, // First point which defines the rectangle
                region2_pointB, // Second point which defines the rectangle
                BLACK, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines



        /*
         * Now that we found the values, we actually need to go and
         * figure out how they compare to the minimum average color value
         */
        if(avg2 < min_avg){ // Is there enough yellow in the top region
            amount = RingAmount.MOST; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    ORANGE, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }
        else if(avg1 < min_avg) { // Was it from region 2?
            amount = RingAmount.ONE; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    ORANGE, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }
        else { // if neither had enough yellow
            amount = RingAmount.NONE; // Record our analysis
        }

        /*
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */
        return input;
    }

    /*
     * Call this from the OpMode thread to obtain the latest analysis - this is the overriding of the CustomPipeline class
     */

    @Override
    public String getAnalysis()
    {
        return enumToNumString(amount) + " " + avg1 + ", " + avg2;
    }

    public String enumToNumString(RingAmount amount){
        switch (amount.toString()){
            case "NONE":
                return "0 Rings";

            case "ONE":
                return "1 Ring";

            case "MOST":
                return "4 Rings";

            default:
                return "4 Rings";
        }
    }
}
