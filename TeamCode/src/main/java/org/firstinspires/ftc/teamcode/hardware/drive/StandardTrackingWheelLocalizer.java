package org.firstinspires.ftc.teamcode.hardware.drive;

//import android.support.annotation.NonNull;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;
import org.jetbrains.annotations.Nullable;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {

    public static double TICKS_PER_REV = 8192; // the number of encoder tics per revolution of the encoder - should be available on the encoder's manufacture's website
    public static double WHEEL_RADIUS = 0.7480315; // in inches - the radius of the dead (odometry) wheels
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed - if the dead (odometry) wheel is directly attached to the encoder, just ingore this

    public static double X_MULTIPLIER = 0.9868421052631579; // a distance multiplier for the x axis - tune as needed (see https://learnroadrunner.com for tuning info)
    public static double Y_MULTIPLIER = 1.00; // a distance multiplier for the y axis - tune as needed (see https://learnroadrunner.com for tuning info)

    public static double LATERAL_DISTANCE = 16.66123177; // in; distance between the left and right odometry wheels - update with physical robot changes to distance
    public static double LATERAL_POD_FORWARD_OFFSET = -0.25; // in; forward offset of the lateral odometry wheel - update with physical robot changes to distance - positive is forward of center of rotation
    public static double LATERAL_POD_LEFT_OFFSET = -6.25; // in; left offset of the lataeral odometry wheel - update with physical robot changes to distance - positive is left of center of rotation
    public static double RIGHT_POD_FORWARD_OFFSET = 1.225; // in; forward offset of the right odometry wheel - update with physical robot changes to distance - positive is forward of center of rotation
    public static double LEFT_POD_FORWARD_OFFSET = 1.225; // in; forward offset of the left odometry wheel - update with physical robot changes to distance - positive is forward of center of rotation

    public static double LEFT_POD_ROTATION = Math.toRadians(180.0);
    public static double RIGHT_POD_ROTATION = Math.toRadians(0.0);
    public static double LATERAL_POD_ROTATION = Math.toRadians(90.0);


/* // Test Robot Constants
    public static double TICKS_PER_REV = 8192; // the number of encoder tics per revolution of the encoder - should be available on the encoder's manufacture's website
    public static double WHEEL_RADIUS = 0.7480315; // in inches - the radius of the dead (odometry) wheels
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed - if the dead (odometry) wheel is directly attached to the encoder, just ingore this

    public static double X_MULTIPLIER = 1.00; // a distance multiplier for the x axis - tune as needed (see https://learnroadrunner.com for tuning info)
    public static double Y_MULTIPLIER = 1.00; // a distance multiplier for the y axis - tune as needed (see https://learnroadrunner.com for tuning info)

    public static double LATERAL_DISTANCE = 15.51210412; // in; distance between the left and right odometry wheels - update with physical robot changes to distance
    public static double LATERAL_POD_FORWARD_OFFSET = 3.25; // in; forward offset of the lateral odometry wheel - update with physical robot changes to distance - positive is forward of center of rotation
    public static double LATERAL_POD_LEFT_OFFSET = 0.00; // in; left offset of the lataeral odometry wheel - update with physical robot changes to distance - positive is left of center of rotation
    public static double RIGHT_POD_FORWARD_OFFSET = 2.1; // in; forward offset of the right odometry wheel - update with physical robot changes to distance - positive is forward of center of rotation
    public static double LEFT_POD_FORWARD_OFFSET = 2.1; // in; forward offset of the left odometry wheel - update with physical robot changes to distance - positive is forward of center of rotation
*/

    private Encoder leftEncoder, rightEncoder, lateralEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(LEFT_POD_FORWARD_OFFSET, LATERAL_DISTANCE / 2, LEFT_POD_ROTATION), // left
                new Pose2d(RIGHT_POD_FORWARD_OFFSET, -LATERAL_DISTANCE / 2, RIGHT_POD_ROTATION), // right
                new Pose2d(LATERAL_POD_FORWARD_OFFSET, LATERAL_POD_LEFT_OFFSET, LATERAL_POD_ROTATION) // front
        ));


        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "driveFR")); // the motor name of the associated encoder port
        rightEncoder =  new Encoder(hardwareMap.get(DcMotorEx.class, "driveBR"));
        lateralEncoder =  new Encoder(hardwareMap.get(DcMotorEx.class,"driveBL"));

       /// lateralEncoder.setDirection(Encoder.Direction.REVERSE); // reverse the encoder direction for the lateral encoder
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return super.getPoseVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(lateralEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        //  If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(lateralEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}
