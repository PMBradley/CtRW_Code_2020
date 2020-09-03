package org.firstinspires.ftc.teamcode.custom;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/*
    Welcome to the template Provider class!
    To use it just make a copy of this class, and change all appearances of "20XX" with the starting year of the current season
    (for example, for the 2020-2021 season, the class would be named Provider2020 and all code within would have to reflect that change)

    Happy coding!
 */



// The main robot data class - called provider because it provides hardware classes with the robot data that they need to function

public class Provider20XX {
    // Motor and servo variables

    // Drive motors
    public DcMotor driveFL = null;
    public DcMotor driveFR = null;
    public DcMotor driveBL = null;
    public DcMotor driveBR = null;

    // Manipulator motors (motors for other things, not driving) - names are example names, they can be set for whatever application you have
    public DcMotor motorLift = null;
    public DcMotor motorIntakeL = null;
    public DcMotor motorIntakeR = null;

    // Servo Variables - names are example names, they can be set for whatever application you have
    public Servo armPivot = null;
    public Servo armGrab = null;
    public Servo intakeDropL = null;
    public Servo intakeDropR = null;
    public Servo pullerDropL = null;
    public Servo pullerDropR = null;


    // Sensor Variables

    // Touch Sensor variables - it is recommended to change the word "SensorX" in the names with a basic descriptor of what they are for, for example "touchBumper"
    public DigitalChannel touchSensor0;
    public DigitalChannel touchSensor1;
    public DigitalChannel touchSensor2;
    public DigitalChannel touchSensor3;
    public DigitalChannel touchSensor4;
    public DigitalChannel touchSensor5;
    public DigitalChannel touchSensor6;
    public DigitalChannel touchSensor7;

    // Distance Sensor Variables - names are just examples, feel free to change to whatever you like. Just note that "flight" represents the fact that they are Time of Flight sensors (think radar, but with lasers)
    public Rev2mDistanceSensor flightFront0;
    public Rev2mDistanceSensor flightLeft1;
    public Rev2mDistanceSensor flightRight2;
    public Rev2mDistanceSensor flightBack3;

    public BNO055IMU imu; // the IMU class instance

    // The all important hardware map (basically a log of what devices are plugged into what ports. Setup on the FTC Robot Controller app)
    HardwareMap mainMap;


    // Default constructor
    public Provider20XX(HardwareMap hMap){
        init_map(hMap); // pull information from the hardware map - MUST BE DONE BEFORE

        init_imu(); // setup the IMU and calibrate the current position as 0
    }


    // Initialization functions

    private void init_map(HardwareMap hMap){    // setup the hardware map dependant classes (usually by grabbing their components out of the hardware map)
        mainMap = hMap;

        /* NOTE - hardware map requests for hardware that is not in the hardware map (because they are not being used this season, for example),
                  should either be removed or commented out to prevent the computer looking for hardware that isn't there.

                  That said, it is ok to have hardware map requests for hardware that isn't physically plugged in to the hubs at the moment,
                  as long as it is present in the hardware map (again, configured in the FTC Robot Controller app).
                  Just be aware that any attempt to get readings from sensors or motors that aren't plugged in will not be reliable,
                  and motors that aren't plugged in will not move (Don't worry, we've all done that at least once ;)
         */


        // Grabbing motors from hardware map
        driveFL = mainMap.get(DcMotor.class, "driveFL");
        driveFR = mainMap.get(DcMotor.class, "driveFR");
        driveBL = mainMap.get(DcMotor.class, "driveBL");
        driveBR = mainMap.get(DcMotor.class, "driveBR");
        // motorLift = mainMap.get(DcMotor.class, "motorLift");
        // motorIntakeL = mainMap.get(DcMotor.class, "intakeL");
        // motorIntakeR = mainMap.get(DcMotor.class, "intakeR");

        // Set motors to run with encoders (uncomment if you are, comment out if you are not)
        driveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       /* motorIntakeL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorIntakeR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */

        // Grabbing servos from hardware map (uncomment if you are using them, comment out if you are not)
        /*
        armPivot = mainMap.get(Servo.class, "armPivot");
        armGrab = mainMap.get(Servo.class, "armGrab");
        intakeDropL = mainMap.get(Servo.class, "intakeDropL");
        intakeDropR = mainMap.get(Servo.class, "intakeDropR");
        pullerDropL = mainMap.get(Servo.class, "pullerDropL");
        pullerDropR = mainMap.get(Servo.class, "pullerDropR");
        */

        // Grabbing sensors from hardware map (uncomment if you are using them, comment out if you are not)
        /*
        touchSensor0 = mainMap.get(DigitalChannel.class, "touchLift0");
        touchSensor1 = mainMap.get(DigitalChannel.class, "touchArm1");
        touchSensor2 = mainMap.get(DigitalChannel.class, "touchBlock2");
        touchSensor3 = mainMap.get(DigitalChannel.class, "touchLiftUp3");
        touchSensor4 = mainMap.get(DigitalChannel.class, "touchLeft4");
        touchSensor5 = mainMap.get(DigitalChannel.class, "touchRight5");
        touchSensor6 = mainMap.get(DigitalChannel.class, "touchBack6");
        touchSensor7 = mainMap.get(DigitalChannel.class, "touchClamp7");
        */

        // Time of flight sensor setup (uncomment if you are using them, comment out if you are not)
        /*
        flightFront0 = (Rev2mDistanceSensor)mainMap.get(DistanceSensor.class, "flightFront0");
        flightLeft1  = (Rev2mDistanceSensor)mainMap.get(DistanceSensor.class, "flightLeft1");
        flightRight2 = (Rev2mDistanceSensor)mainMap.get(DistanceSensor.class, "flightRight2");
        flightBack3  = (Rev2mDistanceSensor)mainMap.get(DistanceSensor.class, "flightBack3");
        */

        imu = mainMap.get(BNO055IMU.class, "imu");   // get Internal Measurement Unit from the hardware map
    }

    private void init_imu(){ // create a new IMU class instance and set our IMU to that
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "DroopyIMU.json"; // see the calibration sample opmode
        parameters.loggingEnabled      =  true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
    }


    // External utility functions (ones that will be used outside this class)

    public void reset_imu(){ // an alternative name for init_imu, which is in this case public. The purpose of this is to more clearly convey application (you don't need to init the imu as the user, but you may want to reset it)
        init_imu();
    }

    public double getHeading(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle; // get the current heading of the robot in degrees
    }
}

