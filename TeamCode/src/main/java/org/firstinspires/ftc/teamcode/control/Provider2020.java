package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.hardware.drive.DriveConstants;


/*
    Welcome to the template Provider class!
    To use it just make a copy of this class, and change all appearances of "20XX" with the starting year of the current season
    (for example, for the 2020-2021 season, the class would be named Provider2020 and all code within would have to reflect that change)

    Happy coding!
 */



// The main robot data class - called provider because it provides hardware classes with the robot data that they need to function

public class Provider2020 {
    // Motor and servo variables

    // Drive motors
    public DcMotor driveFL;
    public DcMotor driveFR;
    public DcMotor driveBL;
    public DcMotor driveBR;

    // Manipulator motors (motors for other things, not driving) - names are example names, they can be set for whatever application you have
    public DcMotor shooterMotor;
    public DcMotor intakeMotor;
    public DcMotor wobbleArmMotor;
    public DcMotor wobbleArmMotor2;

    // Servo Variables - names are example names, they can be set for whatever application you have
    public Servo intakeLockServo;
    public Servo shooterFeederServo;
    public Servo wobbleLeftWheelServo;
    public Servo wobbleRightWheelServo;
    public Servo wobbleClampServo;

    // Sensor Variables

    // Touch Sensor variables - it is recommended to change the word "SensorX" in the names with a basic descriptor of what they are for, for example "touchBumper"
    public DigitalChannel touchSensor0;

    // Distance Sensor Variables - names are just examples, feel free to change to whatever you like. Just note that "flight" represents the fact that they are Time of Flight sensors (think radar, but with lasers)
    public Rev2mDistanceSensor flightFront0;
    public Rev2mDistanceSensor flightLeft1;


    public BNO055IMU imu; // the IMU class instance

    // The all important hardware map (basically a log of what devices are plugged into what ports. Setup on the FTC Robot Controller app)
    HardwareMap mainMap;

    // Flag values
    public boolean driveUsingEncoders = false;
    private boolean oneHubMode;


    // Hardware map only Contructor
    public Provider2020(HardwareMap hMap){
        this(hMap, false); // call the specific constructor setting oneHubMode to false
    }

    // Hardware map and encoder Constructor
    public Provider2020(HardwareMap hMap, boolean oneHubMode){
        this.oneHubMode = oneHubMode;

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
        // THis is a test


        // Grabbing motors from hardware map
        driveFL = mainMap.get(DcMotor.class, "driveFL");
        driveFR = mainMap.get(DcMotor.class, "driveFR");
        driveBL = mainMap.get(DcMotor.class, "driveBL");
        driveBR = mainMap.get(DcMotor.class, "driveBR");

        // Set motors to run with encoders (if the flag is true)
        if(driveUsingEncoders){
            driveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


       // Reverse drive motor directions as needed
        driveFL.setDirection(DcMotor.Direction.REVERSE);
        driveBL.setDirection(DcMotor.Direction.REVERSE);


        if(!oneHubMode){ // if using more than one hub, setup the other motors and servos
            // Grabbing the motors from the hardware map
            shooterMotor = mainMap.get(DcMotor.class, "shooterMotor");
            intakeMotor = mainMap.get(DcMotor.class, "intakeMotor");
            wobbleArmMotor = mainMap.get(DcMotor.class, "wobbleArmMotor");
            wobbleArmMotor2 = mainMap.get(DcMotor.class, "wobbleArmMotor2");


            shooterMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.FLOAT ); // don't halt the motor actively for the shooter

       //     shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // set these  motors to run using encoders
            wobbleArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wobbleArmMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);


            // Grabbing the servos from the hardware map
            intakeLockServo = mainMap.get(Servo.class, "intakeLockServo");
            shooterFeederServo = mainMap.get(Servo.class, "feederServo");
            wobbleLeftWheelServo = mainMap.get(Servo.class, "wobbleLeftWheelServo");
            wobbleRightWheelServo = mainMap.get(Servo.class, "wobbleRightWheelServo");
            wobbleClampServo = mainMap.get(Servo.class, "wobbleClampServo");

            wobbleLeftWheelServo.getController().pwmDisable();
            wobbleRightWheelServo.getController().pwmDisable();

            wobbleClampServo.getController().pwmEnable();

            wobbleLeftWheelServo.setDirection(Servo.Direction.REVERSE);
        }



        // Grabbing sensors from hardware map (uncomment if you are using them, comment out if you are not)
        /*
        touchSensor0 = mainMap.get(DigitalChannel.class, "touchLift0");
        */

        // Time of flight sensor setup (uncomment if you are using them, comment out if you are not)
        /*
        flightFront0 = (Rev2mDistanceSensor)mainMap.get(DistanceSensor.class, "flightFront0");
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

    public void setEncoderActive(boolean usingEncoders){ // sets encoder mode based off of boolean parameter
        driveUsingEncoders = usingEncoders;

        if(driveUsingEncoders){ // enable controllers if true
            driveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else{ // disable controllers if false
            driveFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}

