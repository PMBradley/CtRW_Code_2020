package org.firstinspires.ftc.teamcode.hardware.drive;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.util.TrajectoryIntDuoHolder;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.hardware.drive.DriveConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.hardware.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.hardware.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.hardware.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.hardware.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.hardware.drive.DriveConstants.getMotorVelocityF;
import static org.firstinspires.ftc.teamcode.hardware.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.hardware.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.hardware.drive.DriveConstants.kV;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
*/
@Config
public class Drive_Mecanum_Auto extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);


    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

    private FtcDashboard dashboard;
    private NanoClock clock;

    private Mode mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    private DriveConstraints constraints;
    private TrajectoryFollower follower;

    private List<Pose2d> poseHistory;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private BNO055IMU imu;

    private int TELEMETRY_TRANSMISSION_INTERVAL = 25; // how often telemetry is sent information (if information to send as per .update()) - in milliseconds

    private ElapsedTime localRuntime;

    // Main costructor
    public Drive_Mecanum_Auto(HardwareMap hardwareMap, boolean usingOdometry) { // only set usingOdometry to true if using odometry pods and they are hooked up properly
        super(kV, kA, kStatic, TRACK_WIDTH); // call the superclass constructor (MecanumDrive) and pass it important setup variables

        dashboard = FtcDashboard.getInstance(); // setup the dashboard
        dashboard.setTelemetryTransmissionInterval(TELEMETRY_TRANSMISSION_INTERVAL); // interval in milliseconds

        clock = NanoClock.system(); // setup the clock

        mode = Mode.IDLE; // have the default drive mode be idle, ensure it is that

        setupDrive(hardwareMap, usingOdometry); // setup the drive systems
    }
    public Drive_Mecanum_Auto(HardwareMap hardwareMap){
        super(kV, kA, kStatic, TRACK_WIDTH); // call the superclass constructor (MecanumDrive) and pass it important setup variables

        dashboard = FtcDashboard.getInstance(); // setup the dashboard
        dashboard.setTelemetryTransmissionInterval(TELEMETRY_TRANSMISSION_INTERVAL); // interval in milliseconds

        clock = NanoClock.system(); // setup the clock

        mode = Mode.IDLE; // have the default drive mode be idle, ensure it is that

        setupDrive(hardwareMap, false); // setup the drive, including RoadRunner - if not given whether or not we are using odometry, pass in that we are not
    }


    private void setupDrive(HardwareMap hardwareMap, boolean usingOdometry){
        localRuntime.reset();

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        poseHistory = new ArrayList<>();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.get(DcMotorEx.class, "driveFL");
        rightFront = hardwareMap.get(DcMotorEx.class, "driveFR");
        leftRear = hardwareMap.get(DcMotorEx.class, "driveBL");
        rightRear = hardwareMap.get(DcMotorEx.class, "driveBR");


        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        // if desired, and told so via the parameter useOdometry, use setLocalizer() to change the localization method
        if (usingOdometry){
            setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap)); // feed (pass) it the hardwareMap - nom nom nom
        }
    }



    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, constraints);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, constraints);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, constraints);
    }



    public void turnAsync(double angle) {
        double heading = getPoseEstimate().getHeading();
        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                constraints.maxAngVel,
                constraints.maxAngAccel,
                constraints.maxAngJerk
        );
        turnStart = clock.seconds();
        mode = Mode.TURN;
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }



    public void followTrajectoryAsync(Trajectory trajectory) {
        follower.followTrajectory(trajectory);
        mode = Mode.FOLLOW_TRAJECTORY;
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return new Pose2d();
        }
        throw new AssertionError();
    }

    public void update() {
        updatePoseEstimate();

        Pose2d currentPose = getPoseEstimate();
        Pose2d lastError = getLastError();

        poseHistory.add(currentPose);

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        packet.put("mode", mode);

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading", currentPose.getHeading());

        packet.put("xError", lastError.getX());
        packet.put("yError", lastError.getY());
        packet.put("headingError", lastError.getHeading());

        switch (mode) {
            case IDLE:
                // do nothing
                break;
            case TURN: {
                double t = clock.seconds() - turnStart;

                MotionState targetState = turnProfile.get(t);

                turnController.setTargetPosition(targetState.getX());

                double correction = turnController.update(currentPose.getHeading());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();
                setDriveSignal(new DriveSignal(new Pose2d(
                        0, 0, targetOmega + correction
                ), new Pose2d(
                        0, 0, targetAlpha
                )));

                if (t >= turnProfile.duration()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
            case FOLLOW_TRAJECTORY: {
                setDriveSignal(follower.update(currentPose));

                Trajectory trajectory = follower.getTrajectory();

                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("4CAF50");
                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());
                double t = follower.elapsedTime();
                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));

                fieldOverlay.setStroke("#3F51B5");
                DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);
                DashboardUtil.drawRobot(fieldOverlay, currentPose);

                if (!follower.isFollowing()) {
                    mode = Mode.IDLE;
                    setDriveSignal(new DriveSignal());
                }

                break;
            }
        }

        dashboard.sendTelemetryPacket(packet);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy()) {
            update();
        }
    }

    public boolean isBusy() {
        return mode != Mode.IDLE;
    }
    public boolean isFollowing(){ return mode == Mode.FOLLOW_TRAJECTORY; }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, getMotorVelocityF()
            ));
        }
    }


    // State machine controlled asynchronous follow
    private ArrayList<TrajectoryIntDuoHolder> tasks; // an arraylist that holds the list of tasks for
    private int taskIndex = 0; // a number that keeps track of where in the task list we are

    private boolean firstTaskRun = true; // first run flag for doing tasks, ensure proper behavior

    private double waitEndTime; // the time that the program designates as the time to go ahead and move to the next task (in milliseconds)

    public void setTasks(ArrayList<TrajectoryIntDuoHolder> newTasks){
        tasks = newTasks; // set the tasks like promest
        taskIndex = 0; // reset the task index to ensure that everything goes well with the new job
    }
    public ArrayList<TrajectoryIntDuoHolder> getTasks(){ return tasks; } // gets the whole list of tasks
    public TrajectoryIntDuoHolder getTaskAt(int index){return tasks.get(index); } // gets a specified task from the list
    public TrajectoryIntDuoHolder getCurrentTask(){ return tasks.get(taskIndex); } // gets the current task (not the variable, but what it is according to the index)
    public int getTaskIndex(){ return taskIndex; } // get what the current task is

    public boolean doTasksAsync(){ // the main state machine function that runs through each task - when complete it returns true
        boolean allComplete = false;

        if( taskIndex < tasks.size() ){ // if still within the bounds of the task list
            boolean taskComplete = false; // indicates if the current task is complete yet (default is false)

            TrajectoryIntDuoHolder currentTask = getTaskAt(taskIndex); // get the current task and set the currentTask variable to it

            if(currentTask.getTraj() != null){ // if there is a trajector to follow, follow dat trajectory
                if(firstTaskRun){ // only on the first run of the task
                    followTrajectoryAsync( currentTask.getTraj() ); // set the follower to follow the current task trajectory
                }
                update(); // actually do the following of the trajectory, also update any important information, including if we are done following or not

                taskComplete = !isFollowing(); // if isFollowing returns true that means that we are still going and therefore taskComplete will be false, and visa versa
            }
            else if (currentTask.getNum() > 0){ // if there is no trajectory to follow but there is a time, wait
                if(firstTaskRun){
                    waitEndTime = localRuntime.milliseconds() + currentTask.getNum(); // set the target time to the current time plus the input number of milliseconds
                }

                taskComplete = (localRuntime.milliseconds() >= waitEndTime); // if runtime is greater than or equal to the set waitEndTime, taskComplete is set to true, otherwise it is set to false
            }
            else {
                taskComplete = true; // else if there are neither, we just go to the next one and pretend this one didn't happen
            }

            if(firstTaskRun){ // if it is the end of the first loop run for the new task
                firstTaskRun = false; // set the flag to no longer show it is the first run of this task
            }
            if(taskComplete){ // if the current task is complete
                taskIndex++; // go to the next task

                firstTaskRun = true;
            }
        } // end of task doing if
        else { // if we are complete, as the task index has exeded the number of tasks we have
            allComplete = true;
        }

        return allComplete; // return
    }


    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}
