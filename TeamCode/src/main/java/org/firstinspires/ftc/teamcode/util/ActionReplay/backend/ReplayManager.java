package org.firstinspires.ftc.teamcode.util.ActionReplay.backend;

import android.os.Environment;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Scanner;


@Config
public class ReplayManager {
    // Constants
    public static int MAX_LOADED_STATES = 10_000; // the largest the manager will let the states lists get, should hold around 70 seconds of states
    public static double PATH_REFRESH_TRHESHOLD = 0.05; // when 5% of the loaded path has been traversed, the robot unloads the PATH_UNLOAD_PERCENTAGE of the path that is behind it and reloads that much to the front of the path
    public static double PATH_UNLOAD_PERCENTAGE = 0.10; // how much of the path behind us unloads when the path refresh threshold is reached
    public static double LOOK_AHEAD_MSEC = 0.0; // how many milliseconds ahead on the path the robot will try to go to, to prevent lagging behind the timestamp
    public static final String STORAGE_DIRECTORY = "ReplaySaves";
    public static int MIN_RECORD_INTERVAL = 100;
    public static int REPLAY_BUFFER_TIME = 2500;

    // Recording Objects
    private ArrayList<RobotState> recordedStatesHistory ; // a list of recorded states, used for drawing out where we have been while recording
    private ArrayList<RobotState> replayStates; // a list of states that is followed, loaded ahead of where we are
    private ConcurrentArrayList<RobotState> replayLoadedStates; // temporarily holds the states before they get added into the list
    private File statesFile;
    //private BufferedReader stateReader;
    private Scanner stateReader;
    private FileWriter stateWriter;
    private ElapsedTime replayTimer;
    private LoadManager loader; // the threaded object that loads states
    private ElapsedTime timeSinceLastRecord;

    // Flags
    private boolean recording = false;
    private String recordingMode = "NONE";// if not recording "NONE", if recording: "Auto[number]" = Recording for Autonomous path [number], "Tele[number]" = Recording for TeleOp path [number]"
    //     examples: "Auto1", "Auto2", "Auto3", "Auto4", "Tele1", "Tele2", "Tele3", "Tele4"
    private boolean replaying = false;
    private boolean loadBufferTimeCompleted = false;
    private String replayMode = "NONE";// if not recording "NONE", "Tele[number]" = replaying for TeleOp path [number]"
    //     examples: "Tele1", "Tele2", "Tele3", "Tele4"




    public ReplayManager() { // if no path
        this("NO FILE");
    }
    public ReplayManager(String fileName) {
        recordedStatesHistory = new ArrayList<RobotState>();
        replayStates = new ArrayList<RobotState>();

        if(fileName.equals("NO FILE")){
            statesFile = null;
        }
        else {
            setStatesFile(fileName);
        }

        loader = new LoadManager();
        replayTimer = new ElapsedTime();
        timeSinceLastRecord = new ElapsedTime();
    }

    public void updateModesFromGamepad(Gamepad gamepad1, Telemetry telemetry){
        if(gamepad1.b){ // if the b button is pressed, stop all recording and reset modes
            if(recording)
                this.stopRecording();
            if(replaying)
                this.stopStateReplay();
            replayMode = "NONE";
            recordingMode = "NONE";
        }
        else if(replayMode.equals("NONE") && recordingMode.equals("NONE")){ // if not replaying, not recordidng, not primed to record, and not canceling
            if(gamepad1.x){ // prime to record a teleop path
                recordingMode = "Tele";
            }
            else if(gamepad1.a){ // prime to record an autonomous path
                recordingMode = "Auto";
            }

            // replay teleop recordings on command
            if(gamepad1.dpad_up) { // start replaying path 1 if pressed
                replayMode = "Tele1";
                setStatesFile("Tele1.bin");
                startStateReplay();
            }
            else if(gamepad1.dpad_right) { // start replaying path 2 if pressed
                replayMode = "Tele2";
                setStatesFile("Tele2.bin");
                startStateReplay();
            }
            else if(gamepad1.dpad_down) { // start replaying path 3 if pressed
                replayMode = "Tele3";
                setStatesFile("Tele3.bin");
                startStateReplay();
            }
            else if(gamepad1.dpad_left){ // start replaying path 4 if pressed
                replayMode = "Tele4";
                setStatesFile("Tele4.bin");
                startStateReplay();
            }
        }
        else if (recordingMode.equals("Auto") || recordingMode.equals("Tele")){
            if(gamepad1.dpad_up) { // start recording path 1 with the appropriate auto/teleop mode
                recordingMode += "1";
                setStatesFile(recordingMode + ".bin");
                startRecording();
            }
            else if(gamepad1.dpad_right) { // start recording path 2 with the appropriate auto/teleop mode
                recordingMode += "2";
                setStatesFile(recordingMode + ".bin");
                startRecording();
            }
            else if(gamepad1.dpad_down) { // start recording path 3 with the appropriate auto/teleop mode
                recordingMode += "3";
                setStatesFile(recordingMode + ".bin");
                startRecording();
            }
            else if(gamepad1.dpad_left){ // start recording path 4 with the appropriate auto/teleop mode
                recordingMode += "4";
                setStatesFile(recordingMode + ".bin");
                startRecording();
            }
        }


        // add telemetry relating to robot drive mode so they know what to do
        if(isReplaying()){
            telemetry.addLine("Automatically Replaying Path: " + replayMode);
            telemetry.addLine("Press B to stop replaying.\n");
        }
        else if(isRecording()){
            telemetry.addLine("Recording Robot Movements and Gamepad Inputs for: " + recordingMode);
            telemetry.addLine("Press B to stop recording.\n");
        }
        else if(recordingMode.equals("Tele") || recordingMode.equals("Auto")){ // if recording is primed but not started, prompt them to select a path number to record for
            telemetry.addLine("Almost ready to record a " + recordingMode + " path. Press the appropriate D-Pad direction to choose a path number:");
            telemetry.addLine("Up = Path 1, Right = Path 2, Down = Path 3, Left = Path 4\n");
        }
        else { // the default telemetry menu promps
            telemetry.addLine("Not Recording or Replaying.\n\nPress X to prime recording a Teleop path\nPress A to prime recording an Autonomous path\nPress a D-Pad direction to start replaying a Teleop Path:");
            telemetry.addLine("Up = Path 1, Right = Path 2, Down = Path 3, Left = Path 4\n");
        }
    }
    private void setRecordingMode(String mode){
        recordingMode = mode;
    }
    private void setReplayMode(String mode){
        replayMode = mode;
    }
    public String getRecordingMode(){
        return recordingMode;
    }
    public String getReplayMode(){
        return replayMode;
    }

    public boolean startRecording() {
        if(!recording){
            stopStateReplay(); // stop any any previous replaying and make sure everything is cleared
            recordedStatesHistory = new ArrayList<RobotState>();

            if(statesFile != null){ // only attempt to open the file and start the process if we have a file to open
                try {
                    stateWriter = new FileWriter(statesFile);
                }
                catch (IOException e){ return false; }

                recording = true;
                replayTimer.reset();


                return true;
            }

        }

        return false;
    }
    public boolean recordRobotState(RobotState currentState) {
        if(recording && timeSinceLastRecord.milliseconds() > MIN_RECORD_INTERVAL){// only want to do these things if we are recording and has been enough time since recording last, if not this could cause errors as objects may not have been yet setup properly
            try {
                stateWriter.write(currentState.toCSVLine() + "\n"); // save the current state
            }
            catch (IOException e) { return false; }

            recordedStatesHistory.add(currentState); // add the current state to the states history in memory (doesn't impact the file)

            while(recordedStatesHistory.size() > MAX_LOADED_STATES){ // then remove from the earliest point in the state history until we have a list less than the max list size
                recordedStatesHistory.remove(0); // as a note, this only impacts what is held in memory for telemetry purposes, the states stored in the file aren't impacted by this removal
            }

            timeSinceLastRecord.reset();
        }

        return true;
    }
    public boolean stopRecording()  {
        if(recording){ // only want to do these things if we were recording, if not this could cause errors
            try { // attempt to close the file, then if it fails return false
                stateWriter.close();
            }
            catch (IOException e) {
                return false;
            }
        }

        recording = false;

        return true; // return true if successful
    }

    public static int linesToLoad = -1;
    public boolean startStateReplay() {
        if(!replaying) {
            stopRecording();  // stop any recording and make sure everything is cleared
            replayStates = new ArrayList<RobotState>();
            replayLoadedStates = new ConcurrentArrayList<RobotState>();


            if (statesFile != null) {
                try {
                    stateReader = new Scanner(statesFile); // attempt to make a buffered file reader to read out the contents of the file, if it fails, return false
                } catch (IOException e) {
                    return false;
                }

                replaying = true; // set replaying to true so that loadStates will load states (as it checks if replaying is true before loading states)
                loader.loadStates(stateReader, replayLoadedStates); // load as many states as we can into our replayStates list for following, runs in a background thread constantly
                //replayStates = previousRecordedStates;

                loadBufferTimeCompleted = false;
                replayTimer.reset();

                return true;
            }
        }

        return false;
    }
    public RobotState getCurrentTargetState(){
        //telem.addLine("Has a file open? " + (statesFile != null));
        if(replayTimer.milliseconds() > REPLAY_BUFFER_TIME && !loadBufferTimeCompleted) {
            loadBufferTimeCompleted = true;
            replayTimer.reset();
        }

        if(replaying && loadBufferTimeCompleted){
            if(replayStates.size() > 0)
                addReplayStatesFrom(replayLoadedStates, replayStates.get(replayStates.size() - 1).getTimestamp()); // if an existing list, grab only states with timestamps after the current last timestamp
            else
                addReplayStatesFrom(replayLoadedStates);
        }

        if(replaying && replayStates.size() > 1 && loadBufferTimeCompleted){
            int manipulatorStateEndIndex = 1; // same as below but for everything except for driving
            int driveTimeChunkEndIndex = 1; // the index of the first state that our drive time is after or equal to, aka the beginning of the current time chunk we are driving in
            double currentTime = replayTimer.milliseconds();
            double effectiveDriveTime = currentTime + LOOK_AHEAD_MSEC;

            // having separate states for each of
            while(manipulatorStateEndIndex < replayStates.size() - 1 && replayStates.get(manipulatorStateEndIndex).getTimestamp() < currentTime ){ // move the timeChunkStart index forward until we reach a state with a timestamp that is greater than our current timestamp
                manipulatorStateEndIndex++;
            } // once we reach the state with a timestamp that is greater, we have found the end of our time chunk

            RobotState manipState1 = replayStates.get(manipulatorStateEndIndex - 1);
            RobotState manipState2 = replayStates.get(manipulatorStateEndIndex);
            double manipStatesFractionBetween = RepRecMath.getFractionBetween(manipState1.getTimestamp(), manipState2.getTimestamp(), currentTime);
            //GamepadState gamepad1State = GamepadState.getGamepadstateBetween(manipState1.getGamepad1State(), manipState2.getGamepad1State(), manipStatesFractionBetween);
            GamepadState gamepad2State = GamepadState.getGamepadstateBetween(manipState1.getGamepad2State(), manipState2.getGamepad2State(), manipStatesFractionBetween);
            GamepadState gamepad1State = manipState1.getGamepad1State();

            while(driveTimeChunkEndIndex < replayStates.size() - 1 && replayStates.get(driveTimeChunkEndIndex).getTimestamp() < effectiveDriveTime ){ // move the timeChunkStart index forward until we reach a state with a timestamp that is greater than our current timestamp
                driveTimeChunkEndIndex++;
            } // once we reach the state with a timestamp that is greater, we have found the end of our time chunk
            RobotState firstDriveState = replayStates.get(driveTimeChunkEndIndex - 1);
            RobotState secondDriveState = replayStates.get(driveTimeChunkEndIndex);


            if(driveTimeChunkEndIndex > MAX_LOADED_STATES * PATH_REFRESH_TRHESHOLD){ // if we traversed forwards in the list enough, the current time chunk is forward in the list so to save space we want to unload any states behind the current chunk (and load in future ones to replacec them)
                int distanceFromListFront = driveTimeChunkEndIndex - 1;
                int unloadCount = (int)(distanceFromListFront*PATH_UNLOAD_PERCENTAGE); // unload PATH_UNLOAD_PERCENTAGE of the states behind us, rather than all, preventing any ConcurrentModificationErrors

                for(int i = 0; i < unloadCount; i++){ // remove from the font until the front is the current chunk
                    replayStates.remove(0);
                }
            }

            // since we have removed all states before this chunk, the start and end of this chunk are always indexes 0 and 1. now we just get where we are between them
            return RobotState.getStateBetween(firstDriveState, secondDriveState, effectiveDriveTime, gamepad1State, gamepad2State);
        }
        else {
            return new RobotState();
        }
    }
    public void stopStateReplay(){
        if(replaying){ // only want to close the reader if we were replaying, if not this could cause errors

            stateReader.close();

        }

        replaying = false;
    }




    // a bunch of accessor methods so that other classes can get information about this class
    public boolean isRecording(){return recording;}
    public boolean isReplaying(){return replaying;}
    public ArrayList<RobotState> getRecordedStatesHistory(){return recordedStatesHistory;}
    public ArrayList<Pose2d> getRecordedPositionsHistory(){ // returns the states history except just the positions from each state, allows for easy plotting in dashboard for example
        ArrayList<Pose2d> recordedPositions = new ArrayList<Pose2d>();

        for(int i = 0; i < recordedStatesHistory.size(); i++){ // loop through each state
            recordedPositions.add(recordedStatesHistory.get(i).getPosition()); // and add to the recordedPositions list just the pose2d of that state
        }

        return recordedPositions;
    }
    public ArrayList<RobotState> getReplayStates(){return replayStates;}
    public ArrayList<Pose2d> getReplayPositions(){ // returns the replay states except just the positions from each state, allows for easy plotting in dashboard for example
        ArrayList<Pose2d> replayPositions = new ArrayList<Pose2d>();

        for(int i = 0; i < replayStates.size(); i++){ // loop through each state
            replayPositions.add(replayStates.get(i).getPosition()); // and add to the replayPositions list just the pose2d of that state
        }

        return replayPositions;
    }
    public double getTimerMsec(){
        return replayTimer.milliseconds();
    }

    public boolean setStatesFile(String fileName){
        boolean couldLoadFile = false;

        File root = new File(Environment.getExternalStorageDirectory(), STORAGE_DIRECTORY);
        if(!root.exists()){
            root.mkdirs();
        }

        statesFile = new File(root, fileName);

        try{
            couldLoadFile = !statesFile.createNewFile(); // if unable to create a new file at location, could load the file (the create new file method returns true if could create a new file)
        } catch (IOException e){ e.printStackTrace(); }

        return couldLoadFile;
    }
    private void addReplayStatesFrom(ConcurrentArrayList<RobotState> newStates, double allStatesAfterTimestamp){ // adds states into the main list from an outside list (used for loading states from the loading thread)
        for(int i = 0; i < newStates.size(); i++){
            if(newStates.get(i).getTimestamp() > allStatesAfterTimestamp){
                replayStates.add(newStates.get(i));
            }
        }
    }
    private void addReplayStatesFrom(ConcurrentArrayList<RobotState> newStates){
        addReplayStatesFrom(newStates, 0); // if no time provided, just add all valid states
    }


    class LoadManager implements Runnable { // a threaded subclass of ReplayManager that manages loading new states into the program
        Thread thread;
        String threadName;
        Scanner stateReader;
        ConcurrentArrayList<RobotState> stateBuffer;
        //int loadCount = 0;

        public LoadManager() {
            threadName = "Replay-Recorder Load Manager";
        }

        public void loadStates(Scanner stateReader, ConcurrentArrayList<RobotState> stateBuffer){
            thread = new Thread(this, threadName);
            thread.start(); // automatically calls the run method in a separate thread

            //this.loadCount = loadCount;
            this.stateReader = stateReader;
            this.stateBuffer = stateBuffer;
        }

        public void run() {
            try {

                load(); // attempt to load. if it fails, print the stack trace

            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        private void load () throws IOException {
            String currentLine = "";
            for(int i = 0; ( i < MAX_LOADED_STATES && stateReader.hasNextLine() ); i++){ // load as many as we are told to (within what we are allowed to do

                currentLine = stateReader.nextLine();


                if(currentLine != null && currentLine != "")
                    stateBuffer.add( RobotState.parseFromCSVLine(currentLine) );
            }
        }
    }
}
