package org.firstinspires.ftc.teamcode.util.ActionReplay;

import android.os.Environment;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Scanner;


@Config
public class ReplayManager {
    // Constants
    public static int MAX_LOADED_STATES = 20_000; // the largest the manager will let the states lists get, should hold around 70 seconds of states
    public static double PATH_REFRESH_TRHESHOLD = 0.05; // when 5% of the loaded path has been traversed, the robot unloads the PATH_UNLOAD_PERCENTAGE of the path that is behind it and reloads that much to the front of the path
    public static double PATH_UNLOAD_PERCENTAGE = 0.10; // how much of the path behind us unloads when the path refresh threshold is reached
    public static double LOOK_AHEAD_MSEC = 0.0; // how many milliseconds ahead on the path the robot will try to go to, to prevent lagging behind the timestamp
    public static final String STORAGE_DIRECTORY = "ReplaySaves";

    // Recording Objects
    private ArrayList<RobotState> recordedStatesHistory ; // a list of recorded states, used for drawing out where we have been while recording
    private ArrayList<RobotState> replayStates; // a list of states that is followed, loaded ahead of where we are
    private ArrayList<RobotState> replayLoadedStates; // temporarily holds the states before they get added into the list
    private File statesFile;
    //private BufferedReader stateReader;
    private Scanner stateReader;
    private FileWriter stateWriter;
    private ElapsedTime replayTimer;
    private LoadManager loader; // the threaded object that loads states

    // Flags
    private boolean recording = false;
    private boolean replaying = false;


   // Telemetry telem;


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
            loadReplayFile(fileName);
        }

        loader = new LoadManager();
        replayTimer = new ElapsedTime();
    }


    public boolean startRecording() {
        stopRecording(); // stop any recording and any previous replaying just to make sure everything is cleared
        stopStateReplay();
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
        else {
            return false;
        }
    }
    public boolean recordRobotState(RobotState currentState) {
        if(recording){// only want to do these things if we are recording, if not this could cause errors as objects may not have been yet setup properly
            try {
                stateWriter.write(currentState.toCSVLine() + "\n"); // save the current state
            }
            catch (IOException e) { return false; }

            recordedStatesHistory.add(currentState); // add the current state to the states history in memory (doesn't impact the file)

            while(recordedStatesHistory.size() > MAX_LOADED_STATES){ // then remove from the earliest point in the state history until we have a list less than the max list size
                recordedStatesHistory.remove(0); // as a note, this only impacts what is held in memory for telemetry purposes, the states stored in the file aren't impacted by this removal
            }
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

        //previousRecordedStates = recordedStatesHistory;
        recording = false;

        return true; // return true if successful
    }

    public static int linesToLoad = -1;
    public boolean startStateReplay() {
        stopRecording(); // stop any recording and any previous replaying just to make sure everything is cleared
        stopStateReplay();
        replayStates = new ArrayList<RobotState>();
        replayLoadedStates = new ArrayList<RobotState>();

       // replayStates.add(new RobotState(0, new Pose2d(0, 0, 0), new GamepadState(), new GamepadState()));
       // replayStates.add(new RobotState(10000, new Pose2d(30, 0, 0), new GamepadState(), new GamepadState()));


        if(statesFile != null){
            try {
                stateReader = new Scanner(statesFile); // attempt to make a buffered file reader to read out the contents of the file, if it fails, return false
            }
            catch(IOException e){
                return false;
            }

            replaying = true; // set replaying to true so that loadStates will load states (as it checks if replaying is true before loading states)
            loader.loadStates(stateReader, replayLoadedStates); // load as many states as we can into our replayStates list for following, runs in a background thread constantly
            //replayStates = previousRecordedStates;


            replayTimer.reset();

            return true;
        }
        else {
            return false;
        }
    }
    public RobotState getCurrentTargetState(){
        //telem.addLine("Has a file open? " + (statesFile != null));
        if(replaying){
            if(replayStates.size() > 0)
                addReplayStatesFrom(replayLoadedStates, replayStates.get(replayStates.size() - 1).getTimestamp());
            else
                addReplayStatesFrom(replayLoadedStates);
        }

        if(replaying && replayStates.size() > 1){
            int manipulatorStateEndIndex = 1; // same as below but for everything except for driving
            int driveTimeChunkEndIndex = 1; // the index of the first state that our drive time is after or equal to, aka the beginning of the current time chunk we are driving in
            double currentTime = replayTimer.milliseconds();
            double effectiveDriveTime = currentTime + LOOK_AHEAD_MSEC;

            // having separate states for each of
            while(manipulatorStateEndIndex < replayStates.size() - 1 && replayStates.get(manipulatorStateEndIndex).getTimestamp() < currentTime ){ // move the timeChunkStart index forward until we reach a state with a timestamp that is greater than our current timestamp
                manipulatorStateEndIndex++;
            } // once we reach the state with a timestamp that is greater, we have found the end of our time chunk
            GamepadState gamepad1State = replayStates.get(manipulatorStateEndIndex - 1).getGamepad1State();
            GamepadState gamepad2State = replayStates.get(manipulatorStateEndIndex - 1).getGamepad2State();

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

                //loader.loadStates(distanceFromListFront, stateReader, replayStates); // then load that many onto the front of the list
            }

            // since we have removed all states before this chunk, the start and end of this chunk are always indexes 0 and 1. now we just get where we are between them
            return getStateBetween(firstDriveState, secondDriveState, effectiveDriveTime, gamepad1State, gamepad2State);
        }
        else {
            return new RobotState();
        }
    }
    public void stopStateReplay(){
        if(replaying){ // only want to close the reader if we were replaying, if not this could cause errors
            //try {
                stateReader.close();
            //} catch (IOException e) {
            //    e.printStackTrace();
            //}
        }

        replaying = false;
    }


    private RobotState getStateBetween(RobotState firstState, RobotState secondState, double effectiveDriveTime, GamepadState gamepad1State, GamepadState gamepad2State){
        if( effectiveDriveTime < firstState.getTimestamp() || firstState.getTimestamp() == secondState.getTimestamp()){ // if we are before the timestamp of the first state, just return that state
            return firstState;
        }
        else if( effectiveDriveTime > secondState.getTimestamp() ){ // if we are after the timestamp of the second state, just return that state
            return secondState;
        }


        double firstTimestamp = firstState.getTimestamp();
        double secondTimestamp = secondState.getTimestamp();

        double fractionBetween = (effectiveDriveTime - firstTimestamp)/(secondTimestamp - firstTimestamp); // shift everything such that the first timestamp is 0, then see what the current time is out of the second timestamp
            // for example: the first timestamp = 2, second = 6, current = 3.  3-2 =1, 6-2 =4, we are currently 1/4 of the waybetween 2 and 6

        Pose2d firstPose = firstState.getPosition();
        Pose2d secondPose = secondState.getPosition();

        double x = ((secondPose.getX() - firstPose.getX()) * fractionBetween) + firstPose.getX(); // shift the "line" to the origin, as though firstPose were the base, then multiply by the fraction between, then shift back
        double y = ((secondPose.getY() - firstPose.getY()) * fractionBetween) + firstPose.getY();
        double heading = ((secondPose.getHeading() - firstPose.getHeading()) * fractionBetween) + firstPose.getHeading();


        if(firstState.hasGamepadStates()){
            return new RobotState(effectiveDriveTime, new Pose2d(x, y, heading), gamepad1State, gamepad2State);
        }
        else {
            return new RobotState( effectiveDriveTime, new Pose2d(x, y, heading) );
        }
    }
    private RobotState getStateBetween(RobotState firstState, RobotState secondState, double effectiveDriveTime){
        return getStateBetween(firstState, secondState, effectiveDriveTime, firstState.getGamepad1State(), firstState.getGamepad2State());
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

    public boolean loadReplayFile(String fileName){
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
    private void addReplayStatesFrom(ArrayList<RobotState> newStates, double allStatesAfterTimestamp){ // adds states into the main list from an outside list (used for loading states from the loading thread)
        for(int i = 0; i < newStates.size(); i++){
            if(newStates.get(i).getTimestamp() > allStatesAfterTimestamp){
                replayStates.add(newStates.get(i));
            }
        }
    }
    private void addReplayStatesFrom(ArrayList<RobotState> newStates){
        addReplayStatesFrom(newStates, 0); // if no time provided, just add all valid states
    }


    class LoadManager implements Runnable { // a threaded subclass of ReplayManager that manages loading new states into the program
        Thread thread;
        String threadName;
        Scanner stateReader;
        ArrayList<RobotState> stateBuffer;
        //int loadCount = 0;

        public LoadManager() {
            threadName = "Replay-Recorder Load Manager";
        }

        public void loadStates(Scanner stateReader, ArrayList<RobotState> stateBuffer){
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
            //telem.addData("Has lines to read?", stateReader.hasNextLine());
            String currentLine = "";
            for(int i = 0; ( i < MAX_LOADED_STATES && stateReader.hasNextLine() ); i++){ // load as many as we are told to (within what we are allowed to do

                currentLine = stateReader.nextLine();


                //  telem.addLine("Current CSV Line: " + currentLine);
                //  telem.addLine("Parsed Line: " + RobotState.parseFromCSVLine(currentLine));

                if(currentLine != null && currentLine != "")
                    stateBuffer.add( RobotState.parseFromCSVLine(currentLine) );
            }
        }
    }
}
