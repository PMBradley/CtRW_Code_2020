package org.firstinspires.ftc.teamcode.util.ActionReplay;

import android.os.Environment;
import android.provider.ContactsContract;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.R;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Scanner;


@Config
public class ReplayManager {
    // Constants
    public static int MAX_LOADED_STATES = 690; // the largest the manager will let the states lists get, should hold around 70 seconds of states
    public static double LOOK_AHEAD_MSEC = 0.0; // how many milliseconds ahead on the path the robot will try to go to, to prevent lagging behind the timestamp
    public static final String STORAGE_DIRECTORY = "ReplaySaves";

    // Recording Objects
    private ArrayList<RobotState> recordedStatesHistory ; // a list of recorded states, used for drawing out where we have been while recording
    private ArrayList<RobotState> replayStates; // a list of states that is followed, loaded ahead of where we are
    private ArrayList<RobotState> previousRecordedStates = new ArrayList<RobotState>(); // TODO: remove this at some point
    private File statesFile;
    private Scanner stateReader;
    private FileWriter stateWriter;
    private ElapsedTime replayTimer;

    // Flags
    private boolean recording = false;
    private boolean replaying = false;


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
            File root = new File(Environment.getExternalStorageDirectory(), STORAGE_DIRECTORY);
            if(!root.exists()){
                root.mkdirs();
            }

            statesFile = new File(root, fileName);

            try{
                statesFile.createNewFile();
            } catch (IOException e){ e.printStackTrace(); }
        }

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

        previousRecordedStates = recordedStatesHistory;
        recording = false;

        return true; // return true if successful
    }


    public boolean startStateReplay() {
        stopRecording(); // stop any recording and any previous replaying just to make sure everything is cleared
        stopStateReplay();
        replayStates = new ArrayList<RobotState>();


        if(statesFile != null){
            try {
                stateReader = new Scanner(statesFile);
            }
            catch(IOException e){
                return false;
            }

            loadStates(MAX_LOADED_STATES); // load as many states as we can into our replayStates list for following
            replayStates = previousRecordedStates;

            replaying = true;
            replayTimer.reset();

            return true;
        }
        else {
            return false;
        }
    }
    public RobotState getCurrentTargetState(){
        if(replaying && replayStates.size() > 1){
            int timeChunkEndIndex = 1; // the index of the first state that our current time is after or equal to, aka the beginning of the current time chunk we are operating in
            double effectiveCurrentTime = replayTimer.milliseconds() + LOOK_AHEAD_MSEC;

            while(timeChunkEndIndex < replayStates.size() - 1 && replayStates.get(timeChunkEndIndex).getTimestamp() < effectiveCurrentTime ){ // move the timeChunkStart index forward until we reach a state with a timestamp that is greater than our current timestamp
                timeChunkEndIndex++;
            } // once we reach the state with a timestamp that is greater, we have found the end of our time chunk


            if(timeChunkEndIndex > 1){ // if we traversed forwards in the list at all, the current time chunk is forward in the list so to save space we want to unload any states behind the current chunk (and load in future ones to replacec them)
                int distanceFromListFront = timeChunkEndIndex - 1;

                for(int i = 0; i < distanceFromListFront; i++){ // remove from the font until the front is the current chunk
                    //replayStates.remove(0); TODO: add this back in if this isn't the problem, otherwise fix
                }

                loadStates(distanceFromListFront); // then load that many onto the front of the list
            }

            // since we have removed all states before this chunk, the start and end of this chunk are always indexes 0 and 1. now we just get where we are between them
            return getStateBetween(replayStates.get(timeChunkEndIndex - 1), replayStates.get(timeChunkEndIndex), effectiveCurrentTime);
        }
        else {
            return new RobotState();
        }
    }
    public void stopStateReplay(){
        if(replaying){ // only want to do these things if we were replaying, if not this could cause errors
            stateReader.close();
        }

        replaying = false;
    }


    private void loadStates(int loadCount){ // loads states from the current state file into the replayStates list
        if(replaying){
            for(int i = 0; i < loadCount && i < MAX_LOADED_STATES && stateReader.hasNextLine(); i++){ // load as many as we are told to (within what we are allowed to do
              //  String currentLine = stateReader.nextLine(); // TODO: please make load work

               // replayStates.add( RobotState.parseFromCSVLine(currentLine) );
            }
        }
    }
    private RobotState getStateBetween(RobotState firstState, RobotState secondState, double currentTime){
        if( currentTime < firstState.getTimestamp() || firstState.getTimestamp() == secondState.getTimestamp()){ // if we are before the timestamp of the first state, just return that state
            return firstState;
        }
        else if( currentTime > secondState.getTimestamp() ){ // if we are after the timestamp of the second state, just return that state
            return secondState;
        }


        double firstTimestamp = firstState.getTimestamp();
        double secondTimestamp = secondState.getTimestamp();

        double fractionBetween = (currentTime - firstTimestamp)/(secondTimestamp - firstTimestamp); // shift everything such that the first timestamp is 0, then see what the current time is out of the second timestamp
            // for example: the first timestamp = 2, second = 6, current = 3.  3-2 =1, 6-2 =4, we are currently 1/4 of the waybetween 2 and 6

        Pose2d firstPose = firstState.getPosition();
        Pose2d secondPose = secondState.getPosition();

        double x = ((secondPose.getX() - firstPose.getX()) * fractionBetween) + firstPose.getX(); // shift the "line" to the origin, as though firstPose were the base, then multiply by the fraction between, then shift back
        double y = ((secondPose.getY() - firstPose.getY()) * fractionBetween) + firstPose.getY();
        double heading = ((secondPose.getHeading() - firstPose.getHeading()) * fractionBetween) + firstPose.getHeading();


        if(firstState.hasGamepadStates()){
            return new RobotState(currentTime, new Pose2d(x, y, heading), firstState.getGamepad1State(), secondState.getGamepad2State());
        }
        else {
            return new RobotState( currentTime, new Pose2d(x, y, heading) );
        }
    }


    // a bunch of accessor methods so that other classes can get information about this class
    public boolean isRecording(){return recording;}
    public boolean isReplaying(){return replaying;}
    public ArrayList<RobotState> getRecordedStatesHistory(){return recordedStatesHistory;}
    public ArrayList<Pose2d> getRecordedPositionsHistory(){ // returns the states history except just the positions from each state, allows for easy plotting in dashboard for example
        ArrayList<Pose2d> recordedPositions = new ArrayList<Pose2d>();

        for(RobotState state : recordedStatesHistory){ // loop through each state
            recordedPositions.add(state.getPosition()); // and add to the recordedPositions list just the pose2d of that state
        }

        return recordedPositions;
    }
    public ArrayList<RobotState> getReplayStates(){return replayStates;}
    public ArrayList<Pose2d> getReplayPositions(){ // returns the replay states except just the positions from each state, allows for easy plotting in dashboard for example
        ArrayList<Pose2d> replayPositions = new ArrayList<Pose2d>();

        for(RobotState state : replayStates){ // loop through each state
            replayPositions.add(state.getPosition()); // and add to the replayPositions list just the pose2d of that state
        }

        return replayPositions;
    }
    public double getTimerMsec(){
        return replayTimer.milliseconds();
    }
}
