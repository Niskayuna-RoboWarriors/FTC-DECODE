package Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "selfCorrectingShaepsAUTON", group = "Vision")
public class selfCorrectingShapesAUTON extends LinearOpMode {

    private Follower follower;

    private Timer pathTimer = new Timer();
    private Timer opModeTimer = new Timer();
    public enum PathState {
        // start position - end posisiton
        //drive - movement state
        //shoot - attempt to score
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD

    }

    PathState pathState;

    private final Pose startPose = new Pose(20.13157894736842,126.23684210526316, Math.toRadians(138));
    private final Pose shootPose = new Pose(27.236842105263158, 115.10526315789473, Math.toRadians(138));

    private PathChain driveStartPosToShootPos;

    public void buildPaths(){
        //put in coordnates for starting pose > ending pose

        driveStartPosToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch (pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosToShootPos, true);//true holds robot in final positiion
                setPathState(PathState.SHOOT_PRELOAD); //reset the timer and make new state
                break;
            case SHOOT_PRELOAD:
                //TODO add logic to flywheel shooter
                //check if follower done it's path?
                if (!follower.isBusy()){
                    //TODO add logic to flywheel shooter
                    //not busy
                    //finished path
                    telemetry.addLine("Done path 1");
                }
                //transition to next state
                break;
            default:
                telemetry.addLine("NO state connected");
                break;
        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
        }

    @Override
    public void runOpMode() throws InterruptedException {

        opModeTimer.resetTimer();
        setPathState(pathState);

        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        //opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        //TODO add in any other init mechanics

        buildPaths();
        follower.setPose(startPose);



        while (true){
            follower.update();
            statePathUpdate();

            telemetry.addData("path state: ", pathState.toString());
            telemetry.addData("x: ", follower.getPose().getX());
            telemetry.addData("y: ", follower.getPose().getY());
            telemetry.addData("heading: ", follower.getPose().getHeading());
            telemetry.addData("Path time: ", pathTimer.getElapsedTimeSeconds());
        }


    }




}


