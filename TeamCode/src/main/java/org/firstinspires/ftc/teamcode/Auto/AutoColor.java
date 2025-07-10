package org.firstinspires.ftc.teamcode.Auto;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Vision.Vision;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "Culoare", group = "org/firstinspires/ftc/teamcode/Auto")
public class AutoColor extends OpMode{
    /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
    */
    private Follower follower;
    private Robot r;
    private Vision vision;
    private int[] unwanted;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private boolean ok=true;

    private final Pose startPose = new Pose(7, 64, Math.toRadians(0));  // Starting position
    private final Pose scorePose = new Pose(37, 68, Math.toRadians(0)); // Scoring position
    private final Pose humanPose = new Pose(13, 24, Math.toRadians(0)); // Scoring position
    private int scoreY=77;
    private Pose location,target;
    private PathChain ParkSpec6;
    private PathChain positionLine1,linePickup1,scorePreload, linePickup2, linePickup3, humanLeave1, humanLeave2, humanLeave3, scorePickup1,HumanToScore,ScoreToHuman;
    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(8.000, 64.000, Point.CARTESIAN),
                                new Point(40.000, scoreY, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();


        linePickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(40.000, scoreY, Point.CARTESIAN),
                                new Point(30.000, 47.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-60))
                .build();

        humanLeave1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(30.000, 47.000, Point.CARTESIAN),
                                new Point(27, 45, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-60), Math.toRadians(-130))
                .build();

        linePickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(27, 45, Point.CARTESIAN),
                                new Point(30.000, 38.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-130), Math.toRadians(-60))
                .build();

        humanLeave2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(30.000, 38.000, Point.CARTESIAN),
                                new Point(28.000, 36.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-60), Math.toRadians(-140))
                .build();

        linePickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(28.000, 36.000, Point.CARTESIAN),
                                new Point(30.000, 29.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-140), Math.toRadians(-60))
                .build();

        humanLeave3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(30.000, 29.000, Point.CARTESIAN),
                                new Point(28.000, 27.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-60), Math.toRadians(-150))
                .build();
        scorePickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(28.000, 27.000, Point.CARTESIAN),
                                new Point(12.000, 35.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-150), Math.toRadians(0))
                .build();
        HumanToScore = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(12.000, 35.000, Point.CARTESIAN),
                                new Point(40.000, scoreY, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        ScoreToHuman = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(40.000, scoreY, Point.CARTESIAN),
                                new Point(12.000, 35.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }
    public void updatePaths(){
        HumanToScore = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(12.000, 35.000, Point.CARTESIAN),
                                new Point(40.000, scoreY, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        ScoreToHuman = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(40.000, scoreY, Point.CARTESIAN),
                                new Point(12.000, 35.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }
    public void autonomousPathUpdate(){
        switch (pathState) {
            case 0:
                if(ok) {
                    follower.followPath(scorePreload);
                    actionTimer.resetTimer();
                    ok=false;
                }
                if(actionTimer.getElapsedTimeSeconds()<0.3){
                    r.takeSpec();
                }
                else {
                    setPathState(1);
                    ok=true;
                    break;
                }
            case 1:
                if(!follower.isBusy()) {
                   /* vision.find();
                    follower.followPath(vision.toTarget()); */
                    r.leaveSpec();
                    follower.followPath(linePickup1,true);
                    scoreY=scoreY-2;
                    updatePaths();
                    setPathState(2);
                }
                break;

            case 2:
                if(!follower.isBusy()) {
                   /*ParkSpec6 = follower.pathBuilder()
                           .addPath(new BezierLine(
                                           follower.getPose(),
                                           new Pose(15, 30, follower.getPose().getHeading())
                                   )
                           )
                           .setConstantHeadingInterpolation(follower.getPose().getHeading())
                           .build();
                    //Comenzi pentru luat spec 6

                    follower.followPath(ParkSpec6); */
                    if(ok){
                        actionTimer.resetTimer();
                        ok=false;
                    }
                    if(actionTimer.getElapsedTimeSeconds()<0.1){
                        r.i.toHigh();
                    }
                    if(actionTimer.getElapsedTimeSeconds()>0.1 && actionTimer.getElapsedTimeSeconds()<0.3){
                        r.i.ground();
                        r.i.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                        r.i.intake.setPower(0.7);
                        r.i.latch.setPosition(0.3);
                    }
                    if(actionTimer.getElapsedTimeSeconds()>0.7) {
                        follower.followPath(humanLeave1, true);
                        setPathState(3);
                        ok=true;
                        r.i.transfer();
                    }
                }
                break;

            case 3:
                if(!follower.isBusy()) {
                    if(ok){
                        actionTimer.resetTimer();
                        ok=false;
                    }
                    if(actionTimer.getElapsedTimeSeconds()<0.1){
                        r.i.ground();
                        r.i.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                        r.i.intake.setPower(0.38);
                    }
                    if(actionTimer.getElapsedTimeSeconds()>0.4 && actionTimer.getElapsedTimeSeconds()<0.7){
                        r.i.transfer();
                        r.i.intake.setPower(0);
                        r.i.toLow();
                        follower.followPath(linePickup2,true);
                        setPathState(4);
                        ok=true;
                    }

                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(humanLeave2,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(linePickup3,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(humanLeave3,true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    follower.followPath(HumanToScore,true);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    follower.followPath(ScoreToHuman,true);
                    scoreY=scoreY-2;
                    updatePaths();
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()) {

                    follower.followPath(HumanToScore,true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    follower.followPath(ScoreToHuman,true);
                    scoreY=scoreY-2;
                    updatePaths();
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    follower.followPath(HumanToScore,true);
                    setPathState(13);
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    follower.followPath(ScoreToHuman,true);
                    scoreY=scoreY-2;
                    updatePaths();
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    follower.followPath(HumanToScore,true);
                    setPathState(15);
                }
                break;
            case 15:
                if(!follower.isBusy()) {
                    follower.followPath(ScoreToHuman,true);
                    scoreY=scoreY-2;
                    updatePaths();
                    setPathState(16);
                }
                break;
            case 16:
                if(!follower.isBusy()) {
                    follower.followPath(HumanToScore,true);
                    setPathState(17);
                }
                break;
            case 17:
                if(!follower.isBusy()) {
                    follower.followPath(ScoreToHuman,true);
                    scoreY=scoreY-2;
                    updatePaths();
                    setPathState(18);
                }
                break;
            case 18:
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();
        r.aPeriodic();

       // robot.poseteleop=follower.getPose();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Sample Position", "X: %.2f, Y: %.2f", vision.getTarget().getX(), vision.getTarget().getY());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        actionTimer.resetTimer();
        opmodeTimer.resetTimer();

        r = new Robot(hardwareMap, telemetry, gamepad1 , gamepad2,true,true);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        vision = new Vision(hardwareMap,telemetry,"blue","yellow",follower,1);
        r.aInit();
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        vision.on();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}
