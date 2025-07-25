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
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Vision.Vision;

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "RedBasket", group = "org/firstinspires/ftc/teamcode/Auto")
public class AutoBasketRed extends OpMode{
    /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
    */
    private Follower follower;
    private Vision vision;
    private int[] unwanted;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private Robot r;

    private final Pose startPose = new Pose(8, 111, Math.toRadians(0));  // Starting position
    private final Pose scorePose = new Pose(15, 130, Math.toRadians(315)); // Scoring position
    private final Pose humanPose = new Pose(13, 24, Math.toRadians(0)); // Scoring position

    private Path scorePreload, park;
    private Pose location,target;
    private PathChain ParkSpec6;
    private boolean ok=true;
    private PathChain subToScore,positionLine1,positionLine2,subPose,positionLine3,linePickup1, linePickup2, linePickup3, humanLeave1, humanLeave2, humanLeave3, scorePickup1, scorePickup2, scorePickup3, scoreSub1,scoreSub2,scoreSub3,scoreSub4,Sub1,Sub2,Sub3,Sub4;
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        linePickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(scorePose.getX(), scorePose.getY(), Point.CARTESIAN),
                                new Point(18, 134, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-25))
                .addPath(
                        new BezierLine(
                                new Point(18, 134, Point.CARTESIAN),
                                new Point(23, 134, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-25))
                .build();
        positionLine1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(15, 134, Point.CARTESIAN),
                                new Point(23, 134, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-25))
                .build();


        linePickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(scorePose.getX(), scorePose.getY(), Point.CARTESIAN),
                                new Point(20, 120, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(30))
                .addPath(
                        new BezierLine(
                                new Point(20, 120, Point.CARTESIAN),
                                new Point(26, 120, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(30))
                .build();
        positionLine2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(15, 134, Point.CARTESIAN),
                                new Point(23, 134, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        linePickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(scorePose.getX(), scorePose.getY(), Point.CARTESIAN),
                                new Point(30, 111, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(55))
                .addPath(
                        new BezierLine(
                                new Point(30, 111, Point.CARTESIAN),
                                new Point(30, 121, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(55))
                .build();
        positionLine3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(30, 111, Point.CARTESIAN),
                                new Point(30, 121, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(55))
                .build();
        subPose = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(scorePose.getX(), scorePose.getY(), Point.CARTESIAN),
                                new Point(65.000, 127.000, Point.CARTESIAN),
                                new Point(60, 96.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90))
                .build();
        subToScore = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                                new Point(65.000, 127.000, Point.CARTESIAN),
                                new Point(scorePose.getX(), scorePose.getY(), Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45))
                .build();

    }
    public void updatePaths(){
        subToScore = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                                new Point(65.000, 127.000, Point.CARTESIAN),
                                new Point(scorePose.getX(), scorePose.getY(), Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45))
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(ok) {
                    follower.followPath(scorePreload,true);
                    r.o.targetHigh();
                    actionTimer.resetTimer();
                    ok=false;
                }
                if(actionTimer.getElapsedTimeSeconds()>0.7){
                    setPathState(1);
                    ok=true;
                    break;
                }
            case 1:
                if(!follower.isBusy()) {
                    if(ok) {
                        actionTimer.resetTimer();
                        ok=false;
                    }
                    if(actionTimer.getElapsedTimeSeconds()<0.1)r.i.toHigh();
                    if(actionTimer.getElapsedTimeSeconds()>0.1 && actionTimer.getElapsedTimeSeconds()<0.3){
                        r.i.latch.setPosition(0.3);
                        r.o.outtake1.setPosition(0.62);
                        r.o.outtake2.setPosition(0.62);
                        r.o.rotate.setPosition(0.7);
                    }
                    else if(actionTimer.getElapsedTimeSeconds()>=0.6 && actionTimer.getElapsedTimeSeconds()<0.7){
                        r.o.claw.setPosition(0.5);
                    }
                    else if(actionTimer.getElapsedTimeSeconds()>=0.7 && actionTimer.getElapsedTimeSeconds()<0.8){
                        r.o.rotate.setPosition(0.37);
                        r.o.outtake1.setPosition(0.06);
                        r.o.outtake2.setPosition(0.06);
                        r.i.toHigh();

                    }
                    if(actionTimer.getElapsedTimeSeconds()>0.8){
                        follower.followPath(linePickup1,true);
                        r.i.ground();
                        r.i.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                        r.i.intake.setPower(0.7);
                        r.TransferSamp();
                        setPathState(2);
                        ok=true;
                        break;
                    }
                }
                break;

            case 2:
                if(!follower.isBusy()) {
                    if(ok) {
                        scorePickup1 = follower.pathBuilder()
                                .addPath(new BezierLine(
                                                follower.getPose(),
                                                scorePose
                                        )
                                )
                                .setLinearHeadingInterpolation(follower.getPose().getHeading(), scorePose.getHeading())
                                .build();
                        actionTimer.resetTimer();
                        ok=false;
                    }
                    if(actionTimer.getElapsedTimeSeconds()<0.3){
                        r.i.toDownAuto();
                    }
                    if(actionTimer.getElapsedTimeSeconds()>=0.4){
                        r.HighBucket();
                        follower.followPath(scorePickup1,true);
                        setPathState(3);
                        ok=true;
                        break;
                    }
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    if(ok) {
                        actionTimer.resetTimer();
                        ok=false;
                    }
                    if(actionTimer.getElapsedTimeSeconds()>0.9&& actionTimer.getElapsedTimeSeconds()<1){
                        r.o.claw.setPosition(0.5);
                        r.i.toHigh();
                    }
                    else if (actionTimer.getElapsedTimeSeconds()>1){
                        r.i.ground();
                        r.i.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                        r.i.intake.setPower(0.7);
                        follower.followPath(linePickup2,true);
                        r.TransferSamp();
                        setPathState(4);
                        ok=true;
                        break;
                    }
                }
                break;


            case 4:
                if(!follower.isBusy()) {
                    if(ok) {
                        scorePickup2 = follower.pathBuilder()
                                .addPath(new BezierLine(
                                                follower.getPose(),
                                                scorePose
                                        )
                                )
                                .setLinearHeadingInterpolation(follower.getPose().getHeading(), scorePose.getHeading())
                                .build();
                        actionTimer.resetTimer();
                        ok=false;
                    }
                    if(actionTimer.getElapsedTimeSeconds()<0.3)r.i.toDown();
                    if(actionTimer.getElapsedTimeSeconds()>=0.4){
                        r.HighBucket();
                        follower.followPath(scorePickup2,true);
                        setPathState(5);
                        ok=true;
                        break;
                    }
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    if(ok) {
                        actionTimer.resetTimer();
                        ok=false;
                    }
                    if(actionTimer.getElapsedTimeSeconds()>0.9&& actionTimer.getElapsedTimeSeconds()<1){
                        r.i.toHigh();
                        r.o.claw.setPosition(0.5);
                    }
                    else if (actionTimer.getElapsedTimeSeconds()>1){
                        r.i.ground();
                        r.i.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                        r.i.intake.setPower(0.7);
                        follower.followPath(linePickup3,true);
                        r.TransferSamp();
                        setPathState(6);
                        ok=true;
                        break;
                    }
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    if(ok) {
                        scorePickup3 = follower.pathBuilder()
                                .addPath(new BezierLine(
                                                follower.getPose(),
                                                scorePose
                                        )
                                )
                                .setLinearHeadingInterpolation(follower.getPose().getHeading(), scorePose.getHeading())
                                .build();
                        actionTimer.resetTimer();
                        ok=false;
                    }
                    if(actionTimer.getElapsedTimeSeconds()<0.3)r.i.toDown();
                    if(actionTimer.getElapsedTimeSeconds()>=0.4){
                        r.HighBucket();
                        follower.followPath(scorePickup3,true);
                        setPathState(7);
                        ok=true;
                        break;
                    }
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    if(ok) {
                        actionTimer.resetTimer();
                        ok=false;
                    }
                    if(actionTimer.getElapsedTimeSeconds()>0.7&& actionTimer.getElapsedTimeSeconds()<0.8){
                        r.o.claw.setPosition(0.5);
                    }
                    else if (actionTimer.getElapsedTimeSeconds()>0.8){
                        r.i.intake.setPower(0);
                        follower.followPath(subPose,true);
                        r.TransferSamp();
                        setPathState(8);
                        ok=true;
                        break;
                    }
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    if(ok) {
                        vision.find();
                        actionTimer.resetTimer();
                        ok=false;
                    }
                    if(actionTimer.getElapsedTimeSeconds()>=1){
                        follower.followPath(vision.toTarget(),true);
                        setPathState(9);
                        ok=true;
                        break;
                    }
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    if(ok) {
                        updatePaths();
                        actionTimer.resetTimer();
                        ok=false;
                    }
                    if(actionTimer.getElapsedTimeSeconds()<0.1){
                        r.i.transfer();
                        r.i.setTarget(vision.positie()-50);
                    }
                    if(actionTimer.getElapsedTimeSeconds()>0.1 && actionTimer.getElapsedTimeSeconds()<0.2){
                        r.i.ground();
                    }
                    if(actionTimer.getElapsedTimeSeconds()>0.4 && actionTimer.getElapsedTimeSeconds()<0.5){
                        r.i.setTarget(vision.positie());
                    }
                    if(actionTimer.getElapsedTimeSeconds()>0.5 && actionTimer.getElapsedTimeSeconds()<0.6){
                        r.i.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                        r.i.intake.setPower(0.7);
                        r.i.latch.setPosition(0.3);
                    }
                    if(actionTimer.getElapsedTimeSeconds()>0.9 && actionTimer.getElapsedTimeSeconds()<1){
                        r.i.latch.setPosition(0.6);
                        r.i.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                        r.i.intake.setPower(0.38);
                    }
                    if(actionTimer.getElapsedTimeSeconds()>1 && actionTimer.getElapsedTimeSeconds()<1.1){
                        r.i.toDown();
                    }
                    if(actionTimer.getElapsedTimeSeconds()>1.1 && actionTimer.getElapsedTimeSeconds()<1.2) {
                        r.i.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                        r.i.intake.setPower(0.7);
                        r.i.latch.setPosition(0.3);
                    }
                    if(actionTimer.getElapsedTimeSeconds()>1.3){
                        r.HighBucket();
                        follower.followPath(subToScore,true);
                        setPathState(10);
                        ok=true;
                        break;
                    }
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    if(ok) {
                        actionTimer.resetTimer();
                        ok=false;
                    }
                    if(actionTimer.getElapsedTimeSeconds()>0.3&& actionTimer.getElapsedTimeSeconds()<0.5){
                        r.TransferSamp();
                    }
                    else if (actionTimer.getElapsedTimeSeconds()>0.5){
                        r.i.intake.setPower(0);
                        follower.followPath(subPose,true);
                        setPathState(11);
                        ok=true;
                        break;
                    }
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    if(ok) {
                        vision.find();
                        actionTimer.resetTimer();
                        ok=false;
                    }
                    if(actionTimer.getElapsedTimeSeconds()>=1){
                        follower.followPath(vision.toTarget(),true);
                        setPathState(12);
                        ok=true;
                        break;
                    }
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    if(ok) {
                        updatePaths();
                        actionTimer.resetTimer();
                        ok=false;
                    }
                    if(actionTimer.getElapsedTimeSeconds()<0.1){
                        r.i.transfer();
                        r.i.setTarget(vision.positie()-50);
                    }
                    if(actionTimer.getElapsedTimeSeconds()>0.1 && actionTimer.getElapsedTimeSeconds()<0.2){
                        r.i.ground();
                    }
                    if(actionTimer.getElapsedTimeSeconds()>0.4 && actionTimer.getElapsedTimeSeconds()<0.5){
                        r.i.setTarget(vision.positie());
                    }
                    if(actionTimer.getElapsedTimeSeconds()>0.5 && actionTimer.getElapsedTimeSeconds()<0.6){
                        r.i.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                        r.i.intake.setPower(0.7);
                        r.i.latch.setPosition(0.3);
                    }
                    if(actionTimer.getElapsedTimeSeconds()>0.9 && actionTimer.getElapsedTimeSeconds()<1){
                        r.i.latch.setPosition(0.6);
                        r.i.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                        r.i.intake.setPower(0.38);
                    }
                    if(actionTimer.getElapsedTimeSeconds()>1 && actionTimer.getElapsedTimeSeconds()<1.1){
                        r.i.toDown();
                    }
                    if(actionTimer.getElapsedTimeSeconds()>1.1 && actionTimer.getElapsedTimeSeconds()<1.2) {
                        r.i.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                        r.i.intake.setPower(0.7);
                        r.i.latch.setPosition(0.3);
                    }
                    if(actionTimer.getElapsedTimeSeconds()>1.3){
                        r.HighBucket();
                        follower.followPath(subToScore,true);
                        setPathState(13);
                        ok=true;
                        break;
                    }
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    if(ok) {
                        actionTimer.resetTimer();
                        ok=false;
                    }
                    if(actionTimer.getElapsedTimeSeconds()>0.3&& actionTimer.getElapsedTimeSeconds()<0.5){
                        r.TransferSamp();
                    }
                    else if (actionTimer.getElapsedTimeSeconds()>0.5){
                        r.i.intake.setPower(0);
                        follower.followPath(subPose,true);
                        setPathState(14);
                        ok=true;
                        break;
                    }
                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    if(ok) {
                        vision.find();
                        actionTimer.resetTimer();
                        ok=false;
                    }
                    if(actionTimer.getElapsedTimeSeconds()>=1){
                        follower.followPath(vision.toTarget(),true);
                        setPathState(15);
                        ok=true;
                        break;
                    }
                }
                break;
            case 15:
                if(!follower.isBusy()) {
                    if(ok) {
                        updatePaths();
                        actionTimer.resetTimer();
                        ok=false;
                    }
                    if(actionTimer.getElapsedTimeSeconds()<0.1){
                        r.i.transfer();
                        r.i.setTarget(vision.positie()-50);
                    }
                    if(actionTimer.getElapsedTimeSeconds()>0.1 && actionTimer.getElapsedTimeSeconds()<0.2){
                        r.i.ground();
                    }
                    if(actionTimer.getElapsedTimeSeconds()>0.4 && actionTimer.getElapsedTimeSeconds()<0.5){
                        r.i.setTarget(vision.positie());
                    }
                    if(actionTimer.getElapsedTimeSeconds()>0.5 && actionTimer.getElapsedTimeSeconds()<0.6){
                        r.i.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                        r.i.intake.setPower(0.7);
                        r.i.latch.setPosition(0.3);
                    }
                    if(actionTimer.getElapsedTimeSeconds()>0.9 && actionTimer.getElapsedTimeSeconds()<1){
                        r.i.latch.setPosition(0.6);
                        r.i.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                        r.i.intake.setPower(0.38);
                    }
                    if(actionTimer.getElapsedTimeSeconds()>1 && actionTimer.getElapsedTimeSeconds()<1.1){
                        r.i.toDown();
                    }
                    if(actionTimer.getElapsedTimeSeconds()>1.1 && actionTimer.getElapsedTimeSeconds()<1.2) {
                        r.i.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                        r.i.intake.setPower(0.7);
                        r.i.latch.setPosition(0.3);
                    }
                    if(actionTimer.getElapsedTimeSeconds()>1.3){
                        r.HighBucket();
                        follower.followPath(subToScore,true);
                        setPathState(16);
                        ok=true;
                        break;
                    }
                }
                break;
            case 16:
                if(!follower.isBusy()) {
                    if(ok) {
                        actionTimer.resetTimer();
                        ok=false;
                    }
                    if(actionTimer.getElapsedTimeSeconds()>0.3&& actionTimer.getElapsedTimeSeconds()<0.5){
                        r.TransferSamp();
                    }
                    else if (actionTimer.getElapsedTimeSeconds()>0.5){
                        r.i.intake.setPower(0);
                        follower.followPath(subPose,true);
                        setPathState(16);
                        ok=true;
                        break;
                    }
                }
                break;
            case 17:
                if(!follower.isBusy()) {
                    if(ok) {
                        vision.find();
                        actionTimer.resetTimer();
                        ok=false;
                    }
                    if(actionTimer.getElapsedTimeSeconds()>=1){
                        follower.followPath(vision.toTarget(),true);
                        setPathState(17);
                        ok=true;
                        break;
                    }
                }
                break;
            case 18:
                if(!follower.isBusy()) {
                    if(ok) {
                        updatePaths();
                        actionTimer.resetTimer();
                        ok=false;
                    }
                    if(actionTimer.getElapsedTimeSeconds()<0.1){
                        r.i.transfer();
                        r.i.toHighish();
                    }
                    if(actionTimer.getElapsedTimeSeconds()>0.3 && actionTimer.getElapsedTimeSeconds()<0.4){
                        r.i.ground();
                    }
                    if(actionTimer.getElapsedTimeSeconds()>0.4 && actionTimer.getElapsedTimeSeconds()<0.5){
                        r.i.toHigh();
                    }
                    if(actionTimer.getElapsedTimeSeconds()>0.5 && actionTimer.getElapsedTimeSeconds()<0.6){
                        r.i.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                        r.i.intake.setPower(0.7);
                        r.i.latch.setPosition(0.3);
                    }
                    if(actionTimer.getElapsedTimeSeconds()>1 && actionTimer.getElapsedTimeSeconds()<1.1){
                        r.i.toDown();
                    }
                    if(actionTimer.getElapsedTimeSeconds()>1.3){
                        r.HighBucket();
                        follower.followPath(subToScore,true);
                        setPathState(18);
                        ok=true;
                        break;
                    }
                }
                break;
            case 19:
                if(!follower.isBusy()) {
                    if(ok) {
                        vision.off();
                        actionTimer.resetTimer();
                        ok=false;
                    }
                    if(actionTimer.getElapsedTimeSeconds()>0.3&& actionTimer.getElapsedTimeSeconds()<0.5){
                        r.TransferSamp();
                    }
                    else if (actionTimer.getElapsedTimeSeconds()>0.5){
                        r.i.intake.setPower(0);
                        setPathState(-1);
                        ok=true;
                        break;
                    }
                }
                break;
            case 20:
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
        vision = new Vision(hardwareMap,telemetry,"abc","blue",follower,1);
        vision.on();
        r.aInit();
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
    }
}
