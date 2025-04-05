package Auto;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import Systems.Claw;
import Systems.Lift;
import Vision.Vision;

import Hardware.hardwarePapiu;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Basket", group = "Auto")
public class AutoBasket extends OpMode{
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
    hardwarePapiu robot = new hardwarePapiu(this);

    private final Pose startPose = new Pose(8, 111, Math.toRadians(180));  // Starting position
    private final Pose scorePose = new Pose(13, 130, Math.toRadians(135)); // Scoring position
    private final Pose humanPose = new Pose(13, 24, Math.toRadians(180)); // Scoring position

    private Path scorePreload, park;
    private Pose location,target;
    private PathChain ParkSpec6;
    private PathChain positionLine1,linePickup1, linePickup2, linePickup3, humanLeave1, humanLeave2, humanLeave3, scorePickup1, scorePickup2, scorePickup3, scoreSub1,scoreSub2,scoreSub3,scoreSub4,Sub1,Sub2,Sub3,Sub4;
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        linePickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(13.000, 130.000, Point.CARTESIAN),
                                new Point(13.000, 130.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(160))
                .build();


        linePickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(13.000, 130.000, Point.CARTESIAN),
                                new Point(15.000, 130.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(180))
                .build();

        linePickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(13.000, 130.000, Point.CARTESIAN),
                                new Point(20.000, 120.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(225))
                .build();
    }

    public void autonomousPathUpdate(){
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    //Comenzi lasa preload
                    follower.followPath(linePickup1);
                    setPathState(2);
                }
                break;

            case 2:
                if(!follower.isBusy()) {
                    scorePickup1 = follower.pathBuilder()
                            .addPath(new BezierLine(
                                            follower.getPose(),
                                            scorePose
                                    )
                            )
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), scorePose.getHeading())
                            .build();
                    //Comenzi pentru luat line 1

                    follower.followPath(scorePickup1);
                    setPathState(3);
                }
                break;


            case 3:
                if(!follower.isBusy()) {
                    //Comenzi pentru lasat line 1

                    follower.followPath(linePickup2,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    scorePickup2 = follower.pathBuilder()
                            .addPath(new BezierLine(
                                            follower.getPose(),
                                            scorePose
                                    )
                            )
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), scorePose.getHeading())
                            .build();
                    //Comenzi pentru luat line 2

                    follower.followPath(scorePickup2,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    //Comenzi pentru lasat line 2

                    follower.followPath(linePickup3,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    scorePickup3 = follower.pathBuilder()
                            .addPath(new BezierLine(
                                            follower.getPose(),
                                            scorePose
                                    )
                            )
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), scorePose.getHeading())
                            .build();
                    //Comenzi pentru luat line 3

                    follower.followPath(scorePickup3,true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    Sub1 = follower.pathBuilder()
                            .addPath(new BezierCurve(
                                            new Point(follower.getPose().getX(),follower.getPose().getY(),Point.CARTESIAN),
                                            new Point(67.700, 119.000, Point.CARTESIAN),
                                            new Point(65.000, 94.000, Point.CARTESIAN)
                                    )
                            )
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(90))
                            .build();
                    //Pregatestete pentru sub 1

                    follower.followPath(Sub1, true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    vision.find();
                    follower.followPath(vision.toTarget());
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    scoreSub1 = follower.pathBuilder()
                            .addPath(new BezierCurve(
                                        new Point(follower.getPose().getX(),follower.getPose().getY(),Point.CARTESIAN),
                                        new Point(67.700, 119.000, Point.CARTESIAN),
                                        new Point(13,130, Point.CARTESIAN)
                                    )
                            )
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(135))
                            .build();
                    //Comenzi pentru luat samp 1

                    follower.followPath(scoreSub1);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    Sub2 = follower.pathBuilder()
                            .addPath(new BezierCurve(
                                            new Point(follower.getPose().getX(),follower.getPose().getY(),Point.CARTESIAN),
                                            new Point(67.700, 119.000, Point.CARTESIAN),
                                            new Point(65.000, 94.000, Point.CARTESIAN)
                                    )
                            )
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(90))
                            .build();
                    //Comenzi pentru lasat samp 1

                    follower.followPath(Sub2,true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    vision.find();
                    follower.followPath(vision.toTarget());
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    scoreSub2 = follower.pathBuilder()
                            .addPath(new BezierCurve(
                                            new Point(follower.getPose().getX(),follower.getPose().getY(),Point.CARTESIAN),
                                            new Point(67.700, 119.000, Point.CARTESIAN),
                                            new Point(13,130, Point.CARTESIAN)
                                    )
                            )
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(135))
                            .build();
                    //Comenzi pentru luat samp 2

                    follower.followPath(scoreSub2);
                    setPathState(13);
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    Sub3 = follower.pathBuilder()
                            .addPath(new BezierCurve(
                                            new Point(follower.getPose().getX(),follower.getPose().getY(),Point.CARTESIAN),
                                            new Point(67.700, 119.000, Point.CARTESIAN),
                                            new Point(65.000, 94.000, Point.CARTESIAN)
                                    )
                            )
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(90))
                            .build();
                    //Comenzi pentru lasat samp 2

                    follower.followPath(Sub3,true);
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    vision.find();
                    follower.followPath(vision.toTarget());
                    setPathState(15);
                }
                break;
            case 15:
                if(!follower.isBusy()) {
                    scoreSub3 = follower.pathBuilder()
                            .addPath(new BezierCurve(
                                            new Point(follower.getPose().getX(),follower.getPose().getY(),Point.CARTESIAN),
                                            new Point(67.700, 119.000, Point.CARTESIAN),
                                            new Point(13,130, Point.CARTESIAN)
                                    )
                            )
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(135))
                            .build();
                    //Comenzi pentru luat samp 3

                    follower.followPath(scoreSub2);
                    setPathState(16);
                }
                break;
            case 16:
                if(!follower.isBusy()) {
                    Sub4 = follower.pathBuilder()
                            .addPath(new BezierCurve(
                                            new Point(follower.getPose().getX(),follower.getPose().getY(),Point.CARTESIAN),
                                            new Point(67.700, 119.000, Point.CARTESIAN),
                                            new Point(65.000, 94.000, Point.CARTESIAN)
                                    )
                            )
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(90))
                            .build();
                    //Comenzi pentru lasat samp 3

                    follower.followPath(Sub3,true);
                    setPathState(17);
                }
                break;
            case 17:
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
        robot.poseteleop=follower.getPose();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        vision = new Vision(hardwareMap,telemetry,unwanted,follower,3);
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
