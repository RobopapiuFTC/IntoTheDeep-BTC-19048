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

@Autonomous(name = "Culoare", group = "Auto")
public class AutoColor extends OpMode{
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

    private final Pose startPose = new Pose(7, 64, Math.toRadians(0));  // Starting position
    private final Pose scorePose = new Pose(40, 68, Math.toRadians(0)); // Scoring position
    private final Pose humanPose = new Pose(13, 24, Math.toRadians(0)); // Scoring position

    private Path scorePreload, park;
    private Pose location,target;
    private PathChain ParkSpec6;
    private PathChain positionLine1,linePickup1, linePickup2, linePickup3, humanLeave1, humanLeave2, humanLeave3, scorePickup1, scorePickup2, scorePickup3, scorePickup4, humanPickup1, humanPickup2, humanPickup3, humanPickup4;
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        linePickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                                new Point(15, 30, Point.CARTESIAN),
                                new Point(62, 44.5, Point.CARTESIAN),
                                new Point(58.000, 24.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        humanLeave1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(58.000, 24.000, Point.CARTESIAN),
                                new Point(16.500, 24.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        linePickup2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(16.500, 24.000, Point.CARTESIAN),
                                new Point(64.097, 35.957, Point.CARTESIAN),
                                new Point(57.700, 14.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        humanLeave2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(57.700, 14.000, Point.CARTESIAN),
                                new Point(16.500, 14.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
        linePickup3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(16.500, 14.000, Point.CARTESIAN),
                                new Point(54.022, 18.239, Point.CARTESIAN),
                                new Point(57.000, 8.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        humanLeave3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(57.000, 8.000, Point.CARTESIAN),
                                new Point(17.000, 8.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
        humanPickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(17.000, 8.000, Point.CARTESIAN),
                                new Point(13.000, 24.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(13.000, 24.000, Point.CARTESIAN),
                                new Point(40.000, 68.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        humanPickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(40.000, 68.000, Point.CARTESIAN),
                                new Point(13.000, 24.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(13.000, 24.000, Point.CARTESIAN),
                                new Point(40.000, 68.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        humanPickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(40.000, 68.000, Point.CARTESIAN),
                                new Point(13.000, 24.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(13.000, 24.000, Point.CARTESIAN),
                                new Point(40.000, 68.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        humanPickup4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(40.000, 68.000, Point.CARTESIAN),
                                new Point(13.000, 24.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        scorePickup4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(13.000, 24.000, Point.CARTESIAN),
                                new Point(40.000, 68.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        park = new Path( new BezierLine(
                new Point(40.000, 68.000, Point.CARTESIAN),
                new Point(13.000, 24.000, Point.CARTESIAN)
        ));
        park.setConstantHeadingInterpolation(Math.toRadians(0));
    }

    public void autonomousPathUpdate() throws InterruptedException{
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                //comenzi pentru preload aka glisiera si cleste si alea
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    //Comenzi pentru preload adica lasa cleste ala in mm
                    vision.find();
                    Thread.sleep(1000);
                    follower.followPath(vision.toTarget());
                    setPathState(2);
                }
                break;

            case 2:
                if(!follower.isBusy()) {
                   ParkSpec6 = follower.pathBuilder()
                           .addPath(new BezierLine(
                                           follower.getPose(),
                                           new Pose(15, 30, follower.getPose().getHeading())
                                   )
                           )
                           .setConstantHeadingInterpolation(follower.getPose().getHeading())
                           .build();
                    //Comenzi pentru luat spec 6

                    follower.followPath(ParkSpec6);
                    setPathState(3);
                }
                break;


            case 3:
                if(!follower.isBusy()) {
                    //Comenzi pentru lasat spec 6
                    follower.followPath(linePickup1,true);
                    if(follower.getPose().getY() < 50) {
                    }
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(humanLeave1,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(linePickup2,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(humanLeave2,true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(linePickup3, true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    follower.followPath(humanLeave3,true);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    follower.followPath(humanPickup1,true);
                    if(follower.getPose().getY() < 20) {
                    }
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup1,true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    follower.followPath(humanPickup2,true);
                    if(follower.getPose().getY() < 60) {
                    }
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup2,true);
                    setPathState(13);
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    follower.followPath(humanPickup3,true);
                    if(follower.getPose().getY() < 60) {
                    }
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup3,true);
                    setPathState(15);
                }
                break;
            case 15:
                if(!follower.isBusy()) {
                    follower.followPath(humanPickup4,true);
                    if(follower.getPose().getY() < 60) {
                    }
                    setPathState(16);
                }
                break;
            case 16:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup4,true);
                    setPathState(17);
                }
                break;
            case 17:
                if(!follower.isBusy()) {
                    follower.followPath(park,true);
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
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {

        }
        robot.poseteleop=follower.getPose();

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
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        vision = new Vision(hardwareMap,telemetry,"blue","yellow",follower,1);
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
