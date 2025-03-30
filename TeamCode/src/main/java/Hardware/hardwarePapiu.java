package Hardware;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.localization.Pose;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import java.util.Objects;
import java.util.concurrent.TimeUnit;
import Hardware.VariableStorage;
import com.pedropathing.pathgen.Path;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;

public class hardwarePapiu extends Subsystem{

    public final OpMode myOpMode;

    public DcMotorEx leftFront;
    public DcMotorEx leftBack;
    public DcMotorEx rightFront;
    public DcMotorEx rightBack;
    public DcMotorEx Glisiera; //slot 2 pe Control hub 0-1-2-3 order
    public DcMotorEx Glisiera1;
    public DcMotorEx misumi;
    public DcMotorEx Ridicare2;

    public Servo ServoBrat;
    public Servo ServoBrat1;
    public Servo ClesteS;
    public Servo Intake1;
    public Servo Intake2;
    public Servo faras;
    public Servo catarare;
    public DcMotorEx IntakeActive;
    public ColorSensor culoare;

    public Servo ServoCleste;
    public Servo Cleste;

    public Pose poseteleop;

    public static double red,blue,green,alpha;

    public hardwarePapiu(OpMode opmode) {myOpMode = opmode;}



    public void init() {

    /*    leftFront = myOpMode.hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = myOpMode.hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = myOpMode.hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = myOpMode.hardwareMap.get(DcMotorEx.class, "rightRear");
        Glisiera = myOpMode.hardwareMap.get(DcMotorEx.class, "glisiera");
        Glisiera1 = myOpMode.hardwareMap.get(DcMotorEx.class, "glisiera1");
        misumi = myOpMode.hardwareMap.get(DcMotorEx.class, "misumi");
        IntakeActive = myOpMode.hardwareMap.get(DcMotorEx.class, "active");
        ServoBrat = myOpMode.hardwareMap.get(Servo.class, "brat");
        ServoBrat1 = myOpMode.hardwareMap.get(Servo.class, "brat1");
        Intake1 = myOpMode.hardwareMap.get(Servo.class, "intake1");
        Intake2 = myOpMode.hardwareMap.get(Servo.class, "intake2");
        Cleste = myOpMode.hardwareMap.get(Servo.class, "cleste");
        faras = myOpMode.hardwareMap.get(Servo.class, "faras");
        culoare = myOpMode.hardwareMap.get(ColorSensor.class, "culoare"); */

        //Configurari
       /* Glisiera.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Glisiera1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        culoare.enableLed(true);
        faras.setPosition(0.5);
        Intake1.setPosition(0.95);
        Intake2.setPosition(0.05);
        Cleste.setPosition(0.47);
        Glisiera1.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeActive.setDirection(DcMotorSimple.Direction.REVERSE);
        misumi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        misumi.setMode(DcMotor.RunMode.RUN_USING_ENCODER); */

    }
    public Command movement(Gamepad gamepad1){
        double drive = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
        double strafe = gamepad1.left_stick_x; // Counteract imperfect strafing
        double turn = gamepad1.right_stick_x;

        double frontLeftPower = (drive + strafe + turn);
        double backLeftPower = (drive - strafe + turn);
        double frontRightPower = (drive - strafe - turn);
        double backRightPower = (drive + strafe - turn);
        if(gamepad1.right_trigger>0.3){
            leftFront.setPower(frontLeftPower/3);
            leftBack.setPower(backLeftPower/3);
            rightFront.setPower(frontRightPower/3);
            rightBack.setPower(backRightPower/3);
        }else {
            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);
        }

    }

}