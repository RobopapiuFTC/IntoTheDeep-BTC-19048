package org.firstinspires.ftc.teamcode.Systems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    public DcMotorEx intake,misumi;
    public Servo intake1,intake2,latch;
    public ColorSensor colorSensor;
    public int pidLevel=0;
    public static int target=0;
    public PIDController pid;
    public Telemetry telemetry;

    public boolean r,y,b;
    public int pos;
    public static double p = 0.01, i = 0, d = 0.00000000000005, f = 0.05;
    public static int down=0,low=200,medium=400,high=600;
    public static double red,blue,green;
    public Intake(HardwareMap hardwareMap, Telemetry telemetry){
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        intake1 = hardwareMap.get(Servo.class, "intake1");
        intake2 = hardwareMap.get(Servo.class, "intake2");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        misumi = hardwareMap.get(DcMotorEx.class, "misumi");
        latch = hardwareMap.get(Servo.class, "latch");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        misumi.setDirection(DcMotorSimple.Direction.REVERSE);
        misumi.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pid = new PIDController(p , i , d);
    }
    public void update() {
        if(pidLevel == 1) {
            pid.setPID(p,i,d);
            misumi.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

            double pid_output = pid.calculate(getPos(), target);
            double power = pid_output + f;

            if (getPos() < 5 && target < 1) {
                misumi.setPower(0);
            } else {
                misumi.setPower(power);
            }
        } else if (pidLevel == 2){
            target = getPos();
            misumi.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        } else {
            target = getPos();
            misumi.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            misumi.setPower(0);
        }
    }
    public void setTarget(int b) {
        pidLevel = 1;
        target = b;
    }

    public int getPos() {
        pos = misumi.getCurrentPosition();
        return misumi.getCurrentPosition();
    }

    public void init() {
        pid.setPID(p,i,d);
    }

    public void start() {
        target = 0;
    }

    public void toDown() {
        setTarget(down);
    }
    public void toLow() {
        setTarget(low);
    }
    public void toMedium(){setTarget(medium);}
    public void toHigh() {
        setTarget(high);
    }

    public boolean roughlyAtTarget() {
        return Math.abs(getPos() - target) < 25;
    }

    public boolean halfwayToTarget() {
        return Math.abs(getPos() - target) < target/2;
    }

    public void pidOn() {
        pidLevel = 1;
    }

    public void pidOff() {
        pidLevel = 2;
    }

    public void telemetry() {
    }

    public void periodic() {
        //color();
        update();
        telemetry();
    }
    public void ground(){
        intake1.setPosition(0.65);
        intake2.setPosition(0.35);
    };
    public void transfer(){
        intake1.setPosition(0.5);
        intake2.setPosition(0.5);
    };
}
