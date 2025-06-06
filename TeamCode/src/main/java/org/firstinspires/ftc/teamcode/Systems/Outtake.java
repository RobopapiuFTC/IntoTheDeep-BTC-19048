package org.firstinspires.ftc.teamcode.Systems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.util.Timer;
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

public class Outtake {
    public DcMotorEx lift1,lift2;
    public Servo outtake1,outtake2, claw, rotate;
    public int pidLevel=0;
    public static int target=0;
    public PIDController pid;
    public Telemetry telemetry;

    public boolean r,y,b;
    public int pos;
    public static double p = 0.01, i = 0, d = 0.00000000000005, f = 0.05;
    public static double down=0.0,low=200.0,high=500.0;
    public static double red,blue,green;
    public boolean needL=false,needH=false,needT=true;
    public static Timer timerL,timerH;
    public Outtake(HardwareMap hardwareMap, Telemetry telemetry){
        outtake1 = hardwareMap.get(Servo.class, "outtake1");
        outtake2 = hardwareMap.get(Servo.class, "outtake2");
        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        claw = hardwareMap.get(Servo.class, "claw");
        rotate = hardwareMap.get(Servo.class, "clawR");
        //Go in init in robot or auto prob
        timerL= new Timer();
        timerH= new Timer();
        outtake1.setPosition(0.1);
        outtake2.setPosition(0.9);
        claw.setPosition(0.5);
        rotate.setPosition(0.5);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void update() {
        if(pidLevel == 1) {
            lift1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            lift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

            double pid_output = pid.calculate(getPos(), target);
            double power = pid_output + f;

            if (getPos() < 10 && target < 10) {
                lift1.setPower(0);
                lift2.setPower(0);
            } else {
                lift1.setPower(power);
                lift2.setPower(power);
            }
        } else if (pidLevel == 2){
            target = getPos();
            lift1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            lift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        } else {
            target = getPos();
            lift1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            lift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            lift1.setPower(0);
            lift2.setPower(0);

        }
    }
    public void setTarget(int b) {
        pidLevel = 1;
        target = b;
    }

    public int getPos() {
        pos = lift1.getCurrentPosition();
        return lift1.getCurrentPosition();
    }

    public void init() {
        pid.setPID(p,i,d);
    }

    public void start() {
        target = 0;
    }

    public void toDown() {
        setTarget(0);
    }
    public void targetTransfer(){
        setTarget(30);
    }
    public void targetLow(){
        setTarget(100);
    }
    public void targetHigh(){
        setTarget(500);
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
        update();
        toHigh();
        toLow();
        telemetry();
    }
    public void toHigh() {
        if(needH){
            if(needT){
                timerH.resetTimer();
                needT=false;
            }
            if(timerH.getElapsedTimeSeconds()>0 && timerH.getElapsedTimeSeconds()<=0.1)targetLow();
            if(timerH.getElapsedTimeSeconds()>0.1 && timerH.getElapsedTimeSeconds()<=0.3){
                claw.setPosition(0.5);
                rotate.setPosition(0.3);
                outtake1.setPosition(0.1);
                outtake2.setPosition(0.9);
            }
            if(timerH.getElapsedTimeSeconds()>0.3 && timerH.getElapsedTimeSeconds()<=0.4){
                targetTransfer();
            }
            if(timerH.getElapsedTimeSeconds()>0.4 && timerH.getElapsedTimeSeconds()<=0.5){
                claw.setPosition(0.3);
            }
            if(timerH.getElapsedTimeSeconds()>0.5 && timerH.getElapsedTimeSeconds()<=0.8){
                targetHigh();
            }
            if(timerH.getElapsedTimeSeconds()>1.3 && timerH.getElapsedTimeSeconds()<=1.5){
                outtake1.setPosition(0.6);
                outtake2.setPosition(0.4);
                rotate.setPosition(0.6);
                needH=false;
            }
        }
    }
    public void toLow(){
        if(needL){
            if(needT){
                timerL.resetTimer();
                needT=false;
            }
            if(timerL.getElapsedTimeSeconds()>0 && timerL.getElapsedTimeSeconds()<=0.1)claw.setPosition(0.5);
            if(timerL.getElapsedTimeSeconds()>0.3 && timerL.getElapsedTimeSeconds()<=0.5){
                rotate.setPosition(0.3);
                outtake1.setPosition(0.3);
                outtake2.setPosition(0.7);
            }
            if(timerL.getElapsedTimeSeconds()>0.5 && timerL.getElapsedTimeSeconds()<=0.7){
                toDown();
                needL=false;
            }
        }
    }
}
