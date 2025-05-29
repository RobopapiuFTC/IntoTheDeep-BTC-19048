package org.firstinspires.ftc.teamcode.Hardware;


import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Systems.Intake;
import org.firstinspires.ftc.teamcode.Systems.Lift;
import org.firstinspires.ftc.teamcode.Systems.Outtake;
import org.firstinspires.ftc.teamcode.Systems.Movement;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.Objects;

public class Robot {
    private HardwareMap h;
    private Telemetry t;
    public static double red,blue,green;
    private Gamepad g1,g2;
    private Follower f;
    private Intake i;
    private Movement m;
    private Lift l;
    private Outtake o;
    private boolean a;
    private boolean ro = true;
    public static Pose autoEndPose = new Pose();

    public Pose s = new Pose();
    public double speed = 0.9;
    public Timer iTimer,rTimer;
    public boolean da=false,r,y,b,need=false,running = false;
    public int flip = 1, iState = -1;
    private boolean aInitLoop, frontScore = false, backScore = true, automationActive = false;

    public Robot(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2,boolean robotCentric, boolean alliance) {
        this.h = h;
        this.t = t;
        this.g1 = g1;
        this.g2 = g2;
        this.ro = robotCentric;
        this.a = alliance;

        f = new Follower(h, FConstants.class, LConstants.class);


        //l = new Lift(this.h,this.t);
        i = new Intake(this.h,this.t);
        //o = new Outtake(this.h,this.t);

        iTimer = new Timer();
        rTimer = new Timer();

    }

    public void aPeriodic() {

       // l.periodic();
        i.periodic();
        m.periodic(g1);
       //
        // o.periodic();
        t.update();
    }

    public void tPeriodic() {
        intake();
        run();
       // l.periodic();
        m.periodic(g1);
        i.periodic();
       // o.periodic();
        t.update();
    }

    public void tStart() {
        f.startTeleopDrive();
    }

    public void stop() {
        //  v.off();
        autoEndPose = f.getPose();
    }

    public void dualControls() {

        if (g1.dpad_up){
            setIntakeState(0);
        }
        if(g1.y){
            rotation(da);
            da=!da;
        }
    }

    public HardwareMap getH() {
        return h;
    }

    public Telemetry getT() {
        return t;
    }


    public Gamepad getG1() {
        return g1;
    }

    public Gamepad getG2() {
        return g2;
    }

    public Follower getF() {
        return f;
    }

    public Intake getI() {
        return i;
    }

    public void setI(Intake i) {
        this.i = i;
    }

    public void slowDrive() {
        speed = 0.25;
    }

    public void normalDrive() {
        speed = 0.75;
    }

    public void fastDrive() {
        speed = 0.25;
    }

    public void flip() {
        flip = -1;
    }

    public void unflip() {
        flip = 1;
    }
    public void setIntakeState(int x){
        iState = x;
        iTimer.resetTimer();
    }

    public void intake(){
        t.addData("Intake State", iState);

        switch(iState){
            case 0:
                //i.toHigh();
                setIntakeState(1);
                break;
            case 1:
                need=true;
                running = true;
                setIntakeState(-1);
                break;
        }
    }
    public void rotation(boolean da){
        if(rTimer.getElapsedTimeSeconds()>0.2)
            if(da){
                i.transfer();
                rTimer.resetTimer();
            }else {
                i.ground();
                rTimer.resetTimer();
            }
    }
    public void run(){
        t.addData("Need State", need);
        t.addData("Run State", running);
        t.addData("Alliance",a);
        t.addData("r", r);
        t.addData("b", b);
        t.addData("y", y);
        if (need || running){
           if(need) {
               i.intake.setDirection(DcMotorSimple.Direction.REVERSE);
               i.intake.setPower(0.7);
               i.latch.setPosition(0.3);
           }
            red = i.colorSensor.red();
            blue = i.colorSensor.blue();
            green = i.colorSensor.green();
            if(need) {
                if (red >= 500 && blue <= 500 && green <= 500) {
                    b = false;
                    r = true;
                    y = false;
                } else if (red >= 500 && blue <= 500 && green >= 500) {
                    b = false;
                    r = false;
                    y = true;
                } else if (red <= 500 && blue >= 500 && green <= 500) {
                    b = true;
                    y = false;
                    r = false;
                }
            }
            if(a){
                if (b ==true || y==true) {

                    if(need) {
                        iTimer.resetTimer();
                        i.latch.setPosition(0.8);
                        if(b)g1.rumble(1,0,200);
                        else g1.rumble(0,1,500);
                    }
                    need=false;
                    if (iTimer.getElapsedTimeSeconds()>= 0.3 && iTimer.getElapsedTimeSeconds()<0.8) {
                        i.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                        i.intake.setPower(0.38);
                    }
                    if (iTimer.getElapsedTimeSeconds() >= 0.8 && iTimer.getElapsedTimeSeconds()<1.1) {
                        i.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                        i.intake.setPower(0.5);
                        i.transfer();
                    }
                    if (iTimer.getElapsedTimeSeconds() >= 1.1 && iTimer.getElapsedTimeSeconds() <=1.2) {
                        i.intake.setPower(0);
                    }
                    if(iTimer.getElapsedTimeSeconds() >=1.2){
                        i.latch.setPosition(0.3);
                        running = false;
                        r=false;
                        b=false;
                        y=false;
                    }
                } else if (r == true) {
                    need=false;
                    i.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                    i.intake.setPower(0.38);
                    g1.rumble(1,1,200);
                    running = false;
                    r=false;
                    b=false;
                    y=false;
                }
            } else {
                if (r || y) {

                    if(need) {
                        iTimer.resetTimer();
                        i.latch.setPosition(0.8);
                        if(r)g1.rumble(1,0,200);
                        else g1.rumble(0,1,500);
                    }
                    need=false;
                    if (iTimer.getElapsedTimeSeconds()>= 0.3 && iTimer.getElapsedTimeSeconds()<0.8) {
                        i.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                        i.intake.setPower(0.38);
                    }
                    if (iTimer.getElapsedTimeSeconds() >= 0.8 && iTimer.getElapsedTimeSeconds()<1.1) {
                        i.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                        i.intake.setPower(0.5);
                        i.transfer();
                    }
                    if (iTimer.getElapsedTimeSeconds() >= 1.1 && iTimer.getElapsedTimeSeconds() <=1.2) {
                        i.intake.setPower(0);
                    }
                    if(iTimer.getElapsedTimeSeconds() >=1.2){
                        i.latch.setPosition(0.3);
                        running = false;
                        r=false;
                        b=false;
                        y=false;
                    }
                } else if (b) {
                    need=false;
                    i.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                    i.intake.setPower(0.38);
                    g1.rumble(1,1,200);
                    running = false;
                    r=false;
                    b=false;
                    y=false;
                }
            }
        }
    }


}