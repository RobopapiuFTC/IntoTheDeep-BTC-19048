package org.firstinspires.ftc.teamcode.Hardware;


import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.Systems.Intake;
import org.firstinspires.ftc.teamcode.Systems.Outtake;
import org.firstinspires.ftc.teamcode.Systems.Movement;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

public class Robot {
    private HardwareMap h;
    private Telemetry t;
    public static double red,blue,green;
    public Gamepad g1,g2;
    public Follower f;
    public Intake i;
    public Movement m;
    public Outtake o;
    private boolean a;
    private boolean ro = true;
    public static Pose autoEndPose = new Pose();

    public Pose s = new Pose();
    public double speed = 0.9;
    public Timer iTimer,rTimer,rsTimer,specTimer;
    public boolean da=true,r,y,b,need=false,running = false,ts=false,spec=false,daS=false;
    public int flip = 1, iState = -1;
    public static int offset=20;
    private boolean aInitLoop, frontScore = false, backScore = true, automationActive = false;

    public Robot(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2, boolean blue,boolean spec) {
        this.h = h;
        this.t = t;
        this.g1 = g1;
        this.g2 = g2;
        this.a = blue;
        this.spec = spec;

        f = new Follower(h, FConstants.class, LConstants.class);

        i = new Intake(this.h,this.t);
        o = new Outtake(this.h,this.t);
        m = new Movement(this.h,this.t);
        iTimer = new Timer();
        rTimer = new Timer();
        rsTimer = new Timer();
        specTimer = new Timer();
    }

    public void aPeriodic() {
        i.periodic();
        o.aperiodic();
        t.update();
    }

    public void tPeriodic() {
        //intake();
        run();
        m.periodic(g1);
        i.periodic();
        o.tperiodic();
        t.update();
    }
    public void tInit(){
        o.targetoffset=0;
        i.sweeper.setPosition(0.8);
        o.outtake1.setPosition(0.27);
        o.outtake2.setPosition(0.27);
        o.rotate.setPosition(0.6);
        i.latch.setPosition(0.25);
        i.intake1.setPosition(0.5);
        i.intake2.setPosition(0.5);
        o.claw.setPosition(0.5);
    }
    public void aInit(){
        o.targetoffset=0;
        i.sweeper.setPosition(0.8);
        o.outtake1.setPosition(0.27);
        o.outtake2.setPosition(0.27);
        o.claw.setPosition(0.27);
        o.rotate.setPosition(0.25);
        i.resetEncoder();
        o.resetEncoder();
        i.latch.setPosition(0.55);
        i.intake1.setPosition(0.5);
        i.intake2.setPosition(0.5);
    }

    public void tStart() {
        f.startTeleopDrive();
    }

    public void stop() {
        //  v.off();
        autoEndPose = f.getPose();
    }

    public void dualControls() {
        if (spec) {
            if(g1.dpad_up && g1.left_trigger>0.3)o.targetoffset+=offset;
            if(g1.dpad_down && g1.left_trigger>0.3)o.targetoffset-=offset;
            if(g1.dpad_right && g1.left_trigger>0.3)i.targetoffset+=offset;
            if(g1.dpad_left && g1.left_trigger>0.3)i.targetoffset-=offset;
            if (g1.dpad_up && !g1.left_bumper && g1.left_trigger<0.3) i.toHigh();
            if (g1.dpad_down && !g1.left_bumper && g1.left_trigger<0.3) i.toDown();
            if (g1.dpad_right && !g1.left_bumper && g1.left_trigger<0.3) i.toMedium();
            if (g1.dpad_left && !g1.left_bumper && g1.left_trigger<0.3) i.toLow();

            if (g1.dpad_up && g1.left_bumper) { //Intake run
                need = true;
                running = true;
            }
            if (g1.dpad_left && g1.left_bumper) {
                o.targetHigh();
            }
            if(g1.dpad_down && g1.left_bumper){
                o.toClimb();
            }
            if (g1.y && !g1.left_bumper && i.target>50) { //Rotate intake
                if (rTimer.getElapsedTimeSeconds() > 0.3) da = !da;
                rotation(da);
            }
            if(g1.right_bumper && !g1.x ){
                if (rsTimer.getElapsedTimeSeconds() > 0.5) daS = !daS;
                rotationS(daS);
            }
            if (g1.a && !g1.left_bumper) { //Take spec off wall
                o.needTS = true;
                o.needS = true;
            }
            if (g1.b && !g1.left_bumper) { //Leave spec on bar
                o.needTSL = true;
                o.needSL = true;
            }
            if (g1.x && !g1.left_bumper && !g1.right_bumper) {
                need = false;
                running = false;
                i.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                i.intake.setPower(0.38);
                i.latch.setPosition(0.25);
            }
            if (g1.x && !g1.left_bumper && g1.right_bumper) {
                need = false;
                running = false;
                i.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                i.intake.setPower(0.7);
                i.latch.setPosition(0.25);
            }
            if (g1.x && g1.left_bumper && !g1.right_bumper) {
                need = false;
                running = false;
                i.intake.setPower(0);
                i.latch.setPosition(0.25);
            }
            if (g1.b && g1.left_bumper) { //Leave spec on bar manual
                o.rotate.setPosition(0.5);
            }
            if (g1.a && g1.left_bumper) o.claw.setPosition(0.27); // Close claw manual
            if(specTimer.getElapsedTimeSeconds()>1 && (g1.ps || g2.y)){
                specTimer.resetTimer();
                spec=!spec;
            }
        }
        else{
            if(g1.dpad_up && g1.left_trigger>0.3)o.targetoffset+=offset;
            if(g1.dpad_down && g1.left_trigger>0.3)o.targetoffset-=offset;
            if(g1.dpad_right && g1.left_trigger>0.3)i.targetoffset+=offset;
            if(g1.dpad_left && g1.left_trigger>0.3)i.targetoffset-=offset;
            if (g1.dpad_up && !g1.left_bumper && g1.left_trigger<0.3) i.toHigh();
            if (g1.dpad_down && !g1.left_bumper && g1.left_trigger<0.3) i.toDown();
            if (g1.dpad_right && !g1.left_bumper && g1.left_trigger<0.3) i.toMedium();
            if (g1.dpad_left && !g1.left_bumper && g1.left_trigger<0.3) i.toLow();

            if (g1.dpad_up && g1.left_bumper) { //Intake run
                need = true;
                running = true;
            }
            if(g1.dpad_right && g1.left_bumper){
                o.targetLow();
            }
            if (g1.dpad_left && g1.left_bumper) {
                o.targetHigh();
            }
            if(g1.dpad_down && g1.left_bumper){
                o.toClimb();
            }
            if (g1.y && !g1.left_bumper && i.target>50) { //Rotate intake
                if (rTimer.getElapsedTimeSeconds() > 0.3) da = !da;
                rotation(da);
            }
            if (g1.x && !g1.left_bumper && !g1.right_bumper) {
                need = false;
                running = false;
                i.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                i.intake.setPower(0.38);
                i.latch.setPosition(0.25);
            }
            if(g1.right_bumper && !g1.x){
                if (rsTimer.getElapsedTimeSeconds() > 0.5) daS = !daS;
                rotationS(daS);
            }
            if (g1.x && !g1.left_bumper && g1.right_bumper) {
                need = false;
                running = false;
                i.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                i.intake.setPower(0.7);
                i.latch.setPosition(0.25);
            }
            if (g1.x && g1.left_bumper && !g1.right_bumper) {
                need = false;
                running = false;
                i.intake.setPower(0);
                i.latch.setPosition(0.25);
            }
            if (g1.b && !g1.left_bumper) { //Take spec off wall
                o.needH = true;
                o.needT = true;
            }
            if (g1.a && !g1.left_bumper) { //Leave spec on bar
                o.needL = true;
                o.needTL = true;
            }
            if (g1.b && g1.left_bumper) { //Leave spec on bar manual
                o.claw.setPosition(0.5);
                o.rotate.setPosition(0.5);
            }
            if (g1.a && g1.left_bumper) o.claw.setPosition(0.27); // Close claw manual
            if(specTimer.getElapsedTimeSeconds()>1 && (g1.ps || g2.y)){
                specTimer.resetTimer();
                spec=!spec;
            }
        }
    }
    public void setIntakeState(int x){
        iState = x;
        iTimer.resetTimer();
    }

    /*public void intake(){
        t.addData("Intake State", iState);

        switch(iState){
            case 0:
                i.toHigh();
                setIntakeState(1);
                break;
            case 1:
                need=true;
                running = true;
                setIntakeState(-1);
                break;
        }
    } */
    public void rotation(boolean da){
        if(rTimer.getElapsedTimeSeconds()>0.3)
            if(da){
                i.transfer();
                rTimer.resetTimer();
            }else {
                i.ground();
                rTimer.resetTimer();
            }
    }
    public void rotationS(boolean yes){
        if(rTimer.getElapsedTimeSeconds()>0.1)
            if(yes){
                i.sweeper.setPosition(0.4);
                rTimer.resetTimer();
            }else {
                i.sweeper.setPosition(0.8);
                rTimer.resetTimer();
            }
    }
    public void spec(boolean baw){
        if(o.timerS.getElapsedTimeSeconds()>0.5 || o.timerSL.getElapsedTimeSeconds()>0.5) {
            if (baw) {
                o.needTS = true;
                o.needS = true;
                ts=!ts;
            } else {
                o.needTSL = true;
                o.needSL = true;
                ts=!ts;
            }
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
               i.latch.setPosition(0.25);
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
                        i.latch.setPosition(0.6);
                        if(b)g1.rumble(1,0,200);
                        else g1.rumble(0,1,500);
                    }
                    need=false;
                    if (iTimer.getElapsedTimeSeconds() >= 0 && iTimer.getElapsedTimeSeconds()<0.2) {
                        i.transfer();
                    }
                    if (iTimer.getElapsedTimeSeconds()>= 0.2 && iTimer.getElapsedTimeSeconds()<0.4) {
                        i.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                        i.intake.setPower(0.38);
                    }
                    if (iTimer.getElapsedTimeSeconds() >= 0.4 && iTimer.getElapsedTimeSeconds()<0.6) {
                        i.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                        i.intake.setPower(0.7);
                    }
                    if(iTimer.getElapsedTimeSeconds() >=0.7 && iTimer.getElapsedTimeSeconds()<1){
                        i.toDown();
                    }
                    if(iTimer.getElapsedTimeSeconds()>=1.1){
                        i.latch.setPosition(0.25);
                        //i.intake.setPower(0);
                        running = false;
                        r=false;
                        b=false;
                        y=false;
                        da=true;
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
                        i.latch.setPosition(0.6);
                        if(r)g1.rumble(1,0,200);
                        else g1.rumble(0,1,500);
                    }
                    need=false;
                    if (iTimer.getElapsedTimeSeconds() >= 0 && iTimer.getElapsedTimeSeconds()<0.2) {
                        i.transfer();
                    }
                    if (iTimer.getElapsedTimeSeconds()>= 0.2 && iTimer.getElapsedTimeSeconds()<0.4) {
                        i.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                        i.intake.setPower(0.38);
                    }
                    if (iTimer.getElapsedTimeSeconds() >= 0.4 && iTimer.getElapsedTimeSeconds()<0.6) {
                        i.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                        i.intake.setPower(0.7);
                    }
                    if(iTimer.getElapsedTimeSeconds() >=0.7 && iTimer.getElapsedTimeSeconds()<1){
                        i.toDown();
                    }
                    if(iTimer.getElapsedTimeSeconds()>=1.3){
                        i.latch.setPosition(0.25);
                       // i.intake.setPower(0);
                        running = false;
                        r=false;
                        b=false;
                        y=false;
                        da=true;
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
    public void leaveSpec(){
        o.needTSL = true;
        o.needSL = true;
    }
    public void takeSpec(){
        o.needTS = true;
        o.needS = true;
    }
    public void HighBucket(){
        o.needH=true;
        o.needT=true;
    }
    public void TransferSamp(){
        o.needL = true;
        o.needTL = true;
    }

}