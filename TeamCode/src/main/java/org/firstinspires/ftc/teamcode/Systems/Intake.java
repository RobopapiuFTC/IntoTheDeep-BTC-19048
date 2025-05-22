package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.Controllable;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

public class Intake extends Subsystem {
    // BOILERPLATE
    public static final Intake INSTANCE = new Intake();
    private Intake() {

    }

    // USER CODE
    public DcMotor intake,misumi;
    public Servo intake1,intake2,latch;
    public ColorSensor colorSensor;
    public PIDFController controller = new PIDFController(0.005, 0.0, 0.0, new StaticFeedforward(0.0));
    public static double down=0.0,low=200.0,high=500.0;
    public static double red,blue,green;
    public Command toDown() {
        return new RunToPosition(
                (Controllable) misumi, // MOTOR TO MOVE
                down, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command toLow() {
        return new RunToPosition(
                (Controllable) misumi, // MOTOR TO MOVE
                low, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command toHigh() {
        return new RunToPosition(
                (Controllable) misumi, // MOTOR TO MOVE
                high, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command ground(){
        intake1.setPosition(0.1);
        intake2.setPosition(0.9);
        return null;
    };
    public Command transfer(){
        intake1.setPosition(0.25);
        intake2.setPosition(0.75);
        return null;
    };
    public Command run(String color) throws InterruptedException {
        int ok=0;
        red = colorSensor.red();
        blue = colorSensor.blue();
        green = colorSensor.green();
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setPower(1);
        if(color == "blue")//1 pt blue 0 pt red daca int si dai numa if(color)
             {
            while (ok != 0) {
                if ((red <= 500 && blue >= 500 && green <= 500) || (red >= 500 && blue <= 500 && green >= 500)) {
                    ok = 1;
                    latch.setPosition(0.5);
                    intake.setDirection(DcMotorSimple.Direction.FORWARD);
                    intake.setPower(0.2);
                    Thread.sleep(1000);
                    intake.setPower(0);
                    this.transfer();
                    this.toDown();
                } else if (red >= 500 && blue <= 500 && green <= 500) {
                    ok=1;
                    intake.setDirection(DcMotorSimple.Direction.FORWARD);
                    intake.setPower(0.2);
                }
            }
        }else {
            while (ok != 0) {
                if ((red >= 500 && blue <= 500 && green <= 500) || (red >= 500 && blue <= 500 && green >= 500)) {
                    ok = 1;
                    latch.setPosition(0.5);
                    intake.setDirection(DcMotorSimple.Direction.FORWARD);
                    intake.setPower(0.2);
                    Thread.sleep(1000);
                    intake.setPower(0);
                    this.transfer();
                    this.toDown();
                } else if (red <= 500 && blue >= 500 && green <= 500) {
                    ok=1;
                    intake.setDirection(DcMotorSimple.Direction.FORWARD);
                    intake.setPower(0.2);
                }
            }
        }
        return null;
    };

    @Override
    public void initialize() {
        colorSensor = OpModeData.INSTANCE.getHardwareMap().get(ColorSensor.class, "colorSensor");
        intake1 = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "intake1");
        intake2 = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "intake2");
        intake = OpModeData.INSTANCE.getHardwareMap().get(DcMotor.class, "intakem");
        latch = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "latch");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}