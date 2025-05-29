package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

public class IntakeVechi extends Subsystem {
    // BOILERPLATE
    public static final IntakeVechi INSTANCE = new IntakeVechi();
    private IntakeVechi() {

    }

    // USER CODE
    public MotorEx intake,misumi;
    public Servo intake1,intake2,latch;
    public ColorSensor colorSensor;
    public PIDFController controller = new PIDFController(0.005, 0.0, 0.0, new StaticFeedforward(0.0));
    public static double down=0.0,low=200.0,high=500.0;
    public static double red,blue,green;
    public Command toDown() {
        return new RunToPosition(
                misumi, // MOTOR TO MOVE
                down, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command toLow() {
        return new RunToPosition(
                misumi, // MOTOR TO MOVE
                low, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command toHigh() {
        return new RunToPosition(
                misumi, // MOTOR TO MOVE
                high, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command ground(){
        intake1.setPosition(0.3);
        return new ServoToPosition(
                intake2,
                0.7,
                this
        );
    };
    public Command transfer(){
        intake1.setPosition(0.5);
        return new ServoToPosition(
                intake2,
                0.5,
                this
        );
    };
    public Command run(String color, Command driverControlled){
        ElapsedTime timer = new ElapsedTime();
        int ok=0;
        red = colorSensor.red();
        blue = colorSensor.blue();
        green = colorSensor.green();
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setPower(0.8);
        latch.setPosition(0.3);
        if(color == "blue")//1 pt blue 0 pt red daca int si dai numa if(color)
             {
            while (ok != 1) {        driverControlled.invoke();
                red = colorSensor.red();
                blue = colorSensor.blue();
                green = colorSensor.green();
                if ((red <= 500 && blue >= 500 && green <= 500) || (red >= 500 && blue <= 500 && green >= 500)) {
                    timer.reset();
                    while(timer.seconds() < 1.2) {
                        ok = 1;
                        latch.setPosition(0.8);
                        if (timer.seconds() >= 0.3 && timer.seconds()<0.8) {
                            intake.setDirection(DcMotorSimple.Direction.FORWARD);
                            intake.setPower(0.38);
                        }
                        if (timer.seconds() >= 0.8 && timer.seconds()<1) {
                            this.transfer();
                            intake.setDirection(DcMotorSimple.Direction.REVERSE);
                            intake.setPower(0.38);
                        }
                        if (timer.seconds() >= 1 && timer.seconds()<1.1) {
                            intake.setPower(0);
                        }
                        if (timer.seconds() >= 1.1) {
                            latch.setPosition(0.3);
                        }
                    }
                    return this.toDown();
                } else if (red >= 500 && blue <= 500 && green <= 500) {
                    ok=1;
                    intake.setDirection(DcMotorSimple.Direction.FORWARD);
                    intake.setPower(0.38);
                }
            }
        }else {
            while (ok != 1) {        driverControlled.invoke();
                red = colorSensor.red();
                blue = colorSensor.blue();
                green = colorSensor.green();
                if ((red >= 500 && blue <= 500 && green <= 500) || (red >= 500 && blue <= 500 && green >= 500)) {
                    timer.reset();
                    while(timer.seconds() < 1.2) {
                        ok = 1;
                        latch.setPosition(0.8);
                        if (timer.seconds() >= 0.3 && timer.seconds()<0.8) {
                            intake.setDirection(DcMotorSimple.Direction.FORWARD);
                            intake.setPower(0.38);
                        }
                        if (timer.seconds() >= 0.8 && timer.seconds()<1) {
                            intake.setDirection(DcMotorSimple.Direction.REVERSE);
                            intake.setPower(0.38);
                        }
                        if (timer.seconds() >= 1 && timer.seconds()<1.1) {
                            intake.setPower(0);
                        }
                        if (timer.seconds() >= 1.1) {
                            latch.setPosition(0.3);
                        }
                    }
                    return this.toDown();
                } else if (red <= 500 && blue >= 500 && green <= 500) {
                    ok=1;
                    intake.setDirection(DcMotorSimple.Direction.FORWARD);
                    intake.setPower(0.38);
                }
            }
        }
        return this.transfer();
    };

    @Override
    public void initialize() {
        colorSensor = OpModeData.INSTANCE.getHardwareMap().get(ColorSensor.class, "colorSensor");
        intake1 = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "intake1");
        intake2 = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "intake2");
        intake = new MotorEx("intakem");
        misumi = new MotorEx("misumi");
        latch = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "latch");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}