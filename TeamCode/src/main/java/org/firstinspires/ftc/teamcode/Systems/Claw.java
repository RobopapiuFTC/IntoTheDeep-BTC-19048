package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

public class Claw extends Subsystem {
    // BOILERPLATE
    public static final Claw INSTANCE = new Claw();
    private Claw() { }

    // USER CODE
    public Servo claw;
    public String name = "claw";
    public Command open() {
        return new ServoToPosition(
                claw,
                0.9,
                this);
    }
    public Command close() {
        return new ServoToPosition(
                claw,
                0.5,
                this);
    }
    @Override
    public void initialize() {
        claw = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, name);
    }


}
