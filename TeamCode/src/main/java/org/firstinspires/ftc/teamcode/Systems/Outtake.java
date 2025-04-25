package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.MultipleServosToPosition;

import java.util.List;

public class Outtake extends Subsystem {
    // BOILERPLATE
    public static final Outtake INSTANCE = new Outtake();
    private Outtake() { }

    // USER CODE
    public Servo outtake1,outtake2;
    public Command out(){
        return new MultipleServosToPosition(
                List.of(
                        outtake1,
                        outtake2
                ),
                0.9,
                this
        );
    };
    public Command in(){
        return new MultipleServosToPosition(
                List.of(
                        outtake1,
                        outtake2
                ),
                0.1,
                this
        );
    };

    @Override
    public void initialize() {
        outtake1 = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "outtake1");
        outtake2 = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "outtake2");
    }


}
