package Systems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.MultipleServosToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

import java.util.List;

public class Intake extends Subsystem {
    // BOILERPLATE
    public static final Intake INSTANCE = new Intake();
    private Intake() { }

    // USER CODE
    public Servo intake1,intake2;
    public Command high(){
        return new MultipleServosToPosition(
                List.of(
                        intake1,
                        intake2
                ),
                0.9,
                this
        );
    };
    public Command low(){
        return new MultipleServosToPosition(
                List.of(
                        intake1,
                        intake2
                ),
                0.75,
                this
        );
    };

    @Override
    public void initialize() {
        intake1 = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "intake1");
        intake2 = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "intake2");
    }
}


