package Systems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.Controllable;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

public class Misumi extends Subsystem{
    public static final Misumi INSTANCE = new Misumi();
    private Misumi() { }

    // USER CODE
    public static double down=0.0,low=200.0,high=500.0;
    public MotorEx motor;
    public PIDFController controller = new PIDFController(0.005, 0.0, 0.0, new StaticFeedforward(0.0));
    public Command toDown() {
        return new RunToPosition(
                (Controllable) motor, // MOTOR TO MOVE
                down, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command toLow() {
        return new RunToPosition(
                (Controllable) motor, // MOTOR TO MOVE
                low, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command toHigh() {
        return new RunToPosition(
                (Controllable) motor, // MOTOR TO MOVE
                high, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    @Override
    public void initialize() {
        motor = OpModeData.INSTANCE.getHardwareMap().get(MotorEx.class, "misumi");
    }
}
