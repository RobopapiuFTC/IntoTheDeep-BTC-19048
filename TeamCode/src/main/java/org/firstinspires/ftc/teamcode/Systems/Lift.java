package org.firstinspires.ftc.teamcode.Systems;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.Controllable;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

public class Lift extends Subsystem {
    public static final Lift INSTANCE = new Lift();
    private Lift() { }

    // USER CODE
    public static double down=0.0,low=500.0,high=1200.0;
    public MotorEx motor1,motor2;
    public MotorGroup Lifts;
    public PIDFController controller = new PIDFController(0.005, 0.0, 0.0, new StaticFeedforward(0.0));
    public Command toDown() {
        return new RunToPosition(
                (Controllable) Lifts, // MOTOR TO MOVE
                down, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command toLow() {
        return new RunToPosition(
                (Controllable) Lifts, // MOTOR TO MOVE
                low, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command toHigh() {
        return new RunToPosition(
                (Controllable) Lifts, // MOTOR TO MOVE
                high, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    @Override
    public void initialize() {
        motor1 = OpModeData.INSTANCE.getHardwareMap().get(MotorEx.class, "motor1");
        motor2 = OpModeData.INSTANCE.getHardwareMap().get(MotorEx.class, "motor2");
        Lifts = new MotorGroup(motor1,motor2);
    }

}
