package TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rowanmcalpin.nextftc.core.command.CommandManager;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.pedro.DriverControlled;

import Systems.Claw;
import Systems.Lift;
import Systems.Outtake;

@TeleOp(name = "TeleBlue")
class TeleBlue extends NextFTCOpMode {

    public TeleBlue() {
        super(Claw.INSTANCE, Lift.INSTANCE, Outtake.INSTANCE);
    }


    @Override
    public void onInit() {

    }

    @Override
    public void onStartButtonPressed() {
        CommandManager.INSTANCE.scheduleCommand(new DriverControlled(gamepadManager.getGamepad1(), true));
        gamepadManager.getGamepad2().getDpadUp().setPressedCommand(
                () -> new SequentialGroup(
                        Claw.INSTANCE.close(),
                        Lift.INSTANCE.toHigh().afterTime(0.2),
                        Outtake.INSTANCE.out().afterTime(1),
                        Claw.INSTANCE.open().afterTime(0.2)
                )
        );
        gamepadManager.getGamepad2().getDpadDown().setPressedCommand(
                () -> new SequentialGroup(
                        Claw.INSTANCE.open(),
                        Outtake.INSTANCE.in(),
                        Lift.INSTANCE.toDown().afterTime(0.2)
                )
        );
    }
}
