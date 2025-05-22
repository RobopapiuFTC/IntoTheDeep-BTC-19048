package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.CommandManager;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.pedro.DriverControlled;

import org.firstinspires.ftc.teamcode.Hardware.hardwarePapiu;
import org.firstinspires.ftc.teamcode.Systems.Intake;

@TeleOp(name = "TeleBlue")
public class TeleBlue extends NextFTCOpMode {
    public Command driverControlled;
    public TeleBlue() {
        super(Intake.INSTANCE);
    }
    hardwarePapiu robot = new hardwarePapiu(this);

    @Override
    public void onInit() {
        robot.init();
    }

    @Override
    public void onStartButtonPressed() {
        CommandManager.INSTANCE.scheduleCommand(new DriverControlled(gamepadManager.getGamepad1(), true));
        gamepadManager.getGamepad1().getDpadUp().setPressedCommand(
                () -> {
                    try {
                        return new SequentialGroup(
                                Intake.INSTANCE.toHigh(),
                                Intake.INSTANCE.ground().afterTime(0.5),
                                Intake.INSTANCE.run("blue").afterTime(0.1)
                        );
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }
        );
    }

    @Override
    public void onUpdate(){
        if(opModeIsActive()){

        }
    }
}
