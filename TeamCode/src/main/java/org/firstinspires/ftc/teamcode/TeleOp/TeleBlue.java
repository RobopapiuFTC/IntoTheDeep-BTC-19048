package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.CommandManager;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.pedro.DriverControlled;

import org.firstinspires.ftc.teamcode.Hardware.hardwarePapiu;
import org.firstinspires.ftc.teamcode.Systems.Claw;
import org.firstinspires.ftc.teamcode.Systems.Intake;
import org.firstinspires.ftc.teamcode.Systems.Lift;
import org.firstinspires.ftc.teamcode.Systems.Misumi;
import org.firstinspires.ftc.teamcode.Systems.Outtake;

@TeleOp(name = "TeleBlue")
class TeleBlue extends NextFTCOpMode {
    public Command driverControlled;
    public TeleBlue() {
        super(Claw.INSTANCE, Lift.INSTANCE, Outtake.INSTANCE, Intake.INSTANCE, Misumi.INSTANCE);
    }
    hardwarePapiu robot = new hardwarePapiu(this);

    @Override
    public void onInit() {
        robot.init();
    }

    @Override
    public void onStartButtonPressed() {
        CommandManager.INSTANCE.scheduleCommand(new DriverControlled(gamepadManager.getGamepad1(), true));
        gamepadManager.getGamepad2().getDpadUp().setPressedCommand(
                () -> new SequentialGroup(
                        Claw.INSTANCE.close(),
                        Lift.INSTANCE.toHigh().afterTime(0.2),
                        Outtake.INSTANCE.out().afterTime(1)
                )
        );
        gamepadManager.getGamepad2().getDpadDown().setPressedCommand(
                () -> new SequentialGroup(
                        Claw.INSTANCE.open(),
                        Outtake.INSTANCE.in().afterTime(0.2),
                        Lift.INSTANCE.toDown().afterTime(0.2)
                )
        );
        gamepadManager.getGamepad1().getDpadUp().setPressedCommand(
                () -> new SequentialGroup(
                        Intake.INSTANCE.high(),
                        Misumi.INSTANCE.toHigh()
                )
        );
        gamepadManager.getGamepad1().getDpadDown().setPressedCommand(
                () -> new SequentialGroup(
                        Intake.INSTANCE.high(),
                        Misumi.INSTANCE.toHigh()
                )
        );
    }

    @Override
    public void onUpdate(){
        if(opModeIsActive()){
            //nush cum sa fac cu color sensoru aici sau in commanda plm
        }
    }
}
