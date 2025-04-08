package TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.CommandManager;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.ftc.driving.MecanumDriverControlled;
import com.rowanmcalpin.nextftc.pedro.DriverControlled;

import Hardware.hardwarePapiu;
import Systems.Claw;
import Systems.Intake;
import Systems.Lift;
import Systems.Misumi;
import Systems.Outtake;

@TeleOp(name = "TeleTest")
public class TeleTest extends NextFTCOpMode {
    public Command driverControlled;
    public TeleTest() {
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
                () -> new SequentialGroup(
                        Intake.INSTANCE.high()
                )
        );
        gamepadManager.getGamepad1().getDpadDown().setPressedCommand(
                () -> new SequentialGroup(
                        Intake.INSTANCE.low()
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
