package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.CommandManager;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.pedro.DriverControlled;

import org.firstinspires.ftc.teamcode.Hardware.hardwarePapiu;
import org.firstinspires.ftc.teamcode.Systems.Intake;

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
    }

    @Override
    public void onUpdate(){
        if(opModeIsActive()){
            //nush cum sa fac cu color sensoru aici sau in commanda plm
        }
    }
}
