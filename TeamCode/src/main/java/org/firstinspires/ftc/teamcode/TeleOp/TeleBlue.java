package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

@TeleOp(name = "TeleBlue", group = "...Sigma")
public class TeleBlue extends OpMode {

    Robot r;


    @Override
    public void init() {
        r = new Robot(hardwareMap, telemetry, gamepad1 , gamepad2,true,true);
        r.aInit();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        r.dualControls();
        r.tPeriodic();
    }
}