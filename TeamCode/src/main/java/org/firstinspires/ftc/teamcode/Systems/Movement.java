package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Movement {
    public DcMotorEx leftFront,leftRear,rightFront,rightRear;
    public Movement(HardwareMap hardwareMap, Telemetry telemetry){
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void periodic(Gamepad gamepad1) {
        movement(gamepad1);
    }
    public void movement(Gamepad gamepad1){
        double drive = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
        double strafe = gamepad1.left_stick_x; // Counteract imperfect strafing
        double turn = gamepad1.right_stick_x;

        double frontLeftPower = (drive + strafe + turn);
        double backLeftPower = (drive - strafe + turn );
        double frontRightPower = (drive - strafe - turn);
        double backRightPower = (drive + strafe - turn );
        if(gamepad1.right_trigger>0.3){
            leftFront.setPower(frontLeftPower/3);
            leftRear.setPower(backLeftPower/3);
            rightFront.setPower(frontRightPower/3);
            rightRear.setPower(backRightPower/3);
        } else {
            leftFront.setPower(frontLeftPower);
            leftRear.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);
        }
    }
}
