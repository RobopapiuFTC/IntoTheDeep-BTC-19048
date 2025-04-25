package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "leftFront";
        FollowerConstants.leftRearMotorName = "leftRear";
        FollowerConstants.rightFrontMotorName = "rightFront";
        FollowerConstants.rightRearMotorName = "rightRear";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 6;

        FollowerConstants.xMovement = (81.8869 + 81.874 + 81.4576) / 3; //54.97212183753447;
        FollowerConstants.yMovement = (67.3542 + 65.638 + 67.5358)/ 3; //52.295;

        FollowerConstants.forwardZeroPowerAcceleration = (- 28.7826 - 32.8075 - 30.5309) / 3; //-41.278;
        FollowerConstants.lateralZeroPowerAcceleration = (- 62.1219 - 61.9892 - 67.4606) / 3; //-59.7819;

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        FollowerConstants.centripetalScaling = 0.0005;

        FollowerConstants.pathEndTimeoutConstraint = 50;
        FollowerConstants.pathEndTValueConstraint = 0.95;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;

        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.useSecondaryDrivePID = true;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(
                .1,
                0,
                0.01,
                0);
        FollowerConstants.translationalIntegral.setCoefficients(
                0,
                0,
                0,
                0);
        FollowerConstants.translationalPIDFFeedForward=0.02;

        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(
                0.075,
                0,
                0.05,
                0);
        FollowerConstants.secondaryTranslationalIntegral.setCoefficients(
                0,
                0,
                0,
                0);
        FollowerConstants.secondaryTranslationalPIDFFeedForward=0.0005;

        FollowerConstants.headingPIDFCoefficients.setCoefficients(
                2,
                0.001,
                0.1,
                0.001);
        FollowerConstants.headingPIDFFeedForward=0.01;

        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(
                2,
                0.001,
                0.1,
                0.001);
        FollowerConstants.secondaryHeadingPIDFFeedForward=0.0005;

        FollowerConstants.drivePIDFCoefficients.setCoefficients(
                0.01,
                0,
                0.0001,
                0.6,
                0);
        FollowerConstants.drivePIDFFeedForward=0.01;

        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(
                0.02,
                0,
                0.0005,
                0.6,
                0);
        FollowerConstants.secondaryDrivePIDFFeedForward=0.01;
    }
}
