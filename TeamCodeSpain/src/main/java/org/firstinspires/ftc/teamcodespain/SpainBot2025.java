package org.firstinspires.ftc.teamcodespain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcodespain.drive.config.SampleMecanumDrive;

import java.util.Arrays;

/**
 * FTC 17240 Spain Bot 2025: GoBuilda Mecanum Chassis
 */
public class SpainBot2025 extends SampleMecanumDrive {

    public SpainBot2025(HardwareMap hardwareMap) {
        super(hardwareMap);

        // Configure the drive motors
        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRightMotor");

        // Set drive motor directions
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        // Default brake behavior
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public DcMotor getFrontLeftMotor() {
        return leftFront;
    }

    public DcMotor getBackLeftMotor() {
        return leftRear;
    }

    public DcMotor getBackRightMotor() {
        return rightRear;
    }

    public DcMotor getFrontRightMotor() {
        return rightFront;
    }

    private void wait(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}

