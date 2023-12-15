package org.firstinspires.ftc.teamcodealpha;

import static org.firstinspires.ftc.teamcodealpha.drive.config.AlphaDriveConstants.TRACK_WIDTH;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcodealpha.drive.config.SampleMecanumDrive;

import java.util.Arrays;

/**
 * FTC 17240 Robot A: Team Grant Bot - REV Robotics Chassis
 */
public class AlphaBot2024 extends SampleMecanumDrive {

    public Servo ClawLiftServo;
    public DcMotor LeftLiftMotor;
    public DcMotor RightLiftMotor;
    public Servo ClawServo;

    public AlphaBot2024(HardwareMap hardwareMap) {
        super(hardwareMap);

        // Override DriveConstants
        TRACK_WIDTH = 15.25; // in

        // Reduce power for easier testing
        VX_PERCENTAGE = 0.6;
        VY_PERCENTAGE = 0.6;
        HEADING_PERCENTAGE = 0.6;

        // Configure the drive motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRearDrive");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRearDrive");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");

        // Set direction of leftRearDrive
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        // Default brake behavior
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Configure Lift and Claw hardware
        ClawLiftServo = hardwareMap.get(Servo.class, "ClawLiftServo");
        LeftLiftMotor = hardwareMap.get(DcMotor.class, "LeftLiftMotor");
        RightLiftMotor = hardwareMap.get(DcMotor.class, "RightLiftMotor");
        ClawServo = hardwareMap.get(Servo.class, "ClawServo");

        ClawLiftServo.scaleRange(-5, 0.8);

        LeftLiftMotor.setDirection(DcMotor.Direction.FORWARD);
        RightLiftMotor.setDirection(DcMotor.Direction.REVERSE);
        RightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}

