package org.firstinspires.ftc.teamcodealpha;

import static org.firstinspires.ftc.teamcodealpha.drive.config.AlphaDriveConstants.TRACK_WIDTH;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcodealpha.drive.config.SampleMecanumDrive;

import java.util.Arrays;

/**
 * FTC 17240 Robot A: Team Grant Bot - REV Robotics Chassis
 */
public class AlphaBot2024 extends SampleMecanumDrive {

    public DcMotor LeftLiftMotor;
    public DcMotor RightLiftMotor;
    private DcMotor LiftTiltMotor;
    public Servo ClawExtend;
    public Servo ClawLevel;
    public Servo ClawClose;

    public AlphaBot2024(HardwareMap hardwareMap) {
        super(hardwareMap);



        // Configure the drive motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRearDrive");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRearDrive");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");

        // Reverse direction of leftFrontDrive, rightFrontDrive, and rightRearDrive
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        // Default brake behavior
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Configure Lift and Claw hardware
        LeftLiftMotor = hardwareMap.get(DcMotor.class, "LeftLiftMotor");
        RightLiftMotor = hardwareMap.get(DcMotor.class, "RightLiftMotor");
        LiftTiltMotor = hardwareMap.get(DcMotor.class, "LiftTiltMotor");

        ClawExtend = hardwareMap.get(Servo.class, "ClawExtend");
        ClawLevel = hardwareMap.get(Servo.class, "ClawLevel");
        ClawClose = hardwareMap.get(Servo.class, "ClawClose");

        ClawExtend.scaleRange(-1.0, 1.0);
        ClawLevel.scaleRange(-1.0, 1.0);
        ClawClose.scaleRange(-1.0, 1.0);

        LeftLiftMotor.setDirection(DcMotor.Direction.REVERSE);
        RightLiftMotor.setDirection(DcMotor.Direction.FORWARD);
        RightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setLiftMotorPowers(double power) {
        LeftLiftMotor.setPower(power);
        RightLiftMotor.setPower(power);
    }
}

