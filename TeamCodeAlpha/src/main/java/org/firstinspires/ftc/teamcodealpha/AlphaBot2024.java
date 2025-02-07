package org.firstinspires.ftc.teamcodealpha;

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

    public DcMotor LeftLiftMotor;
    public DcMotor RightLiftMotor;
    public DcMotor LiftTiltMotor;
    public Servo ClawExtend;
    public Servo ClawLevel;
    public Servo ClawClose;

    public Servo OdometryWheel;

    public AlphaBot2024(HardwareMap hardwareMap) {
        super(hardwareMap);



        // Configure the drive motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRearDrive");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRearDrive");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");

        // Set drive motor directions
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

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

        ClawExtend.scaleRange(0, 1.0);
        ClawLevel.scaleRange(0, 1.0);
        ClawClose.scaleRange(-1.0, 1.0);

        OdometryWheel = hardwareMap.get(Servo.class, "Odometryretract");

        LeftLiftMotor.setDirection(DcMotor.Direction.FORWARD);
        RightLiftMotor.setDirection(DcMotor.Direction.REVERSE);
        RightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftTiltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setLiftMotorPowers(double power) {
        LeftLiftMotor.setPower(power);
        RightLiftMotor.setPower(power);
    }

    public void resetLiftEncoders() {
        LeftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void resetTiltEncoder() {
        LiftTiltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void startLiftToPosition(int position, double power) {
        LeftLiftMotor.setTargetPosition(position);
        RightLiftMotor.setTargetPosition(position);
        LeftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftLiftMotor.setPower(power);
        RightLiftMotor.setPower(power);
    }

    public void waitForLiftToReachPosition() {
        while (LeftLiftMotor.isBusy() && RightLiftMotor.isBusy()) {
            wait(0);
        }
    }

    public void startTiltToPosition(int position, double power) {
        LiftTiltMotor.setTargetPosition(position);
        LiftTiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftTiltMotor.setPower(power);
    }

    public void waitForTiltToReachPosition() {
        while (LiftTiltMotor.isBusy()) {
            wait(100);
        }
    }

    public void lowerOdometryWheel() {
        OdometryWheel.setPosition(0);
    }
    public void clawLevelUp() {
        ClawLevel.setPosition(0.9);
    }

    public void clawLevelNeutral() {
        ClawLevel.setPosition(0.75);
    }

    public void clawLevelDown() {
        ClawLevel.setPosition(0.4);
    }

    public void extendClawArm() {
        ClawExtend.setPosition(0.25);
        clawLevelDown(); //0.4
    }

    public void retractClawArm() {
        ClawExtend.setPosition(1);
        clawLevelNeutral(); //0.75
    }

    public void openClaw() {
        ClawClose.setPosition(1);
    }

    public void closeClaw() {
        ClawClose.setPosition(0);
    }

    public void openClawAndWait() {
        ClawClose.setPosition(1);
        wait(200);
    }
    public void closeClawAndWait() {
        ClawClose.setPosition(0);
        wait(400);
    }

    private void wait(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}

