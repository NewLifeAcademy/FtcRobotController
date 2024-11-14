/*
Java based backup of our IronEagleStrafe2025 OpMode block code (10-31-2024)
*/

package org.firstinspires.ftc.teamcodealpha;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "_20242025IronEagleStrafe (Blocks to Java)")
public class IronEagleStrafe2025 extends LinearOpMode {

    private Servo ClawClose;
    private Servo ClawExtend;
    private Servo ClawLevel;
    private DcMotor rightFrontDrive;
    private DcMotor rightRearDrive;
    private DcMotor leftRearDrive;
    private DcMotor leftFrontDrive;
    private DcMotor LeftLiftMotor;
    private DcMotor RightLiftMotor;
    private DcMotor LiftTiltMotor;

    /**
     * Describe this function...
     */
    private float powerAdjust(float power, float percent) {
        float result;

        result = (power * percent) / 100;
        return result;
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int drivePercentage;
        int strafePercentage;
        int rotatePercentage;
        int Boost_Multiplier;
        double driveValue;
        double strafeValue;
        double rotateValue;

        ClawClose = hardwareMap.get(Servo.class, "ClawClose");
        ClawExtend = hardwareMap.get(Servo.class, "ClawExtend");
        ClawLevel = hardwareMap.get(Servo.class, "ClawLevel");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "rightRearDrive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "leftRearDrive");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        LeftLiftMotor = hardwareMap.get(DcMotor.class, "LeftLiftMotor");
        RightLiftMotor = hardwareMap.get(DcMotor.class, "RightLiftMotor");
        LiftTiltMotor = hardwareMap.get(DcMotor.class, "LiftTiltMotor");

        // Set motor directions
        ClawClose.scaleRange(-1, 1);
        ClawExtend.scaleRange(0, 1);
        ClawLevel.scaleRange(0, 1);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        LeftLiftMotor.setDirection(DcMotor.Direction.FORWARD);
        RightLiftMotor.setDirection(DcMotor.Direction.REVERSE);
        LiftTiltMotor.setDirection(DcMotor.Direction.FORWARD);
        RightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftTiltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Drive speed power as a percent.
        // Set to 100 for full power.
        drivePercentage = 75;
        // Drive speed power as a percent.
        // Set to 100 for full power.
        strafePercentage = 75;
        // Drive speed power as a percent.
        // Set to 100 for full power.
        rotatePercentage = 45;
        // Drive speed power as a percent.
        // Set to 100 for full power.
        Boost_Multiplier = 1;
        // Set orientation
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            ClawExtend.setPosition(0.65);
            ClawLevel.setPosition(0.8);
            while (opModeIsActive()) {
                // Read Values
                driveValue = powerAdjust(-gamepad1.left_stick_y, drivePercentage);
                strafeValue = powerAdjust(-gamepad1.right_stick_x, strafePercentage);
                if (gamepad1.left_trigger > 0) {
                    rotateValue = powerAdjust(-1, gamepad1.left_trigger * 100);
                } else if (gamepad1.right_trigger > 0) {
                    rotateValue = powerAdjust(1, gamepad1.right_trigger * 100);
                } else {
                    rotateValue = powerAdjust(0, rotatePercentage);
                }
                if (gamepad2.a) {
                    ClawClose.setPosition(1);
                }
                if (gamepad2.b) {
                    ClawClose.setPosition(-1);
                }
                if (gamepad2.dpad_right) {
                    ClawExtend.setPosition(0.25);
                    ClawLevel.setPosition(0.4);
                }
                if (gamepad2.dpad_down) {
                    ClawExtend.setPosition(0.1);
                    ClawLevel.setPosition(0);
                }
                if (gamepad2.dpad_left) {
                    ClawExtend.setPosition(1);
                    ClawLevel.setPosition(0.75);
                }
                if (gamepad2.dpad_up) {
                    ClawExtend.setPosition(0);
                    ClawLevel.setPosition(0.2);
                }
                if (gamepad2.right_stick_y > 0) {
                    LiftTiltMotor.setPower(0.5);
                } else {
                    if (gamepad2.right_stick_y == 0) {
                        LiftTiltMotor.setPower(0);
                        LiftTiltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    }
                }
                if (gamepad2.right_stick_y < 0) {
                    LiftTiltMotor.setPower(-0.5);
                } else {
                    if (gamepad2.right_stick_y == 0) {
                        LiftTiltMotor.setPower(0);
                        LiftTiltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    }
                }
                if (gamepad2.left_stick_y > 0) {
                    LeftLiftMotor.setPower(1);
                    RightLiftMotor.setPower(1);
                } else {
                    if (gamepad2.left_stick_y == 0) {
                        RightLiftMotor.setPower(0);
                        LeftLiftMotor.setPower(0);
                    }
                }
                if (gamepad2.left_stick_y < 0) {
                    LeftLiftMotor.setPower(-1);
                    RightLiftMotor.setPower(-1);
                } else {
                    if (gamepad2.left_stick_y == 0) {
                        RightLiftMotor.setPower(0);
                        LeftLiftMotor.setPower(0);
                    }
                }
                // Execute Actions
                drivePower(driveValue, strafeValue, rotateValue);
                // Telemetry Data
                telemetry.addData("drive", driveValue);
                telemetry.addData("strafe", strafeValue);
                telemetry.addData("rotate", rotateValue);
                telemetry.addData("Left-Stick-Y", gamepad2.left_stick_y);
                telemetry.addData("velocity", ((DcMotorEx) LiftTiltMotor).getVelocity());
                telemetry.addData("position", LiftTiltMotor.getCurrentPosition());
                telemetry.addData("is at target", !LiftTiltMotor.isBusy());
                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void drivePower(double drive, double strafe, double rotate) {
        leftFrontDrive.setPower((drive + rotate) - strafe);
        rightFrontDrive.setPower((drive - rotate) + strafe);
        leftRearDrive.setPower(drive + rotate + strafe);
        rightRearDrive.setPower((drive - rotate) - strafe);
    }

    /**
     * Describe this function...
     */
    private String getPowerText(
            // TODO: Enter the type for argument named value
            int value,
            // TODO: Enter the type for argument named percent
            int percent) {
        String text;

        text = "";
        text += "" + value;
        text += " (" + percent;
        text += ")";
        return text;
    }
}
