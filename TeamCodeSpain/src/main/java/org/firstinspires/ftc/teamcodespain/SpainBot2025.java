package org.firstinspires.ftc.teamcodespain;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunLEDStick;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Arrays;

/**
 * FTC 17240 Spain Bot 2025: GoBuilda Mecanum Chassis
 */
public class SpainBot2025 extends MecanumDrive {

    public SparkFunLEDStick ledStick;

    public SpainBot2025(HardwareMap hardwareMap) {
        super(hardwareMap, new Pose2d(0, 0, 0));

        // Configure the LED stick
        ledStick = hardwareMap.get(SparkFunLEDStick.class, "backLEDStick");
        ledStick.setBrightness(1);  // Set initial brightness (0-31)
    }

    public void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
    }

    public void odometryBasedMove(double forwardInches, double strafeInches, double rotationDegrees) {
        Action action = this.actionBuilder(this.localizer.getPose())
                .splineTo(new Vector2d(forwardInches, strafeInches), Math.toRadians(rotationDegrees))
                .build();

        Actions.runBlocking(new SequentialAction(action));
    }

    public void wait(int seconds) {
        try {
            Thread.sleep(seconds * 1000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}

