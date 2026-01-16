package org.firstinspires.ftc.teamcodeiron.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcodeiron.IronBot2025;
import org.firstinspires.ftc.teamcodeiron.autonomous.common.IronBotCommonSequences;

@Autonomous(name = "Auto - Start on Red Field - Fire", preselectTeleOp = "Iron2026Decode")
@Config
public class IronStartOnRedFieldFire extends LinearOpMode {

    public static double START_POSE_X = 63;
    public static double START_POSE_Y = 24;
    public static double START_HEADING = 180;
    public static double WAYPOINT_FIRE_X = 58;
    public static double WAYPOINT_FIRE_Y = 24;
    public static double WAYPOINT_FIRE_HEADING = 160;
    public static double END_POSE_X = 39;
    public static double END_POSE_Y = 30;
    public static double END_HEADING = 180;
    public static double FLYWHEEL_POWER = 0.6; // Power for flywheel motors
    public static int FLYWHEEL_SPINUP_TIME_SEC = 3; // Time to spin up flywheels before firing
    public static int FLYWHEEL_WAIT_BETWEEN_SHOTS_SEC = 3; // Time between shots


    @Override
    public void runOpMode() {
        IronBot2025 robot = new IronBot2025(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (opModeIsActive()) {

            IronBotCommonSequences.RunStartOnFieldFireSequence(
                    robot,
                    telemetry,
                    START_POSE_X,
                    START_POSE_Y,
                    START_HEADING,
                    WAYPOINT_FIRE_X,
                    WAYPOINT_FIRE_Y,
                    WAYPOINT_FIRE_HEADING,
                    END_POSE_X,
                    END_POSE_Y,
                    END_HEADING,
                    FLYWHEEL_POWER,
                    FLYWHEEL_SPINUP_TIME_SEC,
                    FLYWHEEL_WAIT_BETWEEN_SHOTS_SEC
            );
        }
    }
}
