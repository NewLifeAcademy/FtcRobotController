package org.firstinspires.ftc.teamcodeiron.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcodeiron.IronBot2025;
import org.firstinspires.ftc.teamcodeiron.autonomous.common.IronBotCommonSequences;

@Autonomous(name = "Auto - Start on Red Goal", preselectTeleOp = "Iron2026Decode")
@Config
public class IronStartOnRedGoal extends LinearOpMode {

    public static double START_POSE_X = -48;
    public static double START_POSE_Y = 48;
    public static double START_HEADING = 135;
    public static double WAYPOINT_FIRE_01_X = -8;
    public static double WAYPOINT_FIRE_01_Y = 8;
    public static double WAYPOINT_FIRE_01_HEADING = 135;
    public static double WAYPOINT_FIRE_02_X = -12;
    public static double WAYPOINT_FIRE_02_Y = 12;
    public static double WAYPOINT_FIRE_02_HEADING = 135;
    public static double SPIKE_APPROACH_X = -9;
    public static double SPIKE_APPROACH_Y = 22;
    public static double SPIKE_APPROACH_HEADING = 90;
    public static double END_POSE_X = 17;
    public static double END_POSE_Y = 18;
    public static double END_HEADING = 90;
    public static double FLYWHEEL_POWER = 0.4; // Power for flywheel motors
    public static int FLYWHEEL_SPINUP_TIME_SEC = 2; // Time to spin up flywheels before firing
    public static int FLYWHEEL_WAIT_BETWEEN_SHOTS_SEC = 3; // Time between shots


    @Override
    public void runOpMode() {
        IronBot2025 robot = new IronBot2025(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (opModeIsActive()) {

            IronBotCommonSequences.RunStartOnGoalSequence(
                    robot,
                    telemetry,
                    START_POSE_X,
                    START_POSE_Y,
                    START_HEADING,
                    WAYPOINT_FIRE_01_X,
                    WAYPOINT_FIRE_01_Y,
                    WAYPOINT_FIRE_01_HEADING,
                    SPIKE_APPROACH_X,
                    SPIKE_APPROACH_Y,
                    SPIKE_APPROACH_HEADING,
                    WAYPOINT_FIRE_02_X,
                    WAYPOINT_FIRE_02_Y,
                    WAYPOINT_FIRE_02_HEADING,
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
