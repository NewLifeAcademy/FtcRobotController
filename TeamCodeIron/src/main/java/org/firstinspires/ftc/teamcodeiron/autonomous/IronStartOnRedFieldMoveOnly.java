package org.firstinspires.ftc.teamcodeiron.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcodeiron.IronBot2025;
import org.firstinspires.ftc.teamcodeiron.autonomous.common.IronBotCommonSequences;

@Autonomous(name = "Auto - Start on Red Field - Move Only", preselectTeleOp = "Iron2026Decode")
@Config
public class IronStartOnRedFieldMoveOnly extends LinearOpMode {

    public static double START_POSE_X = 63;
    public static double START_POSE_Y = 24;
    public static double START_POSE_HEADING = 180;
    public static double END_POSE_X = 39;
    public static double END_POSE_Y = 30;
    public static double END_POSE_HEADING = 180;

    @Override
    public void runOpMode() {
        IronBot2025 robot = new IronBot2025(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (opModeIsActive()) {

            IronBotCommonSequences.RunStartOnFieldMoveOnlySequence(
                    robot,
                    telemetry,
                    START_POSE_X,
                    START_POSE_Y,
                    START_POSE_HEADING,
                    END_POSE_X,
                    END_POSE_Y,
                    END_POSE_HEADING
            );
        }
    }
}
