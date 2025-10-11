package com.example.meepmeeptesting.plans;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.example.meepmeeptesting.util.BotBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AutoFirstWayPointTesting {
    public static void Run(MeepMeep meepMeep) {
        RoadRunnerBotEntity redBot1 = BotBuilder.Build(meepMeep, new ColorSchemeRedDark());
        RoadRunnerBotEntity blueBot1 = BotBuilder.Build(meepMeep, new ColorSchemeBlueDark());
        RoadRunnerBotEntity redBot2 = BotBuilder.Build(meepMeep, new ColorSchemeRedLight());
        RoadRunnerBotEntity blueBot2 = BotBuilder.Build(meepMeep, new ColorSchemeBlueLight());

        redBot1.runAction(redBot1.getDrive().actionBuilder(new Pose2d(-60, 8, Math.toRadians(90)))
                .splineTo(new Vector2d(-11.5, 32), Math.toRadians(90))
                .build());

        blueBot1.runAction(blueBot1.getDrive().actionBuilder(new Pose2d(-60, -8, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-11.5, 16), Math.toRadians(90))
                .build());

        redBot2.runAction(redBot2.getDrive().actionBuilder(new Pose2d(-60, -24, Math.toRadians(90)))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-11.5, 0, Math.toRadians(90)), Math.toRadians(90))
                .build());

        blueBot2.runAction(blueBot2.getDrive().actionBuilder(new Pose2d(-60, -40, Math.toRadians(90)))
                .setTangent(Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(-11.5, -16, Math.toRadians(90)), Math.toRadians(90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redBot1)
                .addEntity(blueBot1)
                .addEntity(redBot2)
                .addEntity(blueBot2)
                .start();

    }
}
