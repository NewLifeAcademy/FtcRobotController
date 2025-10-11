package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity redBot1 = botBuilder(meepMeep, new ColorSchemeRedDark());
        RoadRunnerBotEntity blueBot1 = botBuilder(meepMeep, new ColorSchemeBlueDark());
        RoadRunnerBotEntity redBot2 = botBuilder(meepMeep, new ColorSchemeRedLight());
        RoadRunnerBotEntity blueBot2 = botBuilder(meepMeep, new ColorSchemeBlueLight());

        redBot1.runAction(redBot1.getDrive().actionBuilder(new Pose2d(-60, 8, Math.toRadians(90)))
                .splineTo(new Vector2d(-11.5, 32), Math.toRadians(90))
                .build());

        blueBot1.runAction(blueBot1.getDrive().actionBuilder(new Pose2d(-60, -8, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-11.5, 16), Math.toRadians(90))
                .build());

        redBot2.runAction(redBot2.getDrive().actionBuilder(new Pose2d(-60, -24, Math.toRadians(90)))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-11.5, 0, Math.toRadians(90)), Math.toRadians(45))
                .build());

        blueBot2.runAction(blueBot2.getDrive().actionBuilder(new Pose2d(-60, -40, Math.toRadians(90)))
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-11.5, -16, Math.toRadians(90)), Math.toRadians(45))
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

    private static RoadRunnerBotEntity botBuilder(MeepMeep meepMeep, ColorScheme colorScheme) {
        return new DefaultBotBuilder(meepMeep)
                .setColorScheme(colorScheme)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
    }
}