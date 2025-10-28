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

public class Aaron {
    public static void Run(MeepMeep meepMeep) {
        RoadRunnerBotEntity redBot1 = BotBuilder.Build(meepMeep, new ColorSchemeRedDark());
        redBot1.runAction(redBot1.getDrive().actionBuilder(new Pose2d(-60, 8, Math.toRadians(90)))

                .splineTo(new Vector2d(-11.5, 32), Math.toRadians(90))
                        .waitSeconds(.5)

                        .splineTo(new Vector2d(36,68), Math.toRadians(90))

                        .splineTo(new Vector2d(-60,8), Math.toRadians(90))




                .build());










        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redBot1)
                .start();

    }
}
