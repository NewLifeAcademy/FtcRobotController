package com.example.meepmeeptesting.util;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BotBuilder {
    public static RoadRunnerBotEntity Build(MeepMeep meepMeep, ColorScheme colorScheme) {
        return new DefaultBotBuilder(meepMeep)
                .setColorScheme(colorScheme)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
    }
}
