package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;

import java.io.File;

public class SpainBotBlockCompanion extends BlocksOpModeCompanion {
    @ExportToBlocks (
            comment = "Write the lift encoder value to a file on the device",
            tooltip = "Write the lift encoder value",
            parameterLabels = {"Lift Encoder Value"},
            parameterDefaultValues = {"0"}
    )
    public static void writeLiftEncoderToFile(int liftEncoderValue) {
        File file = new File("/sdcard/FIRST/SpainBotLiftEncoder.txt");

        ReadWriteFile.writeFile(file, String.valueOf(liftEncoderValue));
    }

    @ExportToBlocks (
            comment = "Read the lift encoder value from a file on the device",
            tooltip = "Read the lift encoder value",
            parameterLabels = {"Lift Encoder Value"},
            parameterDefaultValues = {"0"}
    )
    public static double readLiftEncoderFromFile() {
        File file = new File("/sdcard/FIRST/SpainBotLiftEncoder.txt");
        String liftEncoderValue = ReadWriteFile.readFile(file).trim();
        return Double.parseDouble(liftEncoderValue);
    }
}
