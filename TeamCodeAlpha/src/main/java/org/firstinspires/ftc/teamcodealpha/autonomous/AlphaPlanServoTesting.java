/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcodealpha.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcodealpha.AlphaBot2024;
import org.firstinspires.ftc.teamcode.AlphaBotBlockCompanion;

/*
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "Auto - Servo Testing", preselectTeleOp = "TestLiftEncoderPassthrough")
@Disabled
@Config
public class AlphaPlanServoTesting extends LinearOpMode {
    public static double LIFT_ASCENT_SPEED = 1;
    public static double LIFT_DESCENT_SPEED = 1;
    public static int TESTING_LIFT_HEIGHT = 700;
    public static int TESTING_LIFT_END_HEIGHT = 350;

    @Override
    public void runOpMode() {
        AlphaBot2024 drive = new AlphaBot2024(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        drive.lowerOdometryWheel();

        waitForStart();

        if (opModeIsActive()) {
            drive.resetLiftEncoders();
            drive.resetTiltEncoder();
            drive.closeClawAndWait();
            drive.retractClawArm();

            // raise lift to testing height
            drive.startLiftToPosition(TESTING_LIFT_HEIGHT, LIFT_ASCENT_SPEED);
            drive.waitForLiftToReachPosition();

            // claw close test
            drive.openClaw();
            sleep(1000);
            drive.closeClaw();
            sleep(1000);

            // claw level test
            drive.clawLevelUp();
            sleep(1000);
            drive.clawLevelNeutral();
            sleep(1000);
            drive.clawLevelDown();
            sleep(1000);
            drive.clawLevelNeutral();
            sleep(1000);

            // claw extend test
            drive.extendClawArm();
            sleep(1000);
            drive.retractClawArm();
            sleep(1000);

            drive.startLiftToPosition(TESTING_LIFT_END_HEIGHT, LIFT_DESCENT_SPEED);
            drive.waitForLiftToReachPosition();

            // write lift encoder value to file
            AlphaBotBlockCompanion.writeLiftEncoderToFile(drive.LeftLiftMotor.getCurrentPosition());

            // stop autonomous and wait for finish
            sleep(30000);
        }
    }   // end runOpMode()
}   // end class
