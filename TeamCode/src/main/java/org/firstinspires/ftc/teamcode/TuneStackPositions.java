package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Arm;


@TeleOp
@Config
public class TuneStackPositions extends LinearOpMode {
    private final Gamepad tgp1 = new Gamepad();
    private final Gamepad pgp1 = new Gamepad();

    public static double pos = 0.1;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Arm arm = new Arm(hardwareMap, telemetry);
        waitForStart();

        while (opModeIsActive()) {
            try {
                pgp1.copy(tgp1);
                tgp1.copy(gamepad1);
            } catch (Exception ignored) {
            }

            pos -= tgp1.right_stick_y * 0.0001;
            if (Math.abs(tgp1.right_stick_y) > 0.1) arm.setPosition(pos);

            if (tgp1.right_bumper && !pgp1.right_bumper) arm.changeClaw();
            if (tgp1.y && !pgp1.y) arm.up();
            else if (tgp1.x && !pgp1.x) arm.mid();
            else if (tgp1.a && !pgp1.a) arm.down();

            arm.update();

            telemetry.addData("pos", pos);
            telemetry.update();
        }
    }
}
