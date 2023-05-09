package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.BooleanSupplier;
import java.util.stream.Collectors;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.subsystems.Flipper;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;
import java.util.ArrayList;

@TeleOp(name = "Zero Systems")
public class ZeroSystems extends LinearOpMode {
    private final Gamepad tgp1 = new Gamepad();
    private final Gamepad pgp1 = new Gamepad();
    private final Gamepad tgp2 = new Gamepad();
    private final Gamepad pgp2 = new Gamepad();

    private Extendo intake;
    private Lift lift;
    private Turret turret;
    private Drive drive;
    private Arm arm;
    private Flipper flipper;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new Drive(hardwareMap, telemetry);
        intake = new Extendo(hardwareMap, telemetry, true);
        arm = new Arm(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry, true);
        lift = new Lift(hardwareMap, telemetry, true);
        flipper = new Flipper(hardwareMap, telemetry);
        flipper.unbrace();
        flipper.unbrace();

        while (opModeInInit()) {
            lift.update();
            turret.update();
            drive.setVels(0, 0, 0);
            drive.update();
            arm.update();
            telemetry.addLine("Systems Zeroed");
            telemetry.update();
        }

        long loops = 0;
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {
            loops++;
            if (loops >= Long.MAX_VALUE) {
                loops = 0;
                timer.reset();
            }

            try {
                pgp1.copy(tgp1);
                tgp1.copy(gamepad1);
                pgp2.copy(tgp2);
                tgp2.copy(gamepad2);
            } catch (Exception ignored) {
            }

            if (tgp1.dpad_up) intake.setUnboundedPosition(intake.getCurrentPosition() + 100);
            else if (tgp1.dpad_down) intake.setUnboundedPosition(intake.getCurrentPosition() - 100);
            if (tgp1.dpad_right && !pgp1.dpad_right) {
                intake.resetEncoder();
                intake.retract();
            }

            if (tgp1.right_bumper && !pgp1.right_bumper) arm.changeClaw();
            if (tgp1.y && !pgp1.y) arm.up();
            else if (tgp1.x && !pgp1.x) arm.mid();
            else if (tgp1.a && !pgp1.a) arm.down();

            if (Math.abs(tgp2.right_stick_x) > 0.1)
                turret.setAngle(turret.getTargetAngle() - (double) (tgp2.right_stick_x));
            if (tgp2.dpad_down && !pgp2.dpad_down) {
                turret.resetEncoder();
                turret.setAngle(0);
            }

            if (Math.abs(tgp2.left_stick_y) > 0.1)
                lift.setUnboundedPosition(lift.getTargetPosition() - (int) (100 * tgp2.left_stick_y));
            if (tgp2.a && !pgp2.a) {
                lift.resetEncoder();
                lift.down();
            }

            drive.setVels(tgp1.left_stick_x, tgp1.left_stick_y, -tgp1.right_stick_x);
            if (tgp1.options && !pgp1.options) drive.resetYaw();

            if (tgp2.left_trigger != 0 && pgp2.left_trigger == 0) {
                flipper.changeFlip();
                flipper.changeBrace();
            }

            drive.update();
            intake.update();
            arm.update();
            turret.update();
            lift.update();
            flipper.update();
            telemetry.addData("hz", loops / timer.seconds());
            telemetry.update();
        }
    }
}
