package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.function.BooleanSupplier;
import java.util.stream.Collectors;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Extendo;
import org.firstinspires.ftc.teamcode.subsystems.Flipper;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;
import java.util.ArrayList;

@TeleOp(name = "Zesty Teleop")
public class ZestyTeleop extends LinearOpMode {
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

    private List<BooleanSupplier> runs = new ArrayList<>();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new Drive(hardwareMap, telemetry);
        intake = new Extendo(hardwareMap, telemetry);
        arm = new Arm(hardwareMap, telemetry);
        arm.mid();
        turret = new Turret(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        flipper = new Flipper(hardwareMap, telemetry);
        flipper.unbrace();
        flipper.unflip();

        var savedTurretAngle = 20.0;

        while (opModeInInit()) {
            lift.update();
            turret.update();
            drive.setVels(0, 0, 0);
            arm.update();
            telemetry.update();
            updateRuns();
        }

        while (opModeIsActive()) {
            try {
                pgp1.copy(tgp1);
                tgp1.copy(gamepad1);
                pgp2.copy(tgp2);
                tgp2.copy(gamepad2);
            } catch (Exception ignored) {
            }

            // intake
            if (tgp1.dpad_left && !pgp1.dpad_left) {
                arm.down();
                intake.extend();
            } else if (tgp1.dpad_right && !pgp1.dpad_right) {
                intake.retract();
                arm.mid();

                runs.add(new BooleanSupplier() { // auto transfer
                    private int s = 0;

                    @Override
                    public boolean getAsBoolean() {
                        switch (s) {
                            case 0:
                                if (turret.isZeroed() && lift.isDown() && intake.isRetracted()) {
                                    arm.up();
                                    s++;
                                }
                                break;
                            case 1:
                                if (arm.isArmFinished()) {
                                    arm.open();
                                    s++;
                                }
                                break;
                            case 2:
                                if (arm.isClawFinished()) {
                                    arm.mid();
                                    return false;
                                }
                                break;
                        }
                        return true;
                    }
                });
            } else if (tgp1.dpad_up) {
                intake.setPosition(intake.getCurrentPosition() + 200);
            } else if (tgp1.dpad_down) {
                intake.setPosition(intake.getCurrentPosition() - 200);
            }
            intake.update();

            // arm
            if (tgp1.right_bumper && !pgp1.right_bumper) arm.changeClaw();
            if (tgp1.y && !pgp1.y) arm.up();
            else if (tgp1.x && !pgp1.x) arm.mid();
            else if (tgp1.a && !pgp1.a) arm.down();
            arm.update();

            // turret
            if (tgp2.dpad_left && !pgp2.dpad_left) turret.setAngle(45);
            else if (tgp2.dpad_right && !pgp2.dpad_right) turret.setAngle(-45);
            else if (tgp2.dpad_down && !pgp2.dpad_down) turret.setAngle(0);
            else if (tgp2.dpad_up && !pgp2.dpad_up) turret.setAngle(savedTurretAngle);
            else if (Math.abs(tgp2.right_stick_x) > 0.0)
                turret.setAngle(turret.getTargetAngle() - (double) (tgp2.right_stick_x));
            if (tgp2.left_bumper && !pgp2.left_bumper) savedTurretAngle = turret.getCurrentAngle();
            turret.update();

            // lift
            if (tgp2.y && !pgp2.y) {
                lift.up();
                flipper.brace();
            } else if (tgp2.a && !pgp2.a) {
                lift.down();
                flipper.unbrace();
                turret.setAngle(0);
            } else if (Math.abs(tgp2.left_stick_y) > 0.1) {
                lift.setPosition(lift.getCurrentPosition() - (int) (200 * tgp2.left_stick_y));
                if (tgp2.left_stick_y < 0) flipper.brace();
                else flipper.unbrace();
            }
            lift.update();

            // flipper
            if (tgp2.left_trigger != 0 && pgp2.left_trigger == 0) {
                flipper.flip();

                runs.add(() -> {
                    if (flipper.isFlipperDone()) {
                        flipper.unflip();
                        flipper.unbrace();
                        return false;
                    }
                    return true;
                });
            }
            flipper.update();

            // drive
            if (tgp1.options && !pgp1.options) drive.resetYaw();
            drive.setVels(tgp1.left_stick_x, tgp1.left_stick_y, -tgp1.right_stick_x);
            drive.update();

            updateRuns();
            telemetry.addLine(tgp1.toString());
            telemetry.update();
        }
    }

    private void updateRuns() {
        runs = runs.stream().filter(BooleanSupplier::getAsBoolean).collect(Collectors.toList());
    }
}
