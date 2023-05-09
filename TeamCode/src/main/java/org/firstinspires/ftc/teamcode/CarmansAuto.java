package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.utils.SleeveDetector;


@Autonomous(name = "CarMAN's Auto"/*, preselectTeleop = "Zesty Teleop" */)
@Config
public class CarmansAuto extends LinearOpMode {
    private Extendo intake;
    private Lift lift;
    private Turret turret;
    private Drive drive;
    private Arm arm;
    private Flipper flipper;
    private SleeveDetector sleeveDetector;

    public static double t1 = 1.5;
    public static double t2 = 0.5;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new Drive(hardwareMap, telemetry);
        intake = new Extendo(hardwareMap, telemetry);
        arm = new Arm(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry);
        turret.setAngle(-45);
        lift = new Lift(hardwareMap, telemetry);
        flipper = new Flipper(hardwareMap, telemetry);
        flipper.unflip();

//        sleeveDetector = new SleeveDetector(hardwareMap, telemetry);

        final var armPoses = new double[]{0.15, 0.14, 0.13, 0.1, 0.07};

        while (opModeInInit()) {
            drive.setVels(0, 0, 0);
            drive.update();
            intake.update();
            arm.update();
            turret.update();
            lift.update();
            flipper.update();
            telemetry.update();
        }

        waitForStart();

//        final var verdict = sleeveDetector.getVerdict();
        final var verdict = 2;

        int stage = -1;
        int conesLeft = 5;
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {
            switch (stage) {
                case -1 -> { // go to cycle spot
                    turret.setAngle(-23);
                    drive.setVels(-0.9, 0, 0.0);
                    if (timer.time() > t1) {
                        stage++;
                        drive.setVels(0, 0, 0);
                    }
                } case 0 -> {
                    if (turret.getCurrentAngle() < -15) {
                        stage++;
                    }
                }
                case 1 -> { // lift to safe pole
                    lift.setPosition(Lift.UP - 200);
                    flipper.brace();

                    if (conesLeft > 0) {
                        arm.setPosition(armPoses[5 - conesLeft]);
                        arm.open();
                        intake.setPosition(conesLeft == 1 ? 1200 : 1150);
                    }
                    stage++;
                }
                case 2 -> { // deposit cone
                    if (lift.getCurrentPosition() > Lift.UP - 250) {
                        flipper.flip();
                        stage++;
                        timer.reset();
                    }
                }
                case 3 -> { // stow flipper and brace
                    if (timer.seconds() > 0.5) {
                        flipper.unflip();
                        flipper.unbrace();
                        stage++;
                        timer.reset();
                    }
                }
                case 4 -> { // stow lift, grab cone
                    if (timer.seconds() > 0.2) {
                        lift.down();
                        turret.setAngle(0);
                        if (conesLeft == 0) stage = 10;

                        arm.close();
                        stage++;
                        timer.reset();
                    }
                }
                case 5 -> { // lift cone
                    if (arm.isClawFinished() && timer.seconds() > 0.6) {
                        arm.up();
                        stage++;
                        timer.reset();
                    }
                }
                case 6 -> { // stow intake
                    if (timer.seconds() > 0.5) {
                        intake.retract();
                        stage++;
                    }
                }
                case 7 -> { // transfer
                    if (intake.getCurrentPosition() < 10) {
                        arm.open();
                        turret.setAngle(-20);
                        stage++;
                        timer.reset();
                    }
                }
                case 8 -> { // lower arm
                    if (timer.seconds() > 0.4) {
                        if (conesLeft > 1) arm.down();
                        else arm.mid();
                        stage++;
                    }
                }
                case 9 -> { // wait for transfer, go back for another cycle
                    if (turret.getCurrentAngle() < -10) {
                        conesLeft--;
                        stage = 1;
                    }
                }
                case 10 -> { // park
                    if (lift.isDown()) {
                        switch (verdict) {
                            case 1 -> drive.setVels(0, 0.9, 0);
                            case 3 -> drive.setVels(0, -0.9, 0);
                            default -> drive.setVels(0, 0, 0);
                        }
                        stage++;
                        timer.reset();
                    }
                }
                case 11 -> { // in zone
                    if (timer.seconds() > t2) {
                        drive.setVels(0, 0, 0);
                        stage++;
                    }
                }
                default -> {
                }
            }

            drive.update();
            intake.update();
            arm.update();
            turret.update();
            lift.update();
            flipper.update();
            telemetry.addData("stage", stage);
            telemetry.update();
        }
    }
}
