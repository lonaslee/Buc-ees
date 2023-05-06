package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.*;


@Autonomous
@Config
public class ATest extends LinearOpMode {
    private Extendo intake;
    private Lift lift;
    private Turret turret;
    private Drive drive;
    private Arm arm;
    private Flipper flipper;

    public static double t1 = 1.5;

    @Override
    public void runOpMode() {
        intake = new Extendo(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry, false);
        turret.setAngle(-45);
        drive = new Drive(hardwareMap);
        arm = new Arm(hardwareMap);
        flipper = new Flipper(hardwareMap);
        flipper.unbrace();
        flipper.unflip();

        final double[] poss = new double[]{0.1, 0.1 - 0.02, 0.1 - 0.04, 0.1 - 0.05, 0.1 - 0.06, 0.1 - 0.08, 0.0};


        while (opModeInInit()) {
            lift.update();
            turret.update();
            drive.update(0, 0, 0);
            arm.update();
            telemetry.update();
        }

        waitForStart();

        int s = 0;
        int conesLeft = 5;
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {
            switch (s) {
                case 0: {
                    turret.setAngle(-20);
                    drive.update(0.9, 0, 0.0);
                    if (timer.time() > t1) {
                        s++;
                        drive.update(0, 0, 0);
                    }
                }
                break;
                case 1: {
                    lift.setPosition(Lift.UP - 200);
                    flipper.brace();

                    if (conesLeft > 0) {
                        arm.setPosition(poss[5 - conesLeft]);
                        arm.open();
                        intake.setPosition(1200);
                    }
                    s++;
                }
                break;
                case 2: {
                    if (lift.getPosition() > Lift.UP - 250) {
                        flipper.flip();
                        s++;
                        timer.reset();
                    }
                }
                break;
                case 3: {
                    if (timer.seconds() > 0.5) {
                        flipper.unflip();
                        flipper.unbrace();
                        s++;
                        timer.reset();
                    }
                }
                break;
                case 4: {
                    if (timer.seconds() > 0.2) {
                        lift.down();
                        turret.setAngle(-1);
                        arm.close();
                        s++;
                        timer.reset();
                    }
                }
                break;
                case 5: {
                    if (arm.isClawFinished() && timer.seconds() > 0.8) {
                        arm.up();
                        s++;
                        timer.reset();
                    }
                }
                break;
                case 6: {
                    if (timer.seconds() > 0.5) {
                        intake.retract();
                        s++;
                    }
                }
                break;
                case 7: {
                    if (intake.getPosition() < 10) {
                        arm.open();
                        turret.setAngle(-20);
                        s++;
                        timer.reset();
                    }
                } break;
                case 8: {
                    if (arm.isClawFinished()) {
                        arm.down();
                        s++;
                    }
                } break;
                case 9: {
                    if (turret.getCurrentAngle() < -16 && timer.seconds() > 0.7) {
                        if (--conesLeft == -1) {
                            s++;
                        } else {
                            s = 1;
                        }
                    }
                }
                default:
                    break;
            }

            lift.update();
            turret.update();
            arm.update();
            telemetry.addData("stage", s);
            telemetry.update();
        }
    }
}
