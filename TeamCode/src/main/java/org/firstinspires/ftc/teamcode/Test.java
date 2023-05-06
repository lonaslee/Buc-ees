package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
import java.util.concurrent.Callable;

@TeleOp
public class Test extends LinearOpMode {
    private Gamepad tgp1 = new Gamepad();
    private Gamepad pgp1 = new Gamepad();
    private Gamepad tgp2 = new Gamepad();
    private Gamepad pgp2 = new Gamepad();
    
    private Extendo intake;
    private Lift lift;
    private Turret turret;
    private Drive drive;
    private Arm arm;
    private Flipper flipper;
    
    private List<Callable<Boolean>> runs = new ArrayList<>();
    private List<String> names = new ArrayList<>();
    
    @Override
    public void runOpMode() {
        intake = new Extendo(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        turret = new Turret(hardwareMap, telemetry, true);
        drive = new Drive(hardwareMap);
        arm = new Arm(hardwareMap);
        flipper = new Flipper(hardwareMap);
        flipper.unbrace();
        flipper.unflip();
        
        while (opModeInInit()) {
            updateRuns();
            lift.update();
            turret.update();
            drive.update(0, 0, 0);
            arm.update();
            telemetry.update();
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
                runs.add(
                    new Callable<Boolean>() {
                        private boolean up = false;
                        private ElapsedTime timer = new ElapsedTime();
                        
                        @Override
                        public Boolean call() {
                            if (turret.getAngle() == 0 && lift.isDown()) {
                                if (up && timer.seconds() > 2) {
                                    arm.open();
                                    return false;
                                }
                                arm.up();
                                up = true;
                            }
                            return true;
                        }
                    }
                );
            } else if (tgp1.dpad_up) {
                intake.setPosition(intake.getPosition() + 200);
            } else if (tgp1.dpad_down) {
                intake.setPosition(intake.getPosition() - 200);
            }
            intake.update();
            
            // lift
            if (tgp2.y && !pgp2.y) {
                lift.up();
                flipper.brace();
            } else if (tgp2.a && !pgp2.a) {
                lift.down();
                turret.setAngle(0);
            } else if (Math.abs(tgp2.left_stick_y) > 0.1) {
                lift.setPosition(lift.getPosition() - (int)(200 * tgp2.left_stick_y));
                if (tgp2.left_stick_y < 0) {
                    flipper.brace();
                } else {
                    flipper.unbrace();
                }
            }
            lift.update();
            
            // turret
            if (tgp2.dpad_left && !pgp2.dpad_left) {
                turret.setAngle(45);
            } else if (tgp2.dpad_right && !pgp2.dpad_right) {
                turret.setAngle(-45);
            } else if (tgp2.dpad_down && !pgp2.dpad_down) {
                turret.setAngle(0);
            } else if (Math.abs(tgp2.right_stick_x) > 0.0) {
                turret.setAngle(turret.getAngle() - (double)(tgp2.right_stick_x));
            }
            turret.update();

            // arm
            if (tgp1.right_bumper && !pgp1.right_bumper) {
                arm.changeClaw();
            }
            if (tgp1.y && !pgp1.y) {
                arm.up();
            } else if (tgp1.x && !pgp1.x) {
                arm.mid();
            } else if (tgp1.a && !pgp1.a) {
                arm.down();
            }
            arm.update();

            // flipper
            if (tgp2.left_trigger != 0F && pgp2.left_trigger == 0F) {
                flipper.flip();

                runs.add(
                    new Callable<Boolean>() {
                        @Override
                        public Boolean call() throws Exception {
                            if (flipper.isDone()) {
                                flipper.unflip();
                                flipper.unbrace();
                                return false;
                            }
                            return true;
                        }
                    }
                );
            }
            
            updateRuns();

            drive.update(tgp1.left_stick_x, tgp1.left_stick_y, -tgp1.right_stick_x);
            
            telemetry.addLine(tgp1.toString());
            telemetry.update();
        }
    }
    
    private void updateRuns() {
        runs = runs.stream().filter((f)-> {
                   try{
                       return  f.call();
                   } catch (Exception e) {
                       return false;
                   }
                }
            ).collect(Collectors.toList());
    }
    
}