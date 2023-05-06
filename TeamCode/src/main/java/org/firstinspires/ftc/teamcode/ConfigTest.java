package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Flipper;

@TeleOp
public class ConfigTest extends LinearOpMode {
    private final Gamepad pgp1 = new Gamepad();

    @Override
    public void runOpMode() {
        Arm arm = new Arm(hardwareMap);
        Servo servo = hardwareMap.servo.get("extendoOne");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                servo.setPosition(0);
            } else if (gamepad1.y) {
                servo.setPosition(1);
            }

            if (gamepad1.y && !pgp1.y) {
            }

            if (gamepad1.a && !pgp1.a) {

            }

            if (gamepad1.x && !pgp1.x) {
            }

            if (gamepad1.b && !pgp1.b) {
            }

            telemetry.update();
            try {
                pgp1.copy(gamepad1);
            } catch (RobotCoreException e) {
                throw new RuntimeException(e);
            }
        }
    }
}
