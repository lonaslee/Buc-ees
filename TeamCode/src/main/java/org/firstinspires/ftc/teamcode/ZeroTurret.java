package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp
public class ZeroTurret extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        new Turret(hardwareMap, telemetry, true);
        telemetry.addLine("Turret Zeroed");
        waitForStart();
    }
}
