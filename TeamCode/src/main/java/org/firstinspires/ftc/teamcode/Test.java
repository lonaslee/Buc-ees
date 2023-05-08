package org.firstinspires.ftc.teamcode;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.movendo.Movendo;
import org.firstinspires.ftc.teamcode.movendo.Pose;


@TeleOp
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() {
        PhotonCore.enable();
        Movendo mv = new Movendo(hardwareMap, telemetry);
        mv.setTargetPose(new Pose(5, 0, 0));

        waitForStart();
        while (opModeIsActive()) {
            mv.update();
            telemetry.update();
        }
    }
}
