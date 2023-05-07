package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.movendo.Movendo;
import org.firstinspires.ftc.teamcode.subsystems.Drive;


@TeleOp
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() {
        Drive dt = new Drive(hardwareMap, telemetry);
        Movendo mv = new Movendo(
                () -> new double[]{dt.fl.getVelocity(), dt.bl.getVelocity(), dt.br.getVelocity(), dt.fr.getVelocity()},
                dt::getYaw,
                telemetry
        );
        mv.setTargetPose(new Movendo.Pose(5, 0, 0));

        waitForStart();
        while (opModeIsActive()) {
            final var vels = mv.toWheelVelocities(mv.getTargetVelocities());
            dt.setPowers(vels);

            mv.update();
            dt.update();
            telemetry.update();
        }
    }
}









