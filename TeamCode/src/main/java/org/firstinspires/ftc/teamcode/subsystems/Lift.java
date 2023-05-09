package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class Lift {
    private final DcMotorEx motor;
    @Nullable private final Telemetry tm;

    public Lift(
            @NonNull HardwareMap hardwareMap, @Nullable Telemetry telemetry, boolean resetEncoder
    ) {
        motor = (DcMotorEx) hardwareMap.dcMotor.get("lift");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        if (resetEncoder) resetEncoder();

        motor.setTargetPosition(DOWN);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tm = telemetry;
    }

    public Lift(@NonNull HardwareMap hardwareMap) {
        this(hardwareMap, null, false);
    }

    public Lift(@NonNull HardwareMap hardwareMap, @Nullable Telemetry telemetry) {
        this(hardwareMap, telemetry, false);
    }

    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setPosition(int pos) {
        pos = Math.max(0, pos);
        pos = Math.min(UP, pos);

        motor.setTargetPosition(pos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setVelocity(12000);
    }

    /**
     * Set position without bound checking, only use for resetting encoder positions.
     */
    public void setUnboundedPosition(int pos) {
        motor.setTargetPosition(pos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setVelocity(12000);
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public int getTargetPosition() {
        return motor.getTargetPosition();
    }

    public void up() {
        setPosition(UP);
    }

    public void down() {
        setPosition(DOWN);
    }

    public boolean isDown() {
        return getCurrentPosition() < 5;
    }

    public boolean isBusy() {
        return motor.isBusy();
    }

    public void update() {
        if (tm != null) {
            tm.addData("liftPosition", getCurrentPosition());
        }
    }

    public static final int UP = 3200;
    public static final int DOWN = 0;
}