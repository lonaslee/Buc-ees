package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class Extendo {
    private final DcMotorEx motor;
    @Nullable
    private final Telemetry tm;

    public Extendo(
            @NonNull HardwareMap hardwareMap, @Nullable Telemetry telemetry, boolean resetEncoder
    ) {
        motor = (DcMotorEx) hardwareMap.dcMotor.get("extendo");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (resetEncoder) resetEncoder();

        motor.setTargetPosition(RETRACTED);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        tm = telemetry;
    }

    public Extendo(@NonNull HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, false);
    }

    public Extendo(@NonNull HardwareMap hardwareMap) {
        this(hardwareMap, null, false);
    }

    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setPosition(int pos) {
        pos = Math.max(0, pos);
        pos = Math.min(EXTENDED, pos);

        motor.setTargetPosition(pos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setVelocity(12000);
    }

    /**
     * Set position without bound checking, only use for resetting encoders.
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

    public void extend() {
        setPosition(EXTENDED - 600);
    }

    public void retract() {
        setPosition(RETRACTED);
    }

    public void extendFull() {
        setPosition(EXTENDED);
    }

    public void update() {
        if (tm != null) {
            tm.addData("extendoPosition", getCurrentPosition());
        }
    }

    public boolean isRetracted() {
        return getCurrentPosition() < 5;
    }

    public boolean isBusy() {
        return motor.isBusy();
    }

    public static final int EXTENDED = 3300;
    public static final int RETRACTED = 0;
}
