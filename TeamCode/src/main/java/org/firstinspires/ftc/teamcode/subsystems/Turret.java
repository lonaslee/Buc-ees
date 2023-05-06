package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.TrapezoidalProfile;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class Turret {
    private final DcMotorEx motor;
    @Nullable private final Telemetry tm;

    public Turret(
            @NonNull HardwareMap hardwareMap, @Nullable Telemetry telemetry, boolean resetEncoder
    ) {
        motor = (DcMotorEx) hardwareMap.get("turret");
        if (resetEncoder) resetEncoder();
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tm = telemetry;
    }

    public Turret(@NonNull HardwareMap hardwareMap) {
        this(hardwareMap, null, false);
    }

    public Turret(@NonNull HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, false);
    }

    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double targetDegrees = 0;
    private int curTicks = 0;
    private double integralSum = 0;
    private double lastError = 0;
    private final ElapsedTime pidTimer = new ElapsedTime();

    private TrapezoidalProfile profile = new TrapezoidalProfile(0, 0, mV, mA);
    private final ElapsedTime profileTimer = new ElapsedTime();

    public void setAngle(double degrees) {
        if (getTargetAngle() == degrees) return;
        degrees = Math.max(-90, degrees);
        degrees = Math.min(90, degrees);

        profile = new TrapezoidalProfile(getTargetAngle(), degrees * DEGREES_TO_TICKS, mV, mA);
        profileTimer.reset();

        targetDegrees = degrees;

        integralSum = 0;
        lastError = 0;
        pidTimer.reset();
    }

    public double getTargetAngle() {
        return targetDegrees;
    }

    public double getCurrentAngle() {
        return curTicks * TICKS_TO_DEGREES;
    }

    public double getCurrentProfileAngle() {
        return profile.at(profileTimer.seconds())[2] * TICKS_TO_DEGREES;
    }

    public void update() {
        curTicks = motor.getCurrentPosition();
        double error = profile.at(profileTimer.seconds())[2] - curTicks;

        integralSum += error * pidTimer.milliseconds();
        double value =
                kP * error + kI * integralSum + kD * (error - lastError) / pidTimer.milliseconds();

        lastError = error;
        pidTimer.reset();

        motor.setPower(value);

        System.out.println("Turret[ target=" + getTargetAngle() + ", current=" + getCurrentAngle() + ", pid=" + value + " ]");
        if (tm != null) {
            tm.addData("turretAngle", getTargetAngle());
            tm.addData("turretPower", value);
            tm.addData("turretCurAngle", getCurrentAngle());
        }
    }

    public boolean isApproxAt(double degrees) {
        return Math.abs(degrees - getCurrentAngle()) < 3;
    }

    public boolean isZeroed() {
        return isApproxAt(0);
    }

    public boolean isApproxAtTarget() {
        return isApproxAt(getTargetAngle());
    }

    public static double kP = 0.06;
    public static double kI = 0.0;
    public static double kD = 0;

    public static double mV = 2200;
    public static double mA = 400;

    private static final double TICKS_TO_DEGREES = 360 / 537.7;
    private static final double DEGREES_TO_TICKS = 537.7 / 360;
}
