package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class Turret {
    private final DcMotorEx motor;

    public static double kP = 0.05;
    public static double kI = 0.0;
    public static double kD = 0;

    private final Telemetry tm;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry, boolean resetEncoder) {
        motor = (DcMotorEx) hardwareMap.get("turret");
        if (resetEncoder)
            resetEncoder();
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tm = telemetry;
    }
    
    public Turret(HardwareMap hardwareMap, boolean resetEncoder) {
        this(hardwareMap, null, resetEncoder);
    }
    
    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, true);
    }
    
    public Turret(HardwareMap hardwareMap) {
        this(hardwareMap, true);
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
    private final ElapsedTime profileTimer = new ElapsedTime();

    private TrapezoidalProfile profile = new TrapezoidalProfile(0, 0, 2000, 500);

    public void setAngle(double degrees) {
        if (getAngle() == degrees) return;
        degrees = Math.max(-90, degrees);
        degrees = Math.min(90, degrees);

        profile = new TrapezoidalProfile(getAngle(), degrees * DEGREES_TO_TICKS, 2000, 600);
        profileTimer.reset();
        
        targetDegrees = degrees;

        integralSum = 0;
        lastError = 0;
        pidTimer.reset();
    }

    public double getAngle() {
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
        double value = kP * error + kI * integralSum + kD * (error - lastError) / pidTimer.milliseconds();

        lastError = error;
        pidTimer.reset();

        motor.setPower(value);

        if (tm != null) {
            tm.addData("turretAngle", getAngle());
            tm.addData("turretPower", value);
            tm.addData("turretCurAngle", getCurrentAngle());
        }
    }

    public static final double TICKS_TO_DEGREES = 360 / 537.7;
    public static final double DEGREES_TO_TICKS = 537.7 / 360;
}