package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;


public class PIDController {
    public double kP;
    public double kI;
    public double kD;

    private final ElapsedTime timer = new ElapsedTime();

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public PIDController(double kP) {
        this(kP, 0, 0);
    }

    private double setpoint = 0;
    private double integralSum = 0;
    private double lastError = 0;

    public void set(double setpoint) {
        if (this.setpoint != setpoint) {
            integralSum = 0;
            lastError = 0;
            timer.reset();
        }
        this.setpoint = setpoint;
    }

    public double get(double output) {
        final double error = setpoint - output;
        integralSum += error * timer.milliseconds();

        final double pid =
                (error * kP) + (integralSum * kI) + ((error - lastError) / timer.milliseconds() * kD);

        lastError = error;
        timer.reset();

        return pid;
    }
}