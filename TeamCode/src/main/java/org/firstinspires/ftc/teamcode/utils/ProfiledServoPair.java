package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;


public class ProfiledServoPair {
    public final ServoImplEx servo1;
    public final ServoImplEx servo2;

    private final ElapsedTime timer = new ElapsedTime();
    private TrapezoidalProfile profile;

    private double target;
    private double defaultVel;
    private double defaultAccel;

    public ProfiledServoPair(
            Servo servo1, Servo servo2, double initPos, double defaultVel, double defaultAccel
    ) {
        this.servo1 = (ServoImplEx) servo1;
        this.servo2 = (ServoImplEx) servo2;
        // this.servo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        this.defaultVel = defaultVel;
        this.defaultAccel = defaultAccel;

        target = initPos;
        profile = new TrapezoidalProfile(0, initPos, defaultVel, defaultAccel);
    }

    public ProfiledServoPair(Servo servo1, Servo servo2, double initPos) {
        this(servo1, servo2, initPos, 0.7, 2);
    }

    public double getTarget() {
        return target;
    }

    public void setTarget(double pos, double mV, double mA) {
        if (pos == target) return;
        target = pos;
        profile = new TrapezoidalProfile(getCurrentPosition(), target, mV, mA);
        timer.reset();
    }

    public void setTarget(double pos) {
        setTarget(pos, defaultVel, defaultAccel);
    }

    public boolean isFinished() {
        return profile.isFinished(timer.seconds());
    }

    public double getCurrentPosition() {
        return profile.at(timer.seconds())[2];
    }

    public void update() {
        final double[] res = profile.at(timer.seconds());
        if (servo1.getPosition() != res[2]) {
            servo1.setPosition(res[2]);
            servo2.setPosition(res[2]);
        }
    }

    public double getDefaultVel() {
        return defaultVel;
    }

    public void setDefaultVel(double defaultVel) {
        this.defaultVel = defaultVel;
    }

    public double getDefaultAccel() {
        return defaultAccel;
    }

    public void setDefaultAccel(double defaultAccel) {
        this.defaultAccel = defaultAccel;
    }
}
