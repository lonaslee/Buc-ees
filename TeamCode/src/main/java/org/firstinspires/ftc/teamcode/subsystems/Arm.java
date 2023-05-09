package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.ProfiledServoPair;


public class Arm {
    private final ProfiledServoPair servos;
    private final ServoImplEx claw;
    @Nullable private final Telemetry tm;

    private boolean opened = true;
    private boolean up = true;

    private final ElapsedTime timer = new ElapsedTime();

    public Arm(@NonNull HardwareMap hardwareMap, @Nullable Telemetry telemetry) {
        ServoImplEx s1 = (ServoImplEx) hardwareMap.get("extendoOne");
        ServoImplEx s2 = (ServoImplEx) hardwareMap.get("extendoTwo");
        s2.setDirection(Servo.Direction.REVERSE);
        servos = new ProfiledServoPair(s1, s2, 1);

        claw = (ServoImplEx) hardwareMap.get("intake");
        claw.setDirection(Servo.Direction.REVERSE);

        tm = telemetry;
    }

    public Arm(@NonNull HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public void setPosition(double pos) {
        servos.setTarget(pos);
    }

    public double getTargetPosition() {
        return servos.getTarget();
    }

    public double getCurrentPosition() {
        return servos.getCurrentPosition();
    }

    public void up() {
        up = true;
        servos.setTarget(1);
    }

    public void mid() {
        up = false;
        servos.setTarget(0.6);
    }

    public void down() {
        up = false;
        servos.setTarget(0.07);
    }

    public boolean isArmFinished() {
        return servos.isFinished();
    }

    public boolean isUp() {
        return up;
    }

    public void changeArm() {
        if (up) down();
        else up();
    }

    public void update() {
        servos.update();

        if (tm != null) {
            tm.addData("armCurrentPos", getCurrentPosition());
        }
    }

    public void open() {
        timer.reset();
        opened = true;
        claw.setPosition(0.6);
    }

    public void close() {
        timer.reset();
        opened = false;
        claw.setPosition(0.32);
    }

    public boolean isClawFinished() {
        return timer.seconds() > 0.5;
    }

    public boolean isOpened() {
        return opened;
    }

    public void changeClaw() {
        if (opened) close();
        else open();
    }
}
