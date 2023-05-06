package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;


public class Arm {
    private ProfiledServoPair servos;
    private ServoImplEx claw;
    private boolean opened = true;
    private boolean up = true;

    private final ElapsedTime timer = new ElapsedTime();

    public Arm(HardwareMap hardwareMap) {
        ServoImplEx s1 = (ServoImplEx) hardwareMap.get("extendoOne");
        ServoImplEx s2 = (ServoImplEx) hardwareMap.get("extendoTwo");
        s2.setDirection(Servo.Direction.REVERSE);
        servos = new ProfiledServoPair(s1, s2, 1);

        claw = (ServoImplEx) hardwareMap.get("intake");
        claw.setDirection(Servo.Direction.REVERSE);
    }

    public void setPosition(double pos) {
        servos.setTarget(pos);
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
        servos.setTarget(0.08);
    }

    public boolean isArmFinished() {
        return servos.isFinished();
    }

    public boolean getUp() {
        return up;
    }

    public void update() {
        servos.update();
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
        return timer.seconds() > 0.6;
    }

    public boolean getOpened() {
        return opened;
    }

    public void changeClaw() {
        if (opened) {
            close();
        } else {
            open();
        }
    }

    public void changeArm() {
        if (up) {
            down();
        } else {
            up();
        }
    }
}