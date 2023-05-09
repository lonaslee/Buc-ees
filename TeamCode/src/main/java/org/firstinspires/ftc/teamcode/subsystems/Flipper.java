package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Flipper {
    private final ServoImplEx circle;
    private final ServoImplEx brace;
    @Nullable private final Telemetry tm;

    private boolean flipped = false;
    private boolean braced = false;

    private final ElapsedTime flipTimer = new ElapsedTime();

    public Flipper(@NonNull HardwareMap hardwareMap, @Nullable Telemetry telemetry) {
        circle = (ServoImplEx) hardwareMap.get("scoreCone");
        brace = (ServoImplEx) hardwareMap.get("poleGuide");
        tm = telemetry;
        brace.setDirection(Servo.Direction.REVERSE);
    }

    public Flipper(@NonNull HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public void unflip() {
        flipTimer.reset();
        flipped = false;
        circle.setPosition(0.08);
    }

    public void flip() {
        flipTimer.reset();
        flipped = true;
        circle.setPosition(0.7);
    }

    public boolean isFlipperDone() {
        return flipTimer.seconds() > 0.5;
    }

    public boolean isFlipped() {
        return flipped;
    }

    public void changeFlip() {
        if (flipped) unflip();
        else flip();
    }

    public void unbrace() {
        braced = false;
        brace.setPosition(0.2);
    }

    public void brace() {
        braced = true;
        brace.setPosition(0.4);
    }

    public boolean isBraced() {
        return braced;
    }

    public void changeBrace() {
        if (braced) unbrace();
        else brace();
    }

    public void update() {
        if (tm != null) {
            tm.addData("flipped & braced", isFlipped() + " & " + isBraced());
        }
    }
}
