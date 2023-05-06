package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Flipper {
    private ServoImplEx circle;
    private ServoImplEx brace;
    private boolean flipped = false;
    private boolean braced = false;
    
    private final ElapsedTime timer = new ElapsedTime();
    
    public Flipper(HardwareMap hardwareMap) {
        circle = (ServoImplEx) hardwareMap.get("scoreCone");
        brace = (ServoImplEx) hardwareMap.get("poleGuide");
        brace.setDirection(Servo.Direction.REVERSE);
    }
    
    public void unflip() {
        timer.reset();
        flipped = false;
        circle.setPosition(0.08);
    }
    
    public void flip() {
        timer.reset();
        flipped = true;
        circle.setPosition(0.7);
    }
    
    public boolean isDone() {
        return timer.seconds() > 0.5;
    }
    
    public boolean isFlipped() {
        return flipped;
    }
    
    public boolean isBraced() {
        return braced;
    }
    
    public void changeFlip() {
        if (flipped) {
            unflip();
        } else {
            flip();
        }
    }
    
    public void unbrace() {
        braced = false;
        brace.setPosition(0.15);
    }
    
    public void brace() {
        braced = true;
        brace.setPosition(0.4);
    }
    
    public void changeBrace() {
        if (braced) {
            unbrace();
        } else {
            brace();
        }
    }
}