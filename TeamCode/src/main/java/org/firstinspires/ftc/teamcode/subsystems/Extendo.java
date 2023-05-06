package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class Extendo {
    private final DcMotorEx motor;
    private final Telemetry tm;
    
    public Extendo(HardwareMap hardwareMap, Telemetry telemetry) {
        motor = (DcMotorEx) hardwareMap.dcMotor.get("extendo");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(RETRACTED);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        tm = telemetry;
    }
    
    public Extendo(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }
    
    public void setPosition(int pos) {
        pos = Math.max(0, pos);
        pos = Math.min(EXTENDED, pos);
        
        motor.setTargetPosition(pos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);   
        motor.setVelocity(12000);
    }
    
    public int getPosition() {
        return motor.getCurrentPosition();
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
            tm.addData("extendoPosition", getPosition());
        }
    }
    
    public static final int EXTENDED = 3300;
    public static final int RETRACTED = 0;
}