package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class Drive {
    public DcMotorEx fl, bl, br, fr;
    public BNO055IMU imu;
    private DcMotorEx[] motors;
    private final VoltageSensor vsensor;
    
    public Drive(HardwareMap hardwareMap) {
        fl = (DcMotorEx) hardwareMap.get("fl");
        bl = (DcMotorEx) hardwareMap.get("bl");
        br = (DcMotorEx) hardwareMap.get("br");
        fr = (DcMotorEx) hardwareMap.get("fr");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        motors = new DcMotorEx[]{fl, fr, bl, br};
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        imu = (BNO055IMU) hardwareMap.get("imu");
        imu.initialize(new BNO055IMU.Parameters());
        
        vsensor = hardwareMap.voltageSensor.iterator().next();
    }
    
    public static double speed = 1;

    public double getVoltage() {
        return vsensor.getVoltage();
    }



    public double getYaw() {
        return AngleUnit.normalizeRadians(imu.getAngularOrientation().thirdAngle) - offset;
    }

    private double offset = 0;

    public void resetYaw() {
        offset = getYaw();
    }

    public void setPowers(double[] powers) {
        final double velComp = 12 / getVoltage();

        fl.setPower(powers[0] * speed * velComp);
        fr.setPower(powers[1] * speed * velComp);
        bl.setPower(powers[2] * speed * velComp);
        br.setPower(powers[3] * speed * velComp);
    }
    
    public void update(double x, double y, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y - x + rx) / denominator;
        double backLeftPower = (y + x + rx) / denominator;
        double frontRightPower = (y + x - rx) / denominator;
        double backRightPower = (y - x - rx) / denominator;

        double velComp = 12 / getVoltage();

        fl.setPower(frontLeftPower * speed * velComp);
        fr.setPower(frontRightPower * speed * velComp);
        bl.setPower(backLeftPower * speed * velComp);
        br.setPower(backRightPower * speed * velComp);
    }
}
