package org.firstinspires.ftc.teamcode.movendo;


import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Movendo {
    @Nullable
    private final Telemetry tm;

    private final DcMotorEx fl;
    private final DcMotorEx bl;
    private final DcMotorEx br;
    private final DcMotorEx fr;
    private final BNO055IMU imu;
    private final VoltageSensor voltageSensor;

    public static double TRACK_WIDTH = 17;
    public static double TICKS_TO_RADIANS = (2 * Math.PI) / 537.7;
    public static double WHEEL_RADIUS = 1.8898;

    public static double mV = 50;
    public static double mA = 30;
    public static double maV = 3;
    public static double maA = 1.5;


    public Movendo(@NonNull HardwareMap hardwareMap, @Nullable Telemetry telemetry) {
        fl = (DcMotorEx) hardwareMap.get("fl");
        bl = (DcMotorEx) hardwareMap.get("bl");
        br = (DcMotorEx) hardwareMap.get("br");
        fr = (DcMotorEx) hardwareMap.get("fr");
        imu = (BNO055IMU) hardwareMap.get("imu");
        imu.initialize(new BNO055IMU.Parameters());
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        tm = telemetry;
    }

    public Movendo(@NonNull HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public double getYaw() {
        return imu.getAngularOrientation().thirdAngle;
    }

    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    /* * * * * * * * *\
     * Localization  *
    \* * * * * * * * */

    @NonNull
    private Pose currentPose = new Pose(0, 0, 0);

    @NonNull
    public Pose getCurrentPose() {
        return currentPose;
    }

    public void setCurrentPose(@NonNull Pose currentPose) {
        this.currentPose = currentPose;
    }

    @NonNull
    private WheelValues p_prev = new WheelValues(0, 0, 0, 0);

    private void updateCurrentPose() {
        final var p_cur = new WheelValues(
                fl.getCurrentPosition(),
                bl.getCurrentPosition(),
                br.getCurrentPosition(),
                fr.getCurrentPosition()
        );

        final var v_cur = new WheelValues(
                (p_cur.fl - p_prev.fl) * TICKS_TO_RADIANS * WHEEL_RADIUS,
                (p_cur.bl - p_prev.bl) * TICKS_TO_RADIANS * WHEEL_RADIUS,
                (p_cur.br - p_prev.br) * TICKS_TO_RADIANS * WHEEL_RADIUS,
                (p_cur.fr - p_prev.fr) * TICKS_TO_RADIANS * WHEEL_RADIUS
        );

        final var v_f = (v_cur.fr + v_cur.fl + v_cur.br + v_cur.bl) / 4;
        final var v_s = (v_cur.bl + v_cur.fr - v_cur.fl - v_cur.bl) / 4;
        final var v_w = (v_cur.br + v_cur.fr - v_cur.fl - v_cur.bl) / (4 * TRACK_WIDTH);

        currentPose = currentPose.delta(v_s, v_f, v_w);

        p_prev = p_cur;
    }

    /* * * * * * *\
     * Movement  *
    \* * * * * * */

    @NonNull
    private Pose targetPose = new Pose(0, 0, 0);

    @NonNull
    public Pose getTargetPose() {
        return targetPose;
    }

    public void setTargetPose(@NonNull Pose targetPose) {
        if (currentPose.equals(targetPose)) return;
        this.targetPose = targetPose;

        xProfile = new TrapezoidalProfile(currentPose.x, targetPose.x, mV, mA);
        yProfile = new TrapezoidalProfile(currentPose.y, targetPose.y, mV, mA);
        hProfile = new TrapezoidalProfile(currentPose.h, targetPose.h, mV, mA);
        profileTimer.reset();
    }

    @NonNull
    private TrapezoidalProfile xProfile = new TrapezoidalProfile(0, 0, mV, mA);
    @NonNull
    private TrapezoidalProfile yProfile = new TrapezoidalProfile(0, 0, mV, mA);
    @NonNull
    private TrapezoidalProfile hProfile = new TrapezoidalProfile(0, 0, maV, maA);

    private final ElapsedTime profileTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    private final PIDController xpid = new PIDController(0.1);
    private final PIDController ypid = new PIDController(0.1);
    private final PIDController hpid = new PIDController(0.1);

    @NonNull
    private Pose getDriveVelocities() {
        xpid.set(xProfile.at(profileTimer.time())[2]);
        ypid.set(yProfile.at(profileTimer.time())[2]);

        return new Pose(
                xpid.get(currentPose.x),
                ypid.get(currentPose.y),
                hpid.get(AngleUnit.normalizeRadians(hProfile.at(profileTimer.time())[2] - currentPose.h))
        );
    }

    public void setDriveVelocities(double x, double y, double h) {
        x = x * Math.cos(-getYaw()) - y * Math.sin(-getYaw());
        y = x * Math.sin(-getYaw()) + y * Math.cos(-getYaw());

        final var denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(h), 1);
        fl.setPower((y + x + h) / denominator);
        bl.setPower((y - x + h) / denominator);
        br.setPower((y + x - h) / denominator);
        fr.setPower((y - x - h) / denominator);
    }

    public void update() {
        updateCurrentPose();
        final var vels = getDriveVelocities();
        setDriveVelocities(vels.x, vels.y, vels.h);

        if (tm != null) {
            tm.addData("curPose", getCurrentPose());
            tm.addData("targetPose", getTargetPose());
            tm.addData("vels", vels.toString());
        }
    }

    public static class WheelValues {
        public final double fl;
        public final double bl;
        public final double br;
        public final double fr;

        public WheelValues(double fl, double bl, double br, double fr) {
            this.fl = fl;
            this.bl = bl;
            this.br = br;
            this.fr = fr;
        }
    }
}
