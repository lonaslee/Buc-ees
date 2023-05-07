package org.firstinspires.ftc.teamcode.movendo;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;
import java.util.Arrays;
import java.util.function.DoubleSupplier;


@Config
public class Movendo {
    public Supplier<double[]> velsSupplier;
    public DoubleSupplier angleSupplier;

    @Nullable
    private final Telemetry tm;

    /**
     * Constructor.
     *
     * @param velsSupplier  Supplier giving double[fl, bl, br, fr] of motor velocities.
     * @param angleSupplier DoubleSupplier of imu angle, in radians.
     */
    public Movendo(
            Supplier<double[]> velsSupplier,
            DoubleSupplier angleSupplier,
            @Nullable Telemetry telemetry
    ) {
        this.velsSupplier = velsSupplier;
        this.angleSupplier = angleSupplier;
        tm = telemetry;
    }

    /**
     * Constructor without telemetry.
     */
    public Movendo(Supplier<double[]> velsSupplier, DoubleSupplier angleSupplier) {
        this(velsSupplier, angleSupplier, null);
    }

    @NonNull
    private TrapezoidalProfile xProfile = new TrapezoidalProfile(0, 0, mV, mA);
    @NonNull
    private TrapezoidalProfile yProfile = new TrapezoidalProfile(0, 0, mV, mA);
    @NonNull
    private TrapezoidalProfile hProfile = new TrapezoidalProfile(0, 0, maV, maA);

    private final PIDController xpid = new PIDController(0.1, 0, 0);
    private final PIDController ypid = new PIDController(0.1, 0, 0);
    private final PIDController hpid = new PIDController(0.1, 0, 0);

    private final ElapsedTime profileTimer = new ElapsedTime();

    @NonNull
    private Pose currentPose = new Pose(0, 0, 0);

    @NonNull
    public Pose getCurrentPose() {
        return currentPose;
    }

    public void setCurrentPose(@NonNull Pose pose) {
        this.currentPose = pose;
    }

    @NonNull
    private Pose targetPose = new Pose(0, 0, 0);

    @NonNull
    public Pose getTargetPose() {
        return targetPose;
    }

    public void setTargetPose(Pose target) {
        if (currentPose.equals(target)) return;

        xProfile = new TrapezoidalProfile(currentPose.x, target.x, mV, mA);
        yProfile = new TrapezoidalProfile(currentPose.y, target.y, mV, mA);
        hProfile = new TrapezoidalProfile(currentPose.h, target.h, maV, maA);
        profileTimer.reset();

        this.targetPose = target;
    }

    public static double LATERAL_MULTIPLIER = 1;
    public static double TRACK_WIDTH = 17;
    public static double TICKS_TO_RADIANS = (2 * Math.PI) / 537.7;
    public static double WHEEL_RADIUS = 1.8898;

    public void update() {
        updateCurrentPose();

        System.out.println("Pose[ ");
        if (tm != null) {
            tm.addData("currentPose", currentPose.toString());
            tm.addData("targetPose", targetPose.toString());
        }
    }

    private void updateCurrentPose() {
        final var wheelVels = velsSupplier.get();
        final var fl = wheelVels[0] * TICKS_TO_RADIANS * WHEEL_RADIUS;
        final var bl = wheelVels[1] * TICKS_TO_RADIANS * WHEEL_RADIUS;
        final var br = wheelVels[2] * TICKS_TO_RADIANS * WHEEL_RADIUS;
        final var fr = wheelVels[3] * TICKS_TO_RADIANS * WHEEL_RADIUS;

        final var vf = (fr + fl + br + bl) / 4;
        final var vs = (bl + fr - fl - br) / 4;
        final var w = (br + fr - fl - bl) / (4 * TRACK_WIDTH);

        currentPose = new Pose(currentPose.x + vf, currentPose.y + vs, currentPose.h + w);
    }

    /**
     * Get profiled pid values of double[x, y, h].
     */
    public double[] getTargetVelocities() {
        final double xerror = xProfile.at(profileTimer.time())[2] - currentPose.x;
        final double yerror = yProfile.at(profileTimer.time())[2] - currentPose.y;
        final double herror = hProfile.at(profileTimer.time())[2] - currentPose.h;

        final double x = xpid.get(xerror);
        final double y = ypid.get(yerror);
        final double h = hpid.get(herror);

        System.out.println("error [x, y, h] : " + Arrays.toString(new double[]{xerror, yerror, herror}));
        System.out.println("prof pid [x, y, h]  : " + Arrays.toString(new double[]{x, y, h}));

        return new double[]{x, y, h};
    }

    /**
     * Get field centric double[fl, bl, br, fr] motor powers.
     */
    @NonNull
    public double[] toWheelVelocities(double x, double y, double h) {
        x = x * Math.cos(-currentPose.h) - y * Math.sin(-currentPose.h);
        y = x * Math.sin(-currentPose.h) + y * Math.cos(-currentPose.h);

        final double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(h), 1);
        final double fl = (y + x + h) / denominator;
        final double bl = (y - x + h) / denominator;
        final double br = (y + x - h) / denominator;
        final double fr = (y - x - h) / denominator;

        return new double[]{fl, bl, br, fr};
    }

    @NonNull
    public double[] toWheelVelocities(@NonNull double[] velocities) {
        return toWheelVelocities(velocities[0], velocities[1], velocities[2]);
    }

    public static final double mV = 10;
    public static final double mA = 10;
    public static final double maV = 10;
    public static final double maA = 10;

    public static final class Pose {
        public final double x;
        public final double y;
        public final double h;

        public Pose(double x, double y, double h) {
            this.x = x;
            this.y = y;
            this.h = h;
        }

        @Override
        public boolean equals(Object obj) {
            return obj instanceof Pose && ((Pose) obj).hashCode() == hashCode();
        }

        @Override
        public int hashCode() {
            return Objects.hash(x, y, h);
        }

        @NonNull
        @Override
        public String toString() {
            return "Pose[ (" + roundDecimal(x) + ", " + roundDecimal(y) + "), " + roundDecimal(h) + " ]";
        }
    }

    public static double roundDecimal(double decimal, int places) {
        final var power = Math.pow(10, places);
        return Math.round(decimal * power) / power;
    }

    public static double roundDecimal(double decimal) {
        return roundDecimal(decimal, 2);
    }
}
