package org.firstinspires.ftc.teamcode.utils;


public class TrapezoidalProfile {
    public final double start;
    public final double end;
    public final double mV;
    public final double mA;
    public final double distance;
    public final double duration;
    public final boolean reversed;

    private final double[][] phases;

    public TrapezoidalProfile(double start, double end, double mV, double mA) {
        this.start = start;
        this.end = end;
        this.mV = mV;
        this.mA = mA;

        final var dist = end - start;
        reversed = dist < 0;
        if (reversed) distance = -dist;
        else distance = dist;

        if (mV / mA < distance / mV) {
            final var accelTime = mV / mA;
            final var coastTime = distance / mV - mV / mA;
            phases = new double[][]{{mA, accelTime}, {0, coastTime}, {-mA, accelTime}};
        } else {
            final var accelTime = Math.sqrt(distance / mA);
            phases = new double[][]{{mA, accelTime}, {-mA, accelTime}};
        }

        if (reversed) {
            final var tmp = phases[0];
            phases[0] = phases[phases.length - 1];
            phases[phases.length - 1] = tmp;
        }

        var duration = 0.0;
        for (double[] phase : phases)
            duration += phase[1];
        this.duration = duration;
    }

    /**
     * Returns double[a, v, x] at time t, where t is time in seconds.
     */
    public double[] at(double t) {
        double[] res = null;

        var v0 = 0.0;
        var x0 = start;
        double a = mA;
        for (double[] phase : phases) {
            a = phase[0];
            final var dt = phase[1];

            if (t < dt) {
                res = new double[]{a, v0 + a * t, x0 + v0 * t + a * (t * t) / 2};
                break;
            }

            x0 += v0 * dt + a * (dt * dt) / 2;
            v0 += a * dt;

            t -= dt;
        }
        if (res == null) res = new double[]{a, v0, x0};
        if (Double.isNaN(res[2])) res[2] = end;
        if (Double.isNaN(res[1])) res[1] = 0;

        return res;
    }

    public boolean isFinished(double t) {
        return t > duration;
    }

    public double[][] getPhases() {
        return phases.clone();
    }
}
