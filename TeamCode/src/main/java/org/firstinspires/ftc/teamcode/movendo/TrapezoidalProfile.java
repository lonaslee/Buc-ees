package org.firstinspires.ftc.teamcode.movendo;


public class TrapezoidalProfile {
    public final double start;
    public final double end;
    public final double mV;
    public final double mA;
    public final double distance;
    public final double duration;

    public final double[][] phases;

    public final boolean reversed;

    public TrapezoidalProfile(double start, double end, double mV, double mA) {
        this.start = start;
        this.end = end;
        this.mV = mV;
        this.mA = mA;

        final double dist = end - start;
        reversed = dist < 0;
        if (reversed) distance = -dist;
        else distance = dist;

        if (mV / mA < distance / mV) {
            final double accelTime = mV / mA;
            final double coastTime = distance / mV - mV / mA;
            phases = new double[][]{{mA, accelTime}, {0, coastTime}, {-mA, accelTime}};
        } else {
            final double accelTime = Math.sqrt(distance / mA);
            phases = new double[][]{{mA, accelTime}, {-mA, accelTime}};
        }

        if (reversed) {
            final double[] tmp = phases[0];
            phases[0] = phases[phases.length - 1];
            phases[phases.length - 1] = tmp;
        }

        double duration = 0.0;
        for (double[] phase : phases)
            duration += phase[1];
        this.duration = duration;
    }

    /**
     * Returns double[a, v, x] at time t;
     */
    public double[] at(double t) {
        double[] res = null;

        double v0 = 0.0;
        double x0 = start;
        double a = mA;
        for (double[] phase : phases) {
            a = phase[0];
            final double dt = phase[1];

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
}
