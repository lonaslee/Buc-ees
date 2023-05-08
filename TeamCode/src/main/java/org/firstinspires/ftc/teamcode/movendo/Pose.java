package org.firstinspires.ftc.teamcode.movendo;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Objects;


public class Pose {
    public final double x;
    public final double y;
    public final double h;

    public Pose(double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = AngleUnit.normalizeRadians(h);
    }

    public Pose delta(double dx, double dy, double dh) {
        return new Pose(x + dx, y + dy, h + dh);
    }

    @Override
    public boolean equals(Object o) {
        return o instanceof Pose && ((Pose) o).hashCode() == hashCode();
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y, h);
    }

    @NonNull
    @Override
    public String toString() {
        return "(" + x + ", " + y + ", " + h + ")";
    }
}
