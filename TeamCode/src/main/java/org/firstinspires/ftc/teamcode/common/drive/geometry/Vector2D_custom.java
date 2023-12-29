package org.firstinspires.ftc.teamcode.common.drive.geometry;

public class Vector2D_custom {
    public double x,y;

    public Vector2D_custom(double x, double y){
        this.x = x;
        this.y = y;
    }

    public static Vector2D_custom fromHeadingAndMagnitude(double h, double m){
        return new Vector2D_custom(Math.cos(h) * m, Math.sin(h) * m);
    }

    public Vector2D_custom mult(double scalar) {
        return new Vector2D_custom(x * scalar, y * scalar);
    }

    public Vector2D_custom divide(double scalar) {
        return new Vector2D_custom(x / scalar, y / scalar);
    }

    public Vector2D_custom subt(Vector2D_custom other) {
        return new Vector2D_custom(x - other.x, y - other.y);
    }

    public double dot(Vector2D_custom other) {
        return x * other.x + y * other.y;
    }

    public double magnitude() {
        return Math.hypot(x, y);
    }

    public Vector2D_custom unit() {
        return this.divide(magnitude());
    }

    public Vector2D_custom rotate(double angle) {
        return new Vector2D_custom(
                x * Math.cos(angle) - y * Math.sin(angle),
                x * Math.sin(angle) + y * Math.cos(angle));
    }

    public double cross(Vector2D_custom other) {
        return x * other.y - y * other.x;
    }

    @Override
    public String toString() {
        return String.format("{%.2f, %.2f}", x, y);
    }
}
