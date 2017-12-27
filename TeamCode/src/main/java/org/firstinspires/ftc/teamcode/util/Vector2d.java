package org.firstinspires.ftc.teamcode.general;

public class Vector2d {
    private double x, y;

    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getMagnitude() {
        return Math.sqrt(x*x + y*y);
    }

    public double getAngle() {
        return Math.atan2(y, x);
    }

    public Vector2d add(Vector2d vector) {
        return new Vector2d(vector.x + x, vector.y + y);
    }

    public Vector2d subtract(Vector2d vector) {
        return new Vector2d(x - vector.x, y - vector.y);
    }

    public Vector2d multiply(double s) {
        return new Vector2d(x * s, y * s);
    }

    public Vector2d divide(double s) {
        if(s == 0)
            return new Vector2d(0, 0);
        return new Vector2d(x / s, y / s);
    }

    public Vector2d normalize() {
        return divide(getMagnitude());
    }

    public Vector2d rotate(double angle) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        return new Vector2d(x*cos + y*sin, -x*sin + y*cos);
    }

    public static Vector2d[] normalize(Vector2d... vectors) {
        double magnitude = 0;
        for(Vector2d vec : vectors) {
            double m = vec.getMagnitude();
            if(m > magnitude)
                magnitude = m;
        }

        for(int i = 0; i < vectors.length; i++)
            vectors[i] = vectors[i].divide(magnitude);

        return vectors;
    }

    public void print() {
        System.out.println("Vec2d: " + x + ", " + y);
    }
}
