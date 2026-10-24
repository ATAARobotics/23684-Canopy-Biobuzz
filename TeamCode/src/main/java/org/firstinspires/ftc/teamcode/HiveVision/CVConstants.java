package org.firstinspires.ftc.teamcode.HiveVision;

import com.aaravlabs.safepedropathing.math.Matrix;

public class CVConstants {
    public static final double CAMERA_HEIGHT = 8.5; // CHANGE THIS
    public static final double CAMERA_PITCH = Math.toRadians(0); // CHANGE THIS
    public static final double FORWARD_OFFSET = 9.5; // CHANGE THIS

    public static final Matrix xyzConverter = new Matrix(
            new double[][] {
                    {1, 0, 0},
                    {0, 0, 1},
                    {0, 1, 0},
            });

    public static Matrix createRotationX(double angle) {
        double c = Math.cos(angle);
        double s = Math.sin(angle);
        return new Matrix(
                new double[][] {
                        {1, 0,  0},
                        {0, c, -s},
                        {0, s,  c},
                }
        );
    }

}