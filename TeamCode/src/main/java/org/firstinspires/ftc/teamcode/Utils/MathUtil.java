package org.firstinspires.ftc.teamcode.Utils;

// Copyright (c) FIRST and other WPLib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPLib BSD license file in the root directory of this project.


/** Math utility functions. */
public final class MathUtil {
    private MathUtil() {
        throw new AssertionError("utility class");
    }

    public static double inputModulus(double input, double minimumInput, double maximumInput) {
        double modulus = maximumInput - minimumInput;

        // Wrap input if it's above the maximum input
        int numMax = (int) ((input - minimumInput) / modulus);
        input -= numMax * modulus;

        // Wrap input if it's below the minimum input
        int numMin = (int) ((input - maximumInput) / modulus);
        input -= numMin * modulus;

        return input;
    }
}
