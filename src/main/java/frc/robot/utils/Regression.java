package frc.robot.utils;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import frc.robot.Constants.ShooterRegressionConstants;

public class Regression {

    // Sample data points
    public static double[] distances = new double[] { 1, 6, 7, 9, 12, 20 };
    public static double[] velocities = new double[] { 2, 8, 6, 10, 14, 41 };
    public static double[] angles = new double[] { 3, 9, 7, 11, 15, 42 };

    static SplineInterpolator interpolator = new SplineInterpolator();
    static PolynomialSplineFunction velocityFunction = interpolator.interpolate(distances, velocities);
    static PolynomialSplineFunction angleFunction = interpolator.interpolate(distances, angles);

    public static double[] calculateDesiredShooterState(double distance) {
        double predictedVelocity = 0;
        double predictedAngle = 0;

        // Ensure the distance is within the bounds of the original data
        if (distance >= ShooterRegressionConstants.distances[0]
                && distance <= ShooterRegressionConstants.distances[ShooterRegressionConstants.distances.length - 1]) {
            predictedVelocity = velocityFunction.value(distance);
            predictedAngle = angleFunction.value(distance);

        } else {
            // Handle the case where the distance is out of bounds (will just return a
            // shooter state that has 0 velocity and 0 angle)
            System.out.println("Distance is out of the valid range.");

        }
        // Print the predicted shooter velocity
        System.out
                .println("Predicted shooter velocity at " + distance + " meters: " + predictedVelocity + " units");

        // Print the predicted shooter angle
        System.out.println("Predicted shooter angle at " + distance + " meters: " + predictedAngle + " units");

        double[] desiredShooterStateArray = { predictedVelocity, predictedAngle };

        return desiredShooterStateArray;

    }

    public static void main(String[] args) {

        calculateDesiredShooterState(29);

    }

}
