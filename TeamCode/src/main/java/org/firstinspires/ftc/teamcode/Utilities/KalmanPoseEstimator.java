package org.firstinspires.ftc.teamcode.Utilities;

import com.acmerobotics.roadrunner.Pose2d;
import org.ejml.simple.SimpleMatrix;

public class KalmanPoseEstimator {
    // State Vector x = [x_pos, y_pos, heading] (3x1 matrix)
    private SimpleMatrix x;

    // Covariance Matrix P (3x3 matrix) representing uncertainty
    private SimpleMatrix P;

    // Process Noise Q (3x3 matrix) - Uncertainty in Odometry
    private SimpleMatrix Q;

    // Measurement Noise R (3x3 matrix) - Uncertainty in Vision
    private SimpleMatrix R;

    public KalmanPoseEstimator(Pose2d startPose) {
        // 1. Initialize State Vector x
        x = new SimpleMatrix(3, 1, true, new double[]{
                startPose.position.x,
                startPose.position.y,
                startPose.heading.toDouble()
        });

        // 2. Initialize Covariance P (Initial uncertainty)
        // Identity matrix means we are somewhat confident, but not infinitely
        P = SimpleMatrix.identity(3).scale(1.0);

        // 3. Initialize Process Noise Q (Odometry Drift)
        // Tuning: Lower numbers = we trust odometry physics more.
        // We assume X and Y drift about 0.002 inches per loop, Heading drifts 0.001 rads
        Q = new SimpleMatrix(3, 3, true, new double[]{
                0.002, 0, 0,
                0, 0.002, 0,
                0, 0, 0.001
        });

        // 4. Initialize Measurement Noise R (Vision Jitter)
        // Tuning: Higher numbers = we trust vision less (it's noisier).
        // We start with a base value, but we will scale this dynamically based on distance.
        R = new SimpleMatrix(3, 3, true, new double[]{
                5.0, 0, 0,    // X variance (inches^2)
                0, 5.0, 0,    // Y variance
                0, 0, 0.5     // Heading variance (rad^2)
        });
    }

    /**
     * Prediction Step: Apply the Odometry Delta
     * x_new = x_old + delta_odometry
     * P_new = P_old + Q
     */
    public void predict(Pose2d odoDelta) {
        // Create control input vector u from the odometry change
        SimpleMatrix u = new SimpleMatrix(3, 1, true, new double[]{
                odoDelta.position.x,
                odoDelta.position.y,
                odoDelta.heading.toDouble()
        });

        // Update State: x = x + u
        x = x.plus(u);

        // Update Covariance: P = P + Q
        P = P.plus(Q);

        // Normalize heading in state to keep it between -PI and PI
        x.set(2, 0, normalizeAngle(x.get(2, 0)));
    }

    /**
     * Correction Step: Fuse with Vision Data
     * @param visionPose The pose reported by MegaTag
     * @param trustFactor Multiplier for R. 1.0 = standard trust. >1.0 = trust less (far away).
     */
    public void update(Pose2d visionPose, double trustFactor) {
        // Measurement vector z
        SimpleMatrix z = new SimpleMatrix(3, 1, true, new double[]{
                visionPose.position.x,
                visionPose.position.y,
                visionPose.heading.toDouble()
        });

        // Scale R based on how far the tag is (Trust = R * factor)
        SimpleMatrix R_scaled = R.scale(trustFactor);

        // Innovation (Residual): y = z - x
        SimpleMatrix y = z.minus(x);

        // Normalize the heading difference in the residual so the filter doesn't spin 360 deg
        y.set(2, 0, normalizeAngle(y.get(2, 0)));

        // Innovation Covariance: S = P + R
        SimpleMatrix S = P.plus(R_scaled);

        // Kalman Gain: K = P * S^-1
        // (If S is singular, invert() might throw exception, but S=P+R is usually positive definite)
        SimpleMatrix K = P.mult(S.invert());

        // Update State: x = x + K*y
        x = x.plus(K.mult(y));

        // Normalize Heading in State
        x.set(2, 0, normalizeAngle(x.get(2, 0)));

        // Update Covariance: P = (I - K) * P
        SimpleMatrix I = SimpleMatrix.identity(3);
        P = I.minus(K).mult(P);

        // Force Symmetry to prevent floating point drift
        P = P.plus(P.transpose()).scale(0.5);
    }

    public Pose2d getFusedPose() {
        return new Pose2d(x.get(0, 0), x.get(1, 0), x.get(2, 0));
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
