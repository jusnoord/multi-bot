package frc.robot.util;

import org.ejml.data.SingularMatrixException;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.DemoConstants;

public class WingPoseEstimator {
    private KalmanFilter<N3, N3, N3> wingPoseKF;
    private LinearSystem<N3, N3, N3> plant;
    private Matrix<N3, N1> stateStdDevs;
    private Matrix<N3, N1> measurementStdDevs;
    private final Matrix<N3, N1> ZERO_MATRIX = new Matrix<>(Nat.N3(), Nat.N1());
    private StructPublisher<Pose2d> posePublisher;
    // private StructSubscriber<Pose2d> slavePoseSubscriber;
    private double lastAngle = 0;
    private double lastTime = Timer.getFPGATimestamp();

     
    public WingPoseEstimator() {
        Nat<N3> matrixSize = Nat.N3();
        Matrix<N3, N3> A = new Matrix<>(matrixSize, matrixSize); // state does not propogate as object is stationary (zero velocity)
        Matrix<N3, N3> B = new Matrix<>(matrixSize, matrixSize); // no inputs (e.g. odometry/control)
        Matrix<N3, N3> C = Matrix.eye(matrixSize); // all states are measured directly
        Matrix<N3, N3> D = new Matrix<>(matrixSize, matrixSize); // again, no inputs. also no feedthrough anyway

        plant = new LinearSystem<>(A, B, C, D);

        //dummy values
        stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        measurementStdDevs = VecBuilder.fill(0.02, 0.02, 0.02);

        //construct a linear system plant with my guess of what the states, inputs, and outputs are
        plant = new LinearSystem<N3, N3, N3>(A, B, C, D);

        // intialize kalman filter with 3 states, 3 inputs, and 3 outputs
        // technically no inputs, but we have to put something in
        wingPoseKF = new KalmanFilter<N3, N3, N3>(matrixSize, matrixSize, plant, stateStdDevs, measurementStdDevs, 0.05);

        posePublisher = NetworkTableInstance.getDefault().getTable("WingPoseEstimator").getStructTopic(Constants.currentRobot.toString() + " pose estimate", Pose2d.struct).publish();
        // slavePoseSubscriber = NetworkTableInstance.getDefault().getTable("WingPoseEstimator").getStructTopic("slave pose subscriber", Pose2d.struct).subscribe(new Pose2d());

    }

    public void addVisionMeasurement(Pose2d input) {
        try {
            double x = input.getX();
            double y = input.getY();
            double wrappedTheta = input.getRotation().getRadians();
            
            // unwrap angle so there are no sudden jumps in the kalman input
            double delta = wrappedTheta - lastAngle;
            if (delta > Math.PI) {
                wrappedTheta -= 2 * Math.PI;
            } else if (delta < -Math.PI) {
                wrappedTheta += 2 * Math.PI;
            }

            double theta = lastAngle + (wrappedTheta - lastAngle);
            lastAngle = theta;

            Matrix<N3, N1> measurement = VecBuilder.fill(x, y, theta);
            wingPoseKF.correct(ZERO_MATRIX, measurement); // no control input

        } catch (SingularMatrixException e) {
            System.out.println("[PhotonVision]: WARNING: SingularMatrixException caught in wing pose estimator: " + input.toString());
        }
    }

    public void addVisionMeasurement(Pose2d input, Matrix<N3, N1> measurementStdDevs) {
        double x = input.getX();
        double y = input.getY();
        double wrappedTheta = input.getRotation().getRadians();
        
        // unwrap angle so there are no sudden jumps in the kalman input
        double delta = wrappedTheta - lastAngle;
        if (delta > Math.PI) {
            wrappedTheta -= 2 * Math.PI;
        } else if (delta < -Math.PI) {
            wrappedTheta += 2 * Math.PI;
        }

        double theta = lastAngle + (wrappedTheta - lastAngle);
        lastAngle = theta;
        
        Matrix<N3, N1> measurement = VecBuilder.fill(x, y, theta);

        var covarianceMatrix = StateSpaceUtil.makeCovarianceMatrix(Nat.N3(), measurementStdDevs); // create covariance matrix from std devs
        
        wingPoseKF.correct(ZERO_MATRIX, measurement, covarianceMatrix); // no control input
    }

    public void periodic() {
        try {
            double loopTime = Timer.getFPGATimestamp() - lastTime;
            lastTime = Timer.getFPGATimestamp();
            wingPoseKF.predict(ZERO_MATRIX, loopTime); // no control input

            //grab other robot's wing poses
            // Pose2d[] slaveWingPoses = slavePoseSubscriber.readQueueValues();
            // for(Pose2d pose : slaveWingPoses) {
            //     addVisionMeasurement(pose);
            // }

            //telemetry
            posePublisher.accept(getEstimatedPose());
        } catch (SingularMatrixException e) {
            System.out.println("[PhotonVision]: WARNING: SingularMatrixException caught in wing pose estimator");
        }
    }

    public Pose2d getEstimatedPose() {
        double x = wingPoseKF.getXhat(0);
        double y = wingPoseKF.getXhat(1);
        double theta = wingPoseKF.getXhat(2);

        Pose2d estimatedPose = new Pose2d(x, y, new Rotation2d(theta));

        // if(estimatedPose.equals(Pose2d.kZero)) {
        //     throw new RuntimeException("WingPoseEstimator: estimated pose is zero!");
        // }
        
        return estimatedPose;
    }
}