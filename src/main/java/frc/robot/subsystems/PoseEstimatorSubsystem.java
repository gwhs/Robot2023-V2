package frc.robot.subsystems;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.FieldConstants;
import java.util.ArrayList;
import java.util.Map;

public class PoseEstimatorSubsystem extends SubsystemBase {

  private final DrivetrainSubsystem drivetrainSubsystem;

  // Ordered list of target poses by ID (WPILib is adding some functionality for
  // this)
  private static final Map<Integer, Pose3d> targetPoses = FieldConstants.aprilTags;

  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others.
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /**
   * Standard deviations of model states. Increase these numbers to trust your model's state
   * estimates less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians,
   * then meters.
   */
  private static final Vector<N3> stateStdDevs =
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global
   * measurements from vision less. This matrix is in the form [x, y, theta]ᵀ, with units in meters
   * and radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs =
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  private final SwerveDrivePoseEstimator poseEstimator;

  private final ArrayList<Double> xValues = new ArrayList<Double>();
  private final ArrayList<Double> yValues = new ArrayList<Double>();

  private final Field2d field2d = new Field2d();

  private double previousPipelineTimestamp = 0;

  public PoseEstimatorSubsystem(DrivetrainSubsystem drivetrainSubsystem) {

    this.drivetrainSubsystem = drivetrainSubsystem;

    ShuffleboardTab tab = Shuffleboard.getTab("Vision");

    poseEstimator =
        new SwerveDrivePoseEstimator(
            DrivetrainConstants.KINEMATICS,
            drivetrainSubsystem.getGyroscopeRotation(),
            drivetrainSubsystem.getModulePositions(),
            new Pose2d(),
            stateStdDevs,
            visionMeasurementStdDevs);

    tab.addString("Pose", this::getFormattedPose).withPosition(0, 0).withSize(2, 0);
    tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
  }

  public void addWTrajectory(Trajectory traj) {
    field2d.getObject("Trajectory").setTrajectory(traj);
  }

  public void addTrajectory(PathPlannerTrajectory traj) {
    field2d.getObject("Trajectory").setTrajectory(traj);
  }

  @Override
  public void periodic() {
    // Update pose estimator with the best visible target

    // Update pose estimator with drivetrain sensors
    poseEstimator.update(
        drivetrainSubsystem.getGyroscopeRotation(), drivetrainSubsystem.getModulePositions());

    // field2d.setRobotPose(getCurrentPose());
    // Conversion so robot appears where it actually is on field instead of always
    // on blue.
    // xValues.add(getCurrentPose().getX());
    // yValues.add(getCurrentPose().getY());
    // double xAverage = xValues.stream().mapToDouble(a ->
    // a).average().getAsDouble();
    // double yAverage = yValues.stream().mapToDouble(a ->
    // a).average().getAsDouble();
    // double summation = 0.0;
    // for (int i = 0; i < xValues.size(); i++) {
    // summation += (Math.pow(xValues.get(i) - xAverage, 2) +
    // Math.pow(yValues.get(i) - yAverage, 2));
    // }
    // double RMS = Math.sqrt((1.0 / (double) xValues.size() * summation));
    // System.out.println("RMS: " + RMS);
    if (DriverStation.getAlliance() == Alliance.Red) {
      field2d.setRobotPose(
          new Pose2d(
              FieldConstants.fieldLength - getCurrentPose().getX(),
              FieldConstants.fieldWidth - getCurrentPose().getY(),
              new Rotation2d(getCurrentPose().getRotation().getRadians() + Math.PI)));
    } else {
      field2d.setRobotPose(getCurrentPose());
    }
  }

  private String getFormattedPose() {
    var pose = getCurrentPose();
    return String.format(
        "(%.2f, %.2f) %.2f degrees", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called when the robot's
   * position on the field is known, like at the beginning of a match.
   *
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(
        drivetrainSubsystem.getGyroscopeRotation(),
        drivetrainSubsystem.getModulePositions(),
        newPose);
  }

  public double getAngle() {
    Pose2d pose = getCurrentPose();
    return pose.getRotation().getDegrees();
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  public void initializeGyro(double angleDegree) {
    drivetrainSubsystem.setGyroscopeRotation(angleDegree);
    poseEstimator.resetPosition(
        drivetrainSubsystem.getGyroscopeRotation(),
        drivetrainSubsystem.getModulePositions(),
        new Pose2d(getCurrentPose().getX(), getCurrentPose().getY(), Rotation2d.fromDegrees(0)));
  }

  /**
   * Resets the holonomic rotation of the robot (gyro last year) This would be used if Apriltags are
   * not getting accurate pose estimation
   */
  public void resetHolonomicRotation() {
    poseEstimator.resetPosition(
        Rotation2d.fromDegrees(0), drivetrainSubsystem.getModulePositions(), getCurrentPose());
  }

  public void resetPoseRating() {
    xValues.clear();
    yValues.clear();
  }
}
