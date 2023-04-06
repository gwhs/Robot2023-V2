package frc.robot.swerve;

import static frc.robot.Constants.DrivetrainConstants.DRIVE_kA;
import static frc.robot.Constants.DrivetrainConstants.DRIVE_kD;
import static frc.robot.Constants.DrivetrainConstants.DRIVE_kI;
import static frc.robot.Constants.DrivetrainConstants.DRIVE_kP;
import static frc.robot.Constants.DrivetrainConstants.DRIVE_kS;
import static frc.robot.Constants.DrivetrainConstants.DRIVE_kV;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import org.littletonrobotics.junction.Logger;

public class SwerveSpeedController {

  private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;
  private static final int CAN_TIMEOUT_MS = 250;
  private static final double TICKS_PER_ROTATION = 2048.0;

  private final WPI_TalonFX motor;

  private final double sensorPositionCoefficient;
  private final double sensorVelocityCoefficient;
  private final double nominalVoltage = 12.0;

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(DRIVE_kS, DRIVE_kV, DRIVE_kA);

  private double referenceVelocity;

  public SwerveSpeedController(
      int port,
      ModuleConfiguration moduleConfiguration,
      ShuffleboardContainer container,
      String canivoreName) {
    sensorPositionCoefficient =
        Math.PI
            * moduleConfiguration.getWheelDiameter()
            * moduleConfiguration.getDriveReduction()
            / TICKS_PER_ROTATION;
    sensorVelocityCoefficient = sensorPositionCoefficient * 10.0;

    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

    motorConfiguration.voltageCompSaturation = 12;
    motorConfiguration.supplyCurrLimit.currentLimit = 80;
    motorConfiguration.supplyCurrLimit.enable = true;

    motorConfiguration.slot0.kP = DRIVE_kP;
    motorConfiguration.slot0.kI = DRIVE_kI;
    motorConfiguration.slot0.kD = DRIVE_kD;

    motor = new WPI_TalonFX(port, canivoreName);
    CtreUtils.checkCtreError(
        motor.configAllSettings(motorConfiguration), "Failed to configure Falcon 500");
    motor.enableVoltageCompensation(true);
    motor.setNeutralMode(NeutralMode.Coast);
    motor.setInverted(moduleConfiguration.isDriveInverted());
    motor.setSensorPhase(true);
    motor.setSafetyEnabled(true);

    // Reduce CAN status frame rates
    /**
     * Set the CAN status frames.
     *
     * @param CANStatus1 Applied Motor Output, Fault Information, Limit Switch Information
     * @param CANStatus2 Selected Sensor Position (PID 0), Selected Sensor Velocity (PID 0), Brushed
     *     Supply Current Measurement, Sticky Fault Information
     * @param CANStatus3 Quadrature Information
     * @param CANStatus4 Analog Input, Supply Battery Voltage, Controller Temperature
     * @param CANStatus8 Pulse Width Information
     * @param CANStatus10 Motion Profiling/Motion Magic Information
     * @param CANStatus12 Selected Sensor Position (Aux PID 1), Selected Sensor Velocity (Aux PID 1)
     * @param CANStatus13 PID0 (Primary PID) Information
     * @param CANStatus14 PID1 (Auxiliary PID) Information
     * @param CANStatus21 Integrated Sensor Position (Talon FX), Integrated Sensor Velocity (Talon
     *     FX)
     * @param CANStatusCurrent Brushless Supply Current Measurement, Brushless Stator Current
     *     Measurement
     */
    CtreUtils.checkCtreError(
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20, 1000),
        "Failed to configure Falcon status frame period");

    CtreUtils.checkCtreError(
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 15, 1000),
        "Failed to configure Falcon status frame period");

    CtreUtils.checkCtreError(
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 211, 1000),
        "Failed to configure Falcon status frame period");

    CtreUtils.checkCtreError(
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 251, 1000),
        "Failed to configure Falcon status frame period");

    addDashboardEntries(container);
  }

  private void addDashboardEntries(ShuffleboardContainer container) {
    if (container != null) {
      container.addNumber("Current Velocity", () -> getStateVelocity());
      container.addNumber("Target Velocity", () -> referenceVelocity);
      container.addNumber("Current Position", () -> getStatePosition());
    }
  }

  public void setReferenceVelocity(double velocity) {
    this.referenceVelocity = velocity;
    var arbFeedForward = feedforward.calculate(velocity) / nominalVoltage;
    motor.set(
        TalonFXControlMode.Velocity,
        velocity / sensorVelocityCoefficient,
        DemandType.ArbitraryFeedForward,
        arbFeedForward);

    Logger.getInstance()
        .recordOutput("Drive_Motor" + motor.getDeviceID() + "/SupplyCurrent", getSupplyCurrent());
    Logger.getInstance()
        .recordOutput("Drive_Motor" + motor.getDeviceID() + "/StatorCurrent", getStatorCurrent());
    Logger.getInstance()
        .recordOutput(
            "Drive_Motor" + motor.getDeviceID() + "/MotorOutputPercent", getMotorOutputPercent());
    Logger.getInstance()
        .recordOutput("Drive_Motor" + motor.getDeviceID() + "/Temperature", getTemperature());

    motor.feed();
  }

  /**
   * Returns velocity in meters per second
   *
   * @return drive velocity in meters per second
   */
  public double getStateVelocity() {
    return motor.getSelectedSensorVelocity() * sensorVelocityCoefficient;
  }

  public double getSupplyCurrent() {
    return motor.getSupplyCurrent();
  }

  public double getStatorCurrent() {
    return motor.getStatorCurrent();
  }

  public double getMotorOutputPercent() {
    return motor.getMotorOutputPercent();
  }

  public double getTemperature() {
    return motor.getTemperature();
  }

  /**
   * Returns position in meters
   *
   * @return position in meters
   */
  public double getStatePosition() {
    return motor.getSelectedSensorPosition() * sensorPositionCoefficient;
  }

  /**
   * Sets the neutral mode for the drive motor
   *
   * @param neutralMode neutral mode
   */
  public void setNeutralMode(NeutralMode neutralMode) {
    motor.setNeutralMode(neutralMode);
  }
}
