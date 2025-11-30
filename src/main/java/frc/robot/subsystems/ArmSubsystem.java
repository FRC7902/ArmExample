// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import static java.lang.Math.abs;

public class ArmSubsystem extends SubsystemBase {
  /**
   * The TalonFX Kraken x60 motor for the arm
   */
  private final TalonFX m_armMotor = new TalonFX(ArmConstants.ARM_MOTOR_CAN_ID);

  /**
   * The simulation state object for the arm motor
   */
  private final TalonFXSimState m_armMotorSim = m_armMotor.getSimState();

  /**
   * The MotionMagicVoltage request object to control the arm motor
   */
  private final MotionMagicVoltage m_armMotorControl = new MotionMagicVoltage(0);

  /**
   * The TalonFX configuration object
   */
  private final TalonFXConfiguration m_armMotorConfig = new TalonFXConfiguration();

  /**
   * The Mechanism2d instance to simulate the arm
   */
  private final Mechanism2d m_mech2d = new Mechanism2d(
          50,
          50,
          new Color8Bit(Color.kDarkRed)
  );

  /**
   * The root of the arm ligament; the pivot point of the arm
   */
  private final MechanismRoot2d m_root = m_mech2d.getRoot("ArmRoot", 25, 5);

  /**
   * The ligament to portray the actual arm
   */
  private final MechanismLigament2d m_ligament = m_root.append(
          new MechanismLigament2d("Arm", Units.metersToInches(ArmConstants.ARM_LENGTH_METERS), 0));

  /**
   * The physics simulation instance of the arm
   */
  private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          ArmConstants.ARM_GEARING,
          SingleJointedArmSim.estimateMOI(ArmConstants.ARM_LENGTH_METERS, ArmConstants.ARM_MASS_KG),
          ArmConstants.ARM_LENGTH_METERS,
          Units.degreesToRadians(ArmConstants.ARM_MIN_ANGLE_DEGREES),
          Units.degreesToRadians(ArmConstants.ARM_MAX_ANGLE_DEGREES),
          true,
          0
          );

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    // Configure the motor's Slot0 control config
    Slot0Configs configs = m_armMotorConfig.Slot0;
    // Set PID
    configs.kP = ArmConstants.ARM_KP;
    configs.kI = ArmConstants.ARM_KI;
    configs.kD = ArmConstants.ARM_KD;
    configs.GravityType = GravityTypeValue.Arm_Cosine;

    // Current limits ensure that the motor does not take too much power to preserve it and avoid brownout
    CurrentLimitsConfigs currentLimits = m_armMotorConfig.CurrentLimits;
    currentLimits.StatorCurrentLimit = ArmConstants.STATOR_CURRENT_LIMIT;
    currentLimits.SupplyCurrentLimit = ArmConstants.SUPPLY_CURRENT_LIMIT;

    // Set MotionMagic control parameters
    m_armMotorConfig.MotionMagic.MotionMagicAcceleration = 20;
    m_armMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 20;

    // Gear ratio for the mechanism
    m_armMotorConfig.Feedback.SensorToMechanismRatio = ArmConstants.ARM_GEARING;

    // The system will use the motor's internal encoder as a reference for PID
    m_armMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    // When the motor is unpowered, oppose external movement
    m_armMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Invert the direction of the motor
    m_armMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    m_armMotor.getConfigurator().apply(m_armMotorConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("arm/info/Position (rotations)", getArmPositionDegrees() / 360);
    SmartDashboard.putNumber("arm/info/Setpoint (rotations)", getArmSetpointDegrees() / 360);

    SmartDashboard.putNumber("arm/info/Position (Degrees)", getArmPositionDegrees());
    SmartDashboard.putNumber("arm/info/Setpoint (Degrees)", getArmSetpointDegrees());

    SmartDashboard.putBoolean("arm/info/Reached Angle", hasReachedAngle());
    SmartDashboard.putData("arm/Mechanism2d", m_mech2d);
  }

  @Override
  public void simulationPeriodic() {
    // Set supply voltage
    m_armMotorSim.setSupplyVoltage(12);

    // Feed motor voltage into WPILib physics sim
    m_armSim.setInput(m_armMotorSim.getMotorVoltage());
    m_armSim.update(0.02);

    // Set the rotor position and velocity using the physics simulation
    double rotorPosition = (m_armSim.getAngleRads() / (2.0 * Math.PI)) * ArmConstants.ARM_GEARING;
    double rotorVelocity = (m_armSim.getVelocityRadPerSec() / (2.0 * Math.PI)) * ArmConstants.ARM_GEARING;
    m_armMotorSim.setRawRotorPosition(rotorPosition);
    m_armMotorSim.setRotorVelocity(rotorVelocity);

    // Set the ligament angle to the physics simulation angle
    m_ligament.setAngle(m_armSim.getAngleRads() * (180 / Math.PI));
  }

  /**
   * Return the position of the arm motor in degrees
   * @return the position of the arm motor in degrees
   */
  public double getArmPositionDegrees() {
    return m_armMotor.getPosition().getValueAsDouble()*360;
  }

  /**
   * Return the setpoint of the arm motor in degrees
   * @return the setpoint of the arm motor in degrees
   */
  public double getArmSetpointDegrees() {
    return m_armMotor.getClosedLoopReference().getValueAsDouble()*360;
  }

  /**
   * Return if the arm has reached its target with a desirable level of accuracy
   */
  public boolean hasReachedAngle() {
    return (abs(getArmPositionDegrees() - getArmSetpointDegrees()) < ArmConstants.ARM_TARGET_TOLERANCE);
  }

  /**
   * Sets the arm position in degrees, the arm will attempt to move to this position
   * @param degrees the target in degrees
   */
  public void setArmPositionDegrees(double degrees) {
    // Ensure position is within bounds
    if (degrees < ArmConstants.ARM_MIN_ANGLE_DEGREES) {
      degrees = ArmConstants.ARM_MIN_ANGLE_DEGREES;
    } else if (degrees > ArmConstants.ARM_MAX_ANGLE_DEGREES) {
      degrees = ArmConstants.ARM_MAX_ANGLE_DEGREES;
    }

    // Set motor control
    m_armMotorControl.Position = (degrees/360);
    m_armMotorControl.Slot = 0;
    m_armMotor.setControl(m_armMotorControl);
  }
}
