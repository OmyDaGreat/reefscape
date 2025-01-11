// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotParameters;
import frc.robot.utils.RobotParameters.MotorParameters;

public class CoralManipulator extends SubsystemBase {
  private TalonFX coralManipulatorMotorUp;
  private TalonFX coralManipulatorMotorDown;

  private TalonFXConfigurator coralManipulatorUpConfigurator;
  private TalonFXConfigurator coralManipulatorDownConfigurator;

  private TalonFXConfiguration coralManipulatorUpConfiguration;
  private TalonFXConfiguration coralManipulatorDownConfiguration;

  private Slot0Configs coralManipulatorUpConfigs;
  private Slot0Configs coralManipulatorDownConfigs;

  private PositionTorqueCurrentFOC pos_reqest;
  private VelocityTorqueCurrentFOC vel_voltage;

  private MotorOutputConfigs coralManipulatorConfigs;

  private CurrentLimitsConfigs upMotorCurrentConfig;
  private CurrentLimitsConfigs downMotorCurrentConfig;

  private ClosedLoopRampsConfigs upMotorRampConfig;
  private ClosedLoopRampsConfigs downMotorRampConfig;

  private SoftwareLimitSwitchConfigs upSoftLimitConfig;
  private SoftwareLimitSwitchConfigs downSoftLimitConfig;

  private VoltageOut voltageOut;

  private double deadband = 0.001;

  /**
   * The Singleton instance of this CoralManipulatorSubsystem. Code should use the {@link #getInstance()}
   * method to get the single instance (rather than trying to construct an instance of this class.)
   */
  private static final CoralManipulator INSTANCE = new CoralManipulator();

  /**
   * Returns the Singleton instance of this CoralManipulatorSubsystem. This static method should be used,
   * rather than the constructor, to get the single instance of this class. For example: {@code
   * CoralManipulatorSubsystem.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static CoralManipulator getInstance() {
    return INSTANCE;
  }

  /**
   * Creates a new instance of this CoralManipulatorSubsystem. This constructor is private since this class
   * is a Singleton. Code should use the {@link #getInstance()} method to get the singleton
   * instance.
   */
  private CoralManipulator() {
    coralManipulatorMotorUp = new TalonFX(MotorParameters.CORAL_MANIPULATOR_MOTOR_UP_ID);
    coralManipulatorMotorDown = new TalonFX(MotorParameters.CORAL_MANIPULATOR_MOTOR_DOWN_ID);

    coralManipulatorConfigs = new MotorOutputConfigs();

    coralManipulatorUpConfigurator = coralManipulatorMotorUp.getConfigurator();
    coralManipulatorDownConfigurator = coralManipulatorMotorDown.getConfigurator();

    coralManipulatorUpConfigs = new Slot0Configs();
    coralManipulatorDownConfigs = new Slot0Configs();

    coralManipulatorUpConfiguration = new TalonFXConfiguration();
    coralManipulatorDownConfiguration = new TalonFXConfiguration();

    coralManipulatorMotorUp.getConfigurator().apply(coralManipulatorUpConfiguration);
    coralManipulatorMotorDown.getConfigurator().apply(coralManipulatorDownConfiguration);

    coralManipulatorConfigs.NeutralMode = NeutralModeValue.Brake;
    coralManipulatorUpConfigurator.apply(coralManipulatorConfigs);
    coralManipulatorDownConfigurator.apply(coralManipulatorConfigs);

    coralManipulatorUpConfigs.kP = RobotParameters.CoralManipulatorParameters.CORAL_MANIPULATOR_UP_PID_P;
    coralManipulatorUpConfigs.kI = RobotParameters.CoralManipulatorParameters.CORAL_MANIPULATOR_UP_PID_I;
    coralManipulatorUpConfigs.kD = RobotParameters.CoralManipulatorParameters.CORAL_MANIPULATOR_UP_PID_D;
    coralManipulatorUpConfigs.kV = RobotParameters.CoralManipulatorParameters.CORAL_MANIPULATOR_UP_PID_V;

    coralManipulatorDownConfigs.kP = RobotParameters.CoralManipulatorParameters.CORAL_MANIPULATOR_DOWN_PID_P;
    coralManipulatorDownConfigs.kI = RobotParameters.CoralManipulatorParameters.CORAL_MANIPULATOR_DOWN_PID_I;
    coralManipulatorDownConfigs.kD = RobotParameters.CoralManipulatorParameters.CORAL_MANIPULATOR_DOWN_PID_D;
    coralManipulatorDownConfigs.kV = RobotParameters.CoralManipulatorParameters.CORAL_MANIPULATOR_DOWN_PID_V;

    coralManipulatorMotorUp.getConfigurator().apply(coralManipulatorUpConfigs);
    coralManipulatorMotorDown.getConfigurator().apply(coralManipulatorDownConfigs);

    upMotorCurrentConfig = new CurrentLimitsConfigs();
    downMotorCurrentConfig = new CurrentLimitsConfigs();

    upMotorRampConfig = new ClosedLoopRampsConfigs();
    downMotorRampConfig = new ClosedLoopRampsConfigs();

    upMotorCurrentConfig.SupplyCurrentLimit = 100;
    upMotorCurrentConfig.SupplyCurrentLimitEnable = true;
    upMotorCurrentConfig.StatorCurrentLimit = 100;
    upMotorCurrentConfig.StatorCurrentLimitEnable = true;

    downMotorCurrentConfig.SupplyCurrentLimit = 100;
    downMotorCurrentConfig.SupplyCurrentLimitEnable = true;
    downMotorCurrentConfig.StatorCurrentLimit = 100;
    downMotorCurrentConfig.StatorCurrentLimitEnable = true;

    coralManipulatorMotorUp.getConfigurator().apply(upMotorCurrentConfig);
    coralManipulatorMotorDown.getConfigurator().apply(downMotorCurrentConfig);

    upMotorRampConfig.DutyCycleClosedLoopRampPeriod = 0.1;
    downMotorRampConfig.DutyCycleClosedLoopRampPeriod = 0.1;

    coralManipulatorMotorUp.getConfigurator().apply(upMotorRampConfig);
    coralManipulatorMotorDown.getConfigurator().apply(downMotorRampConfig);

    // on
    coralManipulatorUpConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    coralManipulatorDownConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // absoluteEncoder = new DigitalInput(9);

    vel_voltage = new VelocityTorqueCurrentFOC(0);
    pos_reqest = new PositionTorqueCurrentFOC(0);
    voltageOut = new VoltageOut(0);

    new PositionDutyCycle(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** Stops the coral manipulator motors */
  public void stopMotors() {
    coralManipulatorMotorUp.stopMotor();
    coralManipulatorMotorDown.stopMotor();
    voltageOut.Output = -0.014;
    coralManipulatorMotorUp.setControl(voltageOut);
    coralManipulatorMotorDown.setControl(voltageOut);
  }

  /** Starts the coral manipulator motors */
  public void startMotors() {
    voltageOut.Output = 0.014;
    coralManipulatorMotorUp.setControl(voltageOut);
    coralManipulatorMotorDown.setControl(voltageOut);
  }
}