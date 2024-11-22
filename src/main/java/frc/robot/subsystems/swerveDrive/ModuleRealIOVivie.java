package frc.robot.subsystems.swerveDrive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.swerveDrive.ModuleIOVivie.ModuleIOInputs;
import org.littletonrobotics.junction.Logger;

/* import frc.robot.subsystems.swerve.Module.ModuleConstants;
import frc.robot.subsystems.swerve.PhoenixOdometryThread.Registration;
import frc.robot.subsystems.swerve.PhoenixOdometryThread.SignalType; */

public class ModuleRealIOVivie implements ModuleIOVivie {

  private final PositionVoltage turnPID = new PositionVoltage(0.0).withEnableFOC(true).withSlot(0);
  private final MotionMagicVoltage drivePID = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  private final VoltageOut driveVoltage = new VoltageOut(0.0).withEnableFOC(true);
  private final VoltageOut turnVoltage = new VoltageOut(0.0).withEnableFOC(true);

  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;
  private final StatusSignal<Double> driveSupplyCurrent;

  private final StatusSignal<Double> turnAbsolutePosition;
  private final StatusSignal<Double> turnPosition;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnCurrent;

  public ModuleRealIOVivie(int driveId, int turnId, int cancoderID, Rotation2d rotation2d) {
    driveTalon = new TalonFX(driveId, "canivore");
    turnTalon = new TalonFX(turnId, "canivore");
    cancoder = new CANcoder(cancoderID, "canivore");

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();
    driveSupplyCurrent = driveTalon.getSupplyCurrent();

    turnPosition = turnTalon.getPosition();
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();
    turnAbsolutePosition = cancoder.getAbsolutePosition();

    var driveconfig = new TalonFXConfiguration();
    driveconfig.CurrentLimits.SupplyCurrentLimit = 20.0;
    driveconfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    driveconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    driveconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    var turnconfig = new TalonFXConfiguration();
    turnconfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    turnconfig.Feedback.FeedbackRemoteSensorID = cancoderID;
    turnconfig.Feedback.RotorToSensorRatio = 150.0 / 7.0;
    turnconfig.Feedback.SensorToMechanismRatio = 1.0;
    turnconfig.Slot0.kA = 0.0;
    turnconfig.Slot0.kD = 0.0;
    turnconfig.Slot0.kG = 0;
    turnconfig.Slot0.kI = 0;
    turnconfig.Slot0.kP = 10;
    turnconfig.Slot0.kS = 0.0;
    turnconfig.Slot0.kV = 0;
    turnconfig.MotionMagic.MotionMagicCruiseVelocity = 10;
    turnconfig.MotionMagic.MotionMagicAcceleration = 10;
    turnconfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    turnTalon.getConfigurator().apply(turnconfig);

    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = rotation2d.getRotations();
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    cancoder.getConfigurator().apply(cancoderConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        driveSupplyCurrent,
        turnPosition,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
    cancoder.optimizeBusUtilization();
  }

  private double metersToRotations(double meters) {
    /* Get circumference of wheel */
    final double circumference = Constants.wheelRadiusMeters * 2 * Math.PI;
    /* Every rotation of the wheel travels this many meters */
    /* So now get the rotations per meter traveled */
    final double wheelRotationsPerMeter = 1.0 / circumference;
    /* Now apply wheel rotations to input meters */
    double wheelRotations = wheelRotationsPerMeter * meters;
    /* And multiply by gear ratio to get rotor rotations */
    return wheelRotations * Constants.gearRatio;
  }

  @Override
  public void setDriveDistance(final double meters) {
    driveTalon.setControl(drivePID.withPosition(metersToRotations(meters)));
  }

  @Override
  public void setDriveVoltage(final double volts) {
    driveTalon.setControl(driveVoltage.withOutput(volts));
  }

  @Override
  public void setTurnVoltage(final double volts) {
    turnTalon.setControl(turnVoltage.withOutput(volts));
  }

  @Override
  public void setTurnPosition(final Rotation2d posRotation2d) {
    turnTalon.setControl(turnPID.withPosition(posRotation2d.getRotations()));
    Logger.recordOutput("Swerve/" + "set turn position", posRotation2d);
  }

  @Override
  public void updateInputs(final ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        driveSupplyCurrent,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    inputs.drivePositionMeters = drivePosition.getValueAsDouble();
    inputs.driveVelocityMetersPerSec = driveVelocity.getValueAsDouble();
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};
    inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();

    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};
  }
}
