package frc.robot.subsystems.swerveDrive;

import com.google.common.base.Supplier;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class SwerveDriveSubsystemVivie extends SubsystemBase {

  Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  // Creating my kinematics object using the module locations
  SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  public static ModuleIOVivie[] createTalonFXModules() {
    return new ModuleIOVivie[] {
      new ModuleRealIOVivie(0, 1, 0, Rotation2d.fromRotations(0.377930)),
      new ModuleRealIOVivie(2, 3, 1, Rotation2d.fromRotations(-0.071289)),
      new ModuleRealIOVivie(4, 5, 2, Rotation2d.fromRotations(0.550781)),
      new ModuleRealIOVivie(6, 7, 3, Rotation2d.fromRotations(-0.481689))
    };
  }

  private final ModuleIOVivie[] modules = createTalonFXModules();

  private final ModuleIOInputsAutoLogged[] inputs =
      new ModuleIOInputsAutoLogged[] {
        new ModuleIOInputsAutoLogged(),
        new ModuleIOInputsAutoLogged(),
        new ModuleIOInputsAutoLogged(),
        new ModuleIOInputsAutoLogged()
      };

  public void spin(double rad) {
    Rotation2d r = Rotation2d.fromRadians(rad);
    for (int i = 0; i < 4; i++) {
      modules[i].setTurnPosition(r);
    }
  }

  public void driveMeters(double meters) {
    for (var module : modules) {
      module.setDriveDistance(meters);
    }
  }

  public void setTurnVolts(Double volts) {
    for (var module : modules) {
      module.setTurnVoltage(volts);
      // turnTalon.setControl(turnVoltage.withOutput(volts));
    }
  }

  public void stop(Double volts) {
    for (var module : modules) {
      module.setTurnVoltage(volts);
      module.setDriveVoltage(volts);
    }
  }

  public void turnAndDrive(Double rads, Double driveVolts) {
    Rotation2d r = Rotation2d.fromRadians(rads);
    for (var module : modules) {
      module.setTurnPosition(r);
      module.setDriveVoltage(driveVolts);
    }
  }

  public Command driveChassis(Supplier<ChassisSpeeds> speeds) {
    return this.run(
        () -> {
          SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds.get());
          for (int i = 0; i < 4; i++) {
            modules[i].setTurnPosition(moduleStates[i].angle);
            modules[i].setDriveVoltage(moduleStates[i].speedMetersPerSecond);
          }
        });
  }

  public Command spinCmd(DoubleSupplier rad) {
    return this.run(
        () -> {
          Logger.recordOutput("Swerve/Target Rotation", rad.getAsDouble());
          spin(rad.getAsDouble());
        });
  }

  public Command driveMetersCmd(DoubleSupplier meters) {
    return this.run(() -> driveMeters(meters.getAsDouble()));
  }

  public Command setTurnVoltsCmd(DoubleSupplier volts) {
    return this.run(() -> setTurnVolts(volts.getAsDouble()));
  }

  public Command stopCmd(DoubleSupplier volts) {
    return this.run(() -> stop(volts.getAsDouble()));
  }

  public Command turnAndDriveCmd(DoubleSupplier turnVolts, DoubleSupplier driveVolts) {
    return this.run(() -> turnAndDrive(turnVolts.getAsDouble(), driveVolts.getAsDouble()));
  }

  public Command exampleMethodCommand2() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  @Override
  public void periodic() {
    for (int i = 0; i < 4; i++) {
      modules[i].updateInputs(inputs[i]);
      Logger.processInputs("swerve" + i, inputs[i]);
    }
  }
}
