package frc.robot.subsystems.swerveDrive;

/* import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog; */

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIOVivie {

  @AutoLog
  public static class ModuleIOInputs {
    public String prefix = "";

    public double drivePositionMeters = 0.0;
    public double driveVelocityMetersPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double[] driveCurrentAmps = new double[] {};
    public double driveSupplyCurrentAmps = 0.0;

    public Rotation2d turnAbsolutePosition = new Rotation2d();
    public Rotation2d turnPosition = new Rotation2d();
    public Rotation2d currentTurnPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double[] turnCurrentAmps = new double[] {};
  }

  public void updateInputs(final ModuleIOInputs inputs);

  public void setDriveVoltage(double volts);

  public void setTurnVoltage(double volts);

  public void setTurnPosition(Rotation2d posRotation2d);

  public void setDriveDistance(double meters);
}
