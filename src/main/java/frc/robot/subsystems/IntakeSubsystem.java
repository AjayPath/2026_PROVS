package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Variables;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX intakeLeftMotor;        // Leader
  private final TalonFX intakeRightMotor;   // Follower
  private final VelocityVoltage velocityRequest;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeRightMotor = new TalonFX(51);
    intakeLeftMotor = new TalonFX(30);

    velocityRequest = new VelocityVoltage(0).withSlot(0);
    intakeLeftMotor.getConfigurator().apply(Configs.intakeMotor.intakeConfig);

    // Follower matches leader direction
    intakeRightMotor.setControl(
      new Follower(intakeLeftMotor.getDeviceID(), MotorAlignmentValue.Opposed)
    );
  }

  // --- Getters ---
  public double getLeaderSpeed() {
    return intakeLeftMotor.getVelocity().getValueAsDouble();
  }

  public double getFollowerSpeed() {
    return intakeRightMotor.getVelocity().getValueAsDouble();
  }

  // --- At speed check ---
  public boolean atTargetSpeed() {
    return Math.abs(getLeaderSpeed() - Variables.intake.intakeRPS) < 1;
  }

  @Override
  public void periodic() {
    // Control ONLY the leader
    intakeLeftMotor.setControl(
      velocityRequest.withVelocity(Variables.intake.intakeRPS)
    );
  }
}