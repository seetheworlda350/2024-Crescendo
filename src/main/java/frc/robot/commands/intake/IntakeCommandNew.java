package frc.robot.commands.intake;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.sensors.LaserCANSensor;
import frc.robot.subsystems.intake.IntakeSubsystemNew;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class IntakeCommandNew extends SubsystemBase {
	private final IntakeSubsystemNew intake;
	private final ShooterSubsystem shooter;

    public IntakeCommandNew(IntakeSubsystemNew intake, ShooterSubsystem shooter, LaserCANSensor intakeLaser, LaserCANSensor shooterLaser) {
        this.intake = intake;
		this.shooter = shooter;
    }
    public void execute() {
        intake.run(Constants.Intake.lowerIntakeSpeed, Constants.Intake.upperIntakeSpeed);
    }

	public boolean isFinished() {
		
            return false;
		
    }
    public void end(boolean interrupted) {
        intake.run(0, 0);
    }
}