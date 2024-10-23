package frc.robot.subsystems.intake;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystemNew extends SubsystemBase{
    
    final private CANSparkMax intakeMotorUpperNew;
    final private CANSparkMax intakeMotorLowerNew;
    public IntakeSubsystemNew() {
        intakeMotorUpperNew = new CANSparkMax(Constants.Intake.upperNeoID, CANSparkLowLevel.MotorType.kBrushless);
        intakeMotorLowerNew = new CANSparkMax(Constants.Intake.lowerNeoID, CANSparkLowLevel.MotorType.kBrushless);
        intakeMotorUpperNew.setIdleMode(CANSparkMax.IdleMode.kCoast);
        intakeMotorLowerNew.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public void run(double speedLower, double speedUpper) {
        intakeMotorUpperNew.set(speedUpper);
        intakeMotorLowerNew.set(speedLower);
    }
}
