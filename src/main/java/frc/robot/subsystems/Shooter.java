package frc.robot.subsystems;

import java.util.function.Function;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class Shooter extends SubsystemBase{
    
  public static TalonFX m_rightShooter = new TalonFX(Constants.MotorIDs.RIGHT_SHOOTER_MOTOR_ID);
  public static TalonFX m_leftShooter = new TalonFX(Constants.MotorIDs.LEFT_SHOOTER_MOTOR_ID);
    
    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

    Slot0Configs slot0Configs = talonFXConfig.Slot0;

    
    private void shootLeft() {
        m_leftShooter.set(1);
    }
    private void shootRight() {
        m_rightShooter.set(-1);
    }
    private void shootAll() {
        shootLeft();
        shootRight();
    }
    public Command shootCommand() {
         return this.run(() -> shootAll()  );
    }


    

    



}
