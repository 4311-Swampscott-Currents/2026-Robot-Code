package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MySingingMotors extends SubsystemBase {
  // 1. Create the orchestra
  Orchestra orchestra = new Orchestra();

  public MySingingMotors(TalonFX... instruments) {
    // 2. Add your motors as instruments
    for (TalonFX motor : instruments) {
      orchestra.addInstrument(motor);
    }
    // 3. Load the music file from the deploy directory
    orchestra.loadMusic("megalovania.chrp");
  }

  public void sing() {
    if (DriverStation.isDisabled()) {
      orchestra.play();
    }
  }

  public void stop() {
    {
      orchestra.stop();
    }
  }
}
