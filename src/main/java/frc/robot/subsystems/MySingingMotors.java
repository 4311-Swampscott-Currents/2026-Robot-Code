package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;

public class MySingingMotors extends SubsystemBase {
  // 1. Create the orchestra
  private Orchestra orchestra = new Orchestra();

  public MySingingMotors(TalonFX... instruments) {
    // 2. Add your motors as instruments
    for (TalonFX motor : instruments) {
      orchestra.addInstrument(motor);
    }
    // 3. Load the music file from the deploy directory

    orchestra.loadMusic("pirate.chrp");
  }
  /**
   * Sets up and loads music into the Orchestra object by using a playlist. Currently avaiable
   * songs: - "evangelion" - "pirate" - "megalovania"
   */
  public void setupMusic(String... songs) {
    HashMap<Integer, String> playlist = new HashMap<>();
    int iter = 0;
    for (String song : songs) {
      playlist.put(iter, song.concat(".chrp")); // initializes and adds songs to the playlist
      iter++;
    }

    for (iter = 0; iter < playlist.size(); iter++) {
      if (orchestra.getCurrentTime() == 0) {
        orchestra.loadMusic(playlist.get(iter)); // plays the song in the playlist
      }
    }
  }

  public void sing() {
    orchestra.play();
    // if (DriverStation.isDisabled()) {
    //   orchestra.play();
    // }
  }

  public void stop() {
    {
      orchestra.stop();
    }
  }
}
