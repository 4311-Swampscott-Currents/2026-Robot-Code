// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

 

  

  public static final class MotorIDs{
    //motor ids
    public static final int RIGHT_SHOOTER_MOTOR_ID = 15;
    public static final int LEFT_SHOOTER_MOTOR_ID = 16;
    public static final int KICKER_MOTOR_ID = 17;
    public static final int PROCESSOR_MOTOR_ID = 18;
    public static final int INTAKE_ARM_MOTOR_ID = 19;
    public static final int INTAKE_WHEELs_MOTOR_ID = 20;
    public static final int CLIMBER_MOTOR_ID = 21;
    


  }
  
  
  
  
  
  

}
