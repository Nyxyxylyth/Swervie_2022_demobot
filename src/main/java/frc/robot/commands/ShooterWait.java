package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

// This command waits until the shooter is up to speed.

public class ShooterWait extends CommandBase {

    private ShooterSubsystem m_shooterSubsystem;

    public ShooterWait( ShooterSubsystem subsystem )
    {
        m_shooterSubsystem = subsystem;
        addRequirements( m_shooterSubsystem );    
    }

    @Override
  public void initialize() {
<<<<<<< HEAD
    System.out.println(System.currentTimeMillis() + " shooter wait");
=======
   // System.out.println(System.currentTimeMillis() + " shooter wait");
>>>>>>> 73780bf7f470dbb7a75c6328383eb80c7dd4f9ff
  }
    

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      if (m_shooterSubsystem.isAtSpeed()){
<<<<<<< HEAD
          System.out.println(System.currentTimeMillis() + " at speed");
=======
          //System.out.println(System.currentTimeMillis() + " at speed");
>>>>>>> 73780bf7f470dbb7a75c6328383eb80c7dd4f9ff

      }
      return m_shooterSubsystem.isAtSpeed();
      
  }
}
