// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.IntakePositionCommand;
import frc.robot.commands.KickerCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ShooterWait;
import frc.robot.commands.ShooterYeetCommandPart2ElectricBoogaloo;
import frc.robot.commands.ShooterYeetCommandPart3ElectricBoogaloo;
import frc.robot.Constants.*;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.SwerveJoystickCmd;

import java.io.IOException;

import javax.lang.model.util.ElementScanner6;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  public final XboxController m_controller = new XboxController(OIConstants.kDriverControllerPort);
  public final XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  private double intakeArmPosition=0;
  double driveDivider = 2.2; //1.5;
  double rotateDivider = 2.2; //1.5;

  GamepadAxisButton m_driverDpadUp;
  GamepadAxisButton m_driverDpadLeft;
  GamepadAxisButton m_driverDpadRight;
  GamepadAxisButton m_driverDpadDown;
  GamepadAxisButton m_operatorDpadUp;
  GamepadAxisButton m_operatorDpadLeft;
  GamepadAxisButton m_operatorDpadRight;
  GamepadAxisButton m_operatorDpadDown;
  GamepadAxisButton m_operatorLeftAxisUp;
  GamepadAxisButton m_operatorLeftAxisDown;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    CommandScheduler.getInstance().registerSubsystem(m_intakeSubsystem);
    CommandScheduler.getInstance().registerSubsystem(m_shooterSubsystem);
    
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> getDriverMoveFwdBack(),
      () -> getDriverMoveLeftRight(),
      () -> getDriverRotate(),
      () -> !m_controller.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

    // Configure the button bindings
    configureButtonBindings();
    m_intakeSubsystem.IntakeSubsystemInit();
  }

  public IntakeSubsystem getIntakeSubsystem()
  {
    return m_intakeSubsystem;
  }

  public ShooterSubsystem getShooterSubsystem()
  {
    return m_shooterSubsystem;
  }

private double getDriverMoveFwdBack()
{
    // Handle creeping forward if the driver is pressing D-pad up
    double pos;
    // Use the joystick axis
    pos = m_controller.getRawAxis(OIConstants.kDriverYAxis) / driveDivider;
    return pos;
}

private double getDriverMoveLeftRight()
{
    double pos;
        pos = m_controller.getRawAxis(OIConstants.kDriverXAxis) / driveDivider;
    return pos;
}

private double getDriverRotate()
{
    double pos;
    pos = m_controller.getRawAxis(OIConstants.kDriverRotAxis) / rotateDivider;
    return pos;
}


/**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(m_controller, XboxController.Button.kStart.value)
      .whenPressed( () -> swerveSubsystem.zeroHeading(0.0) );

    new JoystickButton(m_controller, XboxController.Button.kBack.value)
      .whenPressed( () -> swerveSubsystem.zeroHeading(0.0) );

    //Shooter controls on operator controller
    // center to back bumper  12' 1" = 2500 rpm   limelight ty 0 degrees
    // center to back bumper: 13' 3" = 2500 rpm   limelight ty 4.00 degrees
    // center to back bumper: 15' 1" = 2900 rpm
    m_operatorDpadUp = new GamepadAxisButton(this::operatorDpadUp);
    m_operatorDpadUp
      .whileTrue( new ShooterYeetCommandPart2ElectricBoogaloo( m_shooterSubsystem, 2200)
        .andThen( ()->m_shooterSubsystem.setKickerSpeed(0.0) )
        .andThen( ()->m_shooterSubsystem.setYeetSpeed(0.0) ) 
      );

    m_operatorDpadLeft = new GamepadAxisButton(this::operatorDpadLeft);
    m_operatorDpadLeft
      .whileTrue( new ShooterYeetCommandPart2ElectricBoogaloo( m_shooterSubsystem, Constants.YEET_SPEED_LOW)
        .andThen( ()->m_shooterSubsystem.setKickerSpeed(0.0) )
        .andThen( ()->m_shooterSubsystem.setYeetSpeed(0.0) ) 
      );

    m_operatorDpadRight = new GamepadAxisButton(this::operatorDpadRight);
    m_operatorDpadRight
      .whileTrue( new ShooterYeetCommandPart2ElectricBoogaloo( m_shooterSubsystem, Constants.YEET_SPEED_HIGH)
        .andThen( ()->m_shooterSubsystem.setKickerSpeed(0.0) )
        .andThen( ()->m_shooterSubsystem.setYeetSpeed(0.0) ) 
      );
  
    m_operatorDpadDown = new GamepadAxisButton(this::operatorDpadDown);
    m_operatorDpadDown
      .whileTrue( new ShooterYeetCommandPart2ElectricBoogaloo( m_shooterSubsystem, 2700)
        .andThen( ()->m_shooterSubsystem.setKickerSpeed(0.0) )
        .andThen( ()->m_shooterSubsystem.setYeetSpeed(0.0) ) 
      );
      
    //Kicker controls on drive controller
    m_driverDpadUp = new GamepadAxisButton(this::driverDpadUp);
    m_driverDpadUp
      .whileTrue( new KickerCommand( m_shooterSubsystem, 1.0, false, false, 0 ) );

    m_driverDpadDown = new GamepadAxisButton(this::driverDpadDown);
    m_driverDpadDown
      .whileTrue(new KickerCommand(m_shooterSubsystem, -1.0, false, false, 0));

    m_driverDpadLeft = new GamepadAxisButton(this::driverDpadLeft);
    m_driverDpadLeft
      .whileTrue(new ShooterYeetCommandPart3ElectricBoogaloo( m_shooterSubsystem, -500));

    m_driverDpadRight = new GamepadAxisButton(this::driverDpadRight);
    m_driverDpadRight
      .whileTrue(new ShooterYeetCommandPart3ElectricBoogaloo( m_shooterSubsystem, 500));

    new JoystickButton(m_controller, XboxController.Button.kA.value)
      .whileTrue( KickerMultipleCommand( 0.7 ) );

    m_operatorLeftAxisUp = new GamepadAxisButton(this::operatorLeftAxisUp);
    m_operatorLeftAxisUp
      .onTrue( new IntakePositionCommand( m_intakeSubsystem, Constants.INTAKE_ARM_POSITION_OUT ) );

    m_operatorLeftAxisDown = new GamepadAxisButton(this::operatorLeftAxisDown);
    m_operatorLeftAxisDown
      .onTrue( IntakeRetractCommand() );
  }

  public boolean driverDpadUp()
  {
      return ( m_controller.getPOV() == 0 );
  }

  public boolean driverDpadLeft()
  {
      return ( m_controller.getPOV() == 270 );
  }

  public boolean driverDpadRight()
  {
      return ( m_controller.getPOV() == 90 );
  }

  public boolean driverDpadDown()
  {
      return ( m_controller.getPOV() == 180 );
  }

  public boolean operatorDpadUp()
  {
      return ( m_operatorController.getPOV() == 0 );
  }

  public boolean operatorDpadLeft()
  {
      return ( m_operatorController.getPOV() == 270 );
  }

  public boolean operatorDpadRight()
  {
      return ( m_operatorController.getPOV() == 90 );
  }

  public boolean operatorDpadDown()
  {
      return ( m_operatorController.getPOV() == 180 );
  }

  public boolean operatorLeftAxisUp()
  {
      return ( m_operatorController.getLeftY() < -0.6 );
  }

  public boolean operatorLeftAxisDown()
  {
      return ( m_operatorController.getLeftY() > 0.6 );
  }

  //We build generate this using a function since it's both parameterized, 
  //and reused multiple times in different autos
  public ParallelCommandGroup IntakeRetractCommand(){
    return    
    new ParallelCommandGroup(
        new IntakePositionCommand( m_intakeSubsystem, Constants.INTAKE_ARM_POSITION_IN_FULL ),
        // run the kicker forward slowly until the light curtain is blocked
        new KickerCommand( m_shooterSubsystem, 0.5, true, true, 0 )
          .withTimeout(5.0)
    );
  }

  public SequentialCommandGroup KickerMultipleCommand(double m_output){
    return    
    new SequentialCommandGroup(
      // run the kicker forward slowly until the light curtain is blocked
      new KickerCommand( m_shooterSubsystem, 1.0, true, true, 0),

      // wait for the shooter wheel to be at the right speed
      new ShooterWait( m_shooterSubsystem ),
      new WaitCommand(0.1),

      // run the kicker forwards until the light curtain is not blocked (ball is gone)
      new KickerCommand( m_shooterSubsystem, m_output, true, false, 200 ).withTimeout(2.0),

      // run the kicker forward slowly until the light curtain is blocked
      new KickerCommand( m_shooterSubsystem, 1.0, true, true, 0 ),

      // wait for the shooter wheel to be at the right speed
      new ShooterWait( m_shooterSubsystem ),
      new WaitCommand(0.1),

      // run the kicker forwards until the light curtain is not blocked (ball is gone)
      new KickerCommand( m_shooterSubsystem, m_output, true, false, 200 ).withTimeout(2.0)
    );
  }
  
  public void controlIntake(){
    double yAxis;
    double setSpeed = 0;
    yAxis = m_operatorController.getRightY();
    if( yAxis < -0.1 )
    {
      setSpeed = -0.4 + (yAxis / 3);
    }
    else if( yAxis > 0.1 )
    {
      setSpeed = 0.4 + (yAxis / 3);
    }
    m_intakeSubsystem.setIntakeSpeed( setSpeed );
  }
}
