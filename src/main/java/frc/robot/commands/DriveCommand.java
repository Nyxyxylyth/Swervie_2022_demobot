package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import common.math.Vector2;
import common.robot.input.Axis;

public class DriveCommand extends CommandBase {
    private DrivetrainSubsystem m_drivetrainSubsystem;
    private Axis forward;
    private Axis strafe;
    private Axis rotation;

    public DriveCommand(DrivetrainSubsystem drivetrain, Axis forward, Axis strafe, Axis rotation ) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;

        m_drivetrainSubsystem = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double driveRotation;

        if( m_drivetrainSubsystem.autoAimAngle != 0 )
        {
            driveRotation = Math.toRadians( m_drivetrainSubsystem.autoAimAngle );
        }
        else
        {
            // !*!*!* FIXME: not sure why we have to divide rotate by 10 here...  maybe it should be radians instead of -1..1
            driveRotation = rotation.get(true) / 10.0;
            driveRotation = driveRotation / 5.0;  // slow it down for demo driving
        }
        m_drivetrainSubsystem.drive(new Vector2(forward.get(true)/m_drivetrainSubsystem.joystickDivider,
                                              strafe.get(true)/m_drivetrainSubsystem.joystickDivider),
                                      driveRotation, 
                                      true);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(Vector2.ZERO, 0, false);
    }
}