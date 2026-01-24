package frc.robot.subsystems.Swerve.SwerveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;

public class LimelightChassisAim extends Command {
    
    private CommandSwerveDrivetrain m_Drivetrain;

    public LimelightChassisAim(CommandSwerveDrivetrain RealDescriptiveName){

        m_Drivetrain  = RealDescriptiveName;
        addRequirements(RealDescriptiveName);

    }

    @Override
    public void initialize(){
        
    }


    @Override
    public void execute(){
        
    }

     @Override
    public void end(boolean interrupted)
    {
        
    }

}
