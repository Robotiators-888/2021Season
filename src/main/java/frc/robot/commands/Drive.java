package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;

public class Drive extends CommandBase {
    private static final String Drivetrain = null;
    double left;
    double right;
    
    public Drive() {
        requires(Drivetrain);
    }

    public void initialize() {

    }

    public void execute() {
        left = OI.getLeftSpeed(); 
        right = OI.getRightSpeed();
    }
  
    public boolean isfinished(){
        return false;
    }

    // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

    
    private void requires(String string) {
    }
}