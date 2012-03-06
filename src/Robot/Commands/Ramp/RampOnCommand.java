package Robot.Commands.Ramp;

import Robot.Commands.CommandBase;

/**
 */
public class RampOnCommand extends CommandBase {

    public RampOnCommand() {
        requires(ramp);
    }

    protected void initialize() {
    }

    protected void execute() {
        ramp.on();
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {
    }

    protected void interrupted() {
    }
}
