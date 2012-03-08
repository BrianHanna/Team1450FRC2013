package Robot.Commands.Shooter;

import Robot.Commands.CommandBase;

public class RampUpShooterForPowerCommand extends CommandBase {

    private final double power;

    public RampUpShooterForPowerCommand(double power) {
        this.power = power;
        setTimeout(2);
    }

    protected void initialize() {
    }

    protected void execute() {
        shooter.throttle(power);
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {
    }

    protected void interrupted() {
    }
}