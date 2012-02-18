package Robot.Devices;

import Robot.Utils.BallReadySwitch;
import Robot.Utils.Threading;
import RobotMain.ButtonMapping;
import RobotMain.Constants;
import RobotMain.IODefines;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * The trigger pushes the ball into the shooter.
 */
public class Trigger extends AbstractRobotDevice {

    private Relay triggerMotor = new Relay(IODefines.TRIGGER_MOTOR);
    private final JoystickButton triggerButton = IODefines.TRIGGER_BUTTON;
    private final BallReadySwitch ballReadySwitch;

    public Trigger(BallReadySwitch _ballReadySwitch) {
        ballReadySwitch = _ballReadySwitch;
        triggerMotor.setDirection(Relay.Direction.kForward);
    }

    /**
     * Shoot the ball.
     */
    public void shoot() {
        triggerMotor.set(Relay.Value.kOn);
        Threading.sleep(2 * 1000);
        triggerMotor.set(Relay.Value.kOff);
    }

    public void initialize() {
        Threading.runInLoop(Constants.LimitSwitches.loopTime, new TriggerLoop(), "Trigger");
    }

    private class TriggerLoop implements Runnable {
        public void run() {
            final boolean triggerPressed = triggerButton.get();
            if (triggerPressed && ballReadySwitch.isBallReady()) {
                shoot();
            }
        }
    }
}
