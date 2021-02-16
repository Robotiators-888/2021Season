package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.OI;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Intake {

    protected DoubleSolenoid intakeFlipper;
    protected TalonSRX intakeMotor;

    protected int multi;
    protected OI oi;

    protected boolean flipButtonLast = false;
    protected boolean flipButton = false;

    private boolean isRunning = false;

    public Intake(OI oi) {
        intakeFlipper = new DoubleSolenoid(Constants.PCM,
                Constants.DS_FORWARD_CHANNEL, Constants.DS_REVERSE_CHANNEL);
        intakeMotor = new TalonSRX(Constants.INTAKE_CANID);

        intakeMotor.setInverted(true);

        this.oi = oi;
    }

    public void intakeTeleopPeriodic() {
        if (oi.getLeftStickButton(Constants.JOYSTICK_TRIGGER)) {
            if (oi.getLeftStickButton(intakeMotor.JOYSTICK_LEFT_BUTTON)){
                intakeReverse();
            }
            else {
                intakeIn();
            }
        }
        else {
            intakeStop();
        }

        flipButton = oi.getLeftStickButton(Constants.JOYSTICK_CENTER_BUTTON);

        flipIntake(flipButton && !flipButtonLast);

        flipButtonLast = flipButton;

    }

    public void flipIntake(boolean flip) {
        if (flip) {

            if (intakeFlipper.get().equals(DoubleSolenoid.Value.kReverse)) {
                intakeFlipper.set(DoubleSolenoid.Value.kForward);
            }

            else {
                intakeFlipper.set(DoubleSolenoid.Value.kReverse);
            }
        }
    }

    public void intakeIn() {
        intakeMotor.set(ControlMode.PercentOutput, 0.75);
        isRunning = true;
    }

    public void intakeReverse() {
        intakeMotor.set(ControlMode.PercentOutput, -0.75);
        isRunning = true;
    }

    public void intakeStop() {
        intakeMotor.set(ControlMode.PercentOutput, 0);
        isRunning = false;
    }

    public void flipDown() {
        if (intakeFlipper.get().equals(DoubleSolenoid.Value.kReverse))
            intakeFlipper.set(DoubleSolenoid.Value.kForward);
    }

    public void flipUp() {
        if (intakeFlipper.get().equals(DoubleSolenoid.Value.kForward))
            intakeFlipper.set(DoubleSolenoid.Value.kReverse);
    }

    public boolean isRunning() {
        return isRunning;
    }

}