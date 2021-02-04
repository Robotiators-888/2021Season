package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Interface for the drive train.
 */
public class DriveTrain {

    // CAN bus IDs for the motor controllers for the drive train
    private static final int MOTOR_FRONT_LEFT = 10;
    private static final int MOTOR_FRONT_RIGHT = 12;
    private static final int MOTOR_REAR_LEFT = 11;
    private static final int MOTOR_REAR_RIGHT = 13;

    // IDs for pneumatic controls
    public static final double RAMP_RATE = 0.2;

    Navigation nav;

    // Motors
    CANSparkMax frontLeft;
    CANSparkMax rearLeft;
    CANSparkMax frontRight;
    CANSparkMax rearRight;

    CANEncoder leftEncoder;
    CANEncoder rightEncoder;

    double rightEncoderOffset;
    double leftEnccoderOffset;

    boolean brake = false;

    /**
     * Constructor for a drive object.
     */
    public DriveTrain() {

        // instantiate motor controllers with CAN ID and motor type
        frontLeft = new CANSparkMax(MOTOR_FRONT_LEFT, CANSparkMaxLowLevel.MotorType.kBrushless);
        rearLeft = new CANSparkMax(MOTOR_REAR_LEFT, CANSparkMaxLowLevel.MotorType.kBrushless);
        frontRight = new CANSparkMax(MOTOR_FRONT_RIGHT, CANSparkMaxLowLevel.MotorType.kBrushless);
        rearRight = new CANSparkMax(MOTOR_REAR_RIGHT, CANSparkMaxLowLevel.MotorType.kBrushless);

        leftEncoder = frontLeft.getEncoder();
        rightEncoder = frontRight.getEncoder();

        // make rear motors follow front motors
        rearLeft.follow(frontLeft);
        rearRight.follow(frontRight);

        // invert right side
        frontLeft.setInverted(false);
        frontRight.setInverted(true);

        // set motors to coast mode
        frontLeft.setIdleMode(IdleMode.kCoast);
        rearLeft.setIdleMode(IdleMode.kCoast);
        frontRight.setIdleMode(IdleMode.kCoast);
        rearRight.setIdleMode(IdleMode.kCoast);

        frontLeft.setClosedLoopRampRate(RAMP_RATE);
        rearLeft.setClosedLoopRampRate(RAMP_RATE);
        frontRight.setClosedLoopRampRate(RAMP_RATE);
        rearRight.setClosedLoopRampRate(RAMP_RATE);

    }

    /**
     * Sets the speed for the left and right side of the drive train.
     * 
     * @param leftThrust Left side output on a scale of -1.0 to 1.0.
     * @param rightThrust Right side output on a scale of -1.0 to 1.0.
     */
    public void move(double leftThrust, double rightThrust) {
        frontLeft.set(leftThrust);
        frontRight.set(rightThrust);
        SmartDashboard.putNumber("Left output", leftThrust);
        SmartDashboard.putNumber("Right output", rightThrust);
    }

    /**
     * Gets encoder values for the left and right SparkMAXs.
     */
    public double[] getEncoders() {
        return new double[] {
                -(frontLeft.getEncoder().getPosition() - leftEnccoderOffset),
                -(frontRight.getEncoder().getPosition() - rightEncoderOffset) };
    }

    /**
     * Resets the value of the encoder to zero.
     */
    public void resetEncoderOffset() {
        leftEnccoderOffset = frontLeft.getEncoder().getPosition();
        rightEncoderOffset = frontRight.getEncoder().getPosition();
    }

    /**
     * Set motors to brake mode.
     */
    public void brake() {
        if (frontLeft.getIdleMode() == IdleMode.kCoast) {
            frontLeft.setIdleMode(IdleMode.kBrake);
            rearLeft.setIdleMode(IdleMode.kBrake);
            frontRight.setIdleMode(IdleMode.kBrake);
            rearRight.setIdleMode(IdleMode.kBrake);
        }
    }

    /**
     * Set motors to coast mode.
     */
    public void coast() {
        if (frontLeft.getIdleMode() == IdleMode.kBrake) {
            frontLeft.setIdleMode(IdleMode.kCoast);
            rearLeft.setIdleMode(IdleMode.kCoast);
            frontRight.setIdleMode(IdleMode.kCoast);
            rearRight.setIdleMode(IdleMode.kCoast);
        }
    }
}