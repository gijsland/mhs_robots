package frc.robot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain {

    // the instance and getInstance method is a singleton pattern
    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        // this next line is called a ternary operator
        return (instance == null) ? instance = new Drivetrain() : instance;
    }

    public static double ticksToMeters(double ticks) {
        return ticks * 9.0 / 74.0 / 2048.0 * Math.PI * 4.0 * 0.0254;
    }

    TalonFX leftMotor;
    TalonFX rightMotor;

    PIDController fwdPID;
    PIDController turnPID;

    Profile forwardProfile;
    Profile turnProfile;

    PigeonIMU gyro;

    CheesyDriveHelper cdh;

    double leftPosition;
    double rightPosition;
    double position;

    double angle;


    private Drivetrain() {
        leftMotor = new TalonFX(0);
        leftMotor.setInverted(true);
        leftMotor.setNeutralMode(NeutralMode.Coast);

        rightMotor = new TalonFX(1);
        rightMotor.setInverted(false);
        rightMotor.setNeutralMode(NeutralMode.Coast);

        leftMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
        rightMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);

        leftMotor.configNeutralDeadband(0.001, 100);
        rightMotor.configNeutralDeadband(0.001, 100);

        leftMotor.configVoltageCompSaturation(12.0, 100);
        leftMotor.enableVoltageCompensation(true);
        rightMotor.configVoltageCompSaturation(12.0, 100);
        rightMotor.enableVoltageCompensation(true);

        leftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
        rightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);

        leftMotor.setControlFramePeriod(ControlFrame.Control_3_General, 5);
        rightMotor.setControlFramePeriod(ControlFrame.Control_3_General, 5);


        gyro = new PigeonIMU(10);
        gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 5);

        fwdPID = new PIDController(20, 0, 0);
        fwdPID.setMinMax(-8.0, 8.0);
        turnPID = new PIDController(0.0, 0, 0.0);
        turnPID.setMinMax(-3.0, 3.0);

        cdh = new CheesyDriveHelper();
    }

    public void resetSensors() {
        gyro.setFusedHeading(0);
        leftMotor.setSelectedSensorPosition(0);
        rightMotor.setSelectedSensorPosition(0);
    }

    public double getLeftMeters() {
        return ticksToMeters(leftMotor.getSelectedSensorPosition());
    }
    public double getRightMeters() {
        return ticksToMeters(rightMotor.getSelectedSensorPosition());
    }
    public double gyroAngleRadians() {
        return Math.toRadians(gyro.getFusedHeading());
    }
    public double gyroAngleDegrees() {
        return gyro.getFusedHeading();
    }

    public double getPosition() {
        return position;
    }

    public double getVelocity() {
        return ticksToMeters(leftMotor.getSelectedSensorVelocity() + rightMotor.getSelectedSensorVelocity()) * 10.0 / 2.0;
    }

    public void setMotorSpeeds(double left, double right) {
        leftMotor.set(ControlMode.PercentOutput, left);
        rightMotor.set(ControlMode.PercentOutput, right);
    }

    public void setTeleopDrive(double fwd, double turn, boolean quickTurn) {
        cdh.cheesyDrive(fwd, turn, quickTurn);
        leftMotor.set(ControlMode.PercentOutput, cdh.getLeftPWM());
        rightMotor.set(ControlMode.PercentOutput, cdh.getRightPWM());
    }

    public void varUpdate() {
        leftPosition = getLeftMeters();
        rightPosition = getRightMeters();
        position = (leftPosition + rightPosition) / 2.0;
        angle = gyroAngleRadians();
    }

    public void setPosition(double meters) {
        this.fwdPID.setTarget(meters);

        double forward = this.fwdPID.calc(this.position);
        double turn = 0;

        leftMotor.set(ControlMode.PercentOutput, (forward - turn) / 12.0);
        rightMotor.set(ControlMode.PercentOutput, (forward + turn) / 12.0);
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("left position", leftPosition);
        SmartDashboard.putNumber("right position", rightPosition);
        SmartDashboard.putNumber("position", position);
        SmartDashboard.putNumber("angle (deg)", gyroAngleDegrees());
    }
}
