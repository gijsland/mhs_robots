package frc.robot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

    public static final double PERIOD = 0.020;

    double startTime;
    double time;
    double dt;

    TalonFX leftMotor;
    TalonFX rightMotor;

    Joystick j;

    PIDController fwdPID;
    PIDController turnPID;

    PigeonIMU gyro;

    Profile p;

    public Robot() {
        super(PERIOD);
    }

    public static double ticksToMeters(double ticks) {
        return ticks * 9.0 / 74.0 / 2048.0 * Math.PI * 4.0 * 0.0254;
    }

    @Override
    public void robotInit() {
        startTime = Timer.getFPGATimestamp();
        time = 0;

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

        j = new Joystick(0);

        fwdPID = new PIDController(40, 0, 0);
        fwdPID.setMinMax(-8.0, 8.0);
        turnPID = new PIDController(0.0, 0, 0.0);
        turnPID.setMinMax(-3.0, 3.0);

        gyro = new PigeonIMU(10);
        gyro.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 5);

        double maxV = 2.0;
        double maxA = 3.0;
        p = new Profile(maxV, maxA);

        NetworkTableInstance.getDefault().setUpdateRate(PERIOD);


    }

    double forward = 0;
    double turn = 0;

    double leftPosition;
    double rightPosition;
    double position;

    double angle;

    int skip;

    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("dt", dt);
        SmartDashboard.putNumber("Time", time);
        SmartDashboard.putNumber("left position", leftPosition);
        SmartDashboard.putNumber("right position", rightPosition);
        SmartDashboard.putNumber("position", position);
        SmartDashboard.putNumber("angle (deg)", Math.toDegrees(angle));
    }

    @Override
    public void teleopInit() {
        gyro.setFusedHeading(0);
        leftMotor.setSelectedSensorPosition(0);
        rightMotor.setSelectedSensorPosition(0);
    }

    @Override
    public void teleopPeriodic() {
        double newTime = Timer.getFPGATimestamp();
        dt = newTime - time;
        time = newTime;
        leftPosition = ticksToMeters(leftMotor.getSelectedSensorPosition());
        rightPosition = ticksToMeters(rightMotor.getSelectedSensorPosition());
        position = (leftPosition + rightPosition) / 2.0;
        angle = Math.toRadians(gyro.getFusedHeading());

        forward = -j.getRawAxis(1);
        turn = -j.getRawAxis(4);

        leftMotor.set(ControlMode.PercentOutput, forward - turn);
        rightMotor.set(ControlMode.PercentOutput, forward + turn);
    }

    boolean autoFirst = true;
    @Override
    public void autonomousInit() {
        autoFirst = true;
        leftMotor.setSelectedSensorPosition(0);
        rightMotor.setSelectedSensorPosition(0);
        gyro.setFusedHeading(0);

        // fwdPID.setTarget(2.0);


        // calculate the trapezoid with 2m and whatever V / A
        // give start time



        turnPID.setTarget(Math.toRadians(0));

        skip = 10;
    }

    @Override
    public void autonomousPeriodic() {
        if (autoFirst) {
            p.calculate(4.0, Timer.getFPGATimestamp());
            autoFirst = false;
        }
        double newTime = Timer.getFPGATimestamp();
        dt = newTime - time;
        time = newTime;
        // if (skip > 0) {
        //   skip--;
        //   return;
        // }
        leftPosition = ticksToMeters(leftMotor.getSelectedSensorPosition());
        rightPosition = ticksToMeters(rightMotor.getSelectedSensorPosition());
        position = (leftPosition + rightPosition) / 2.0;
        angle = Math.toRadians(gyro.getFusedHeading());


        // ask where we should be at current time
        // update the pid target
        // then pid calc
        double profile_position = p.getPosition(Timer.getFPGATimestamp());
        double profile_velocity = p.getVelocity(Timer.getFPGATimestamp());
        fwdPID.setTarget(profile_position);
        forward = (profile_velocity * 2.878) + (0.3 * p.getAccel(Timer.getFPGATimestamp())) + fwdPID.calc(position);

        turn = turnPID.calc(angle);

        SmartDashboard.putNumber("velocity", ticksToMeters(leftMotor.getSelectedSensorVelocity() * 10));
        SmartDashboard.putNumber("profile position", profile_position);
        SmartDashboard.putNumber("profile velocity", profile_velocity);

        leftMotor.set(ControlMode.PercentOutput, (forward - turn) / 12.0);
        rightMotor.set(ControlMode.PercentOutput, (forward + turn) / 12.0);
    }

    @Override
    public void disabledInit() {
        SmartDashboard.putNumber("profile position", 0);
        SmartDashboard.putNumber("profile velocity", 0);
    }

    @Override
    public void disabledPeriodic() {
        leftPosition = ticksToMeters(leftMotor.getSelectedSensorPosition());
        rightPosition = ticksToMeters(rightMotor.getSelectedSensorPosition());
        position = (leftPosition + rightPosition) / 2.0;
        angle = Math.toRadians(gyro.getFusedHeading());
        double newTime = Timer.getFPGATimestamp();
        dt = newTime - time;
        time = newTime;
    }
}
