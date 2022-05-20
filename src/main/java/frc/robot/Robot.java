package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Robot extends TimedRobot {

    public static final double PERIOD = 0.020;

    double startTime;
    double time;
    double dt;

    Drivetrain drive;
    Joystick j;

    public Robot() {
        super(PERIOD);
    }

    @Override
    public void robotInit() {
        startTime = Timer.getFPGATimestamp();
        time = 0;

        j = new Joystick(0);
        drive = Drivetrain.getInstance();

        NetworkTableInstance.getDefault().setUpdateRate(PERIOD);
    }

    int skip;

    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("dt", dt);
        SmartDashboard.putNumber("Time", time);
        drive.updateDashboard();
    }

    @Override
    public void teleopInit() {
        drive.resetSensors();
    }

    @Override
    public void teleopPeriodic() {
        double newTime = Timer.getFPGATimestamp();
        dt = newTime - time;
        time = newTime;

        drive.setTeleopDrive(-j.getRawAxis(1), j.getRawAxis(4), j.getRawButton(6));
    }

    ArrayList<DriveForwardCommand> driveTargets;

    boolean autoFirst = true;
    @Override
    public void autonomousInit() {
        autoFirst = true;
        drive.resetSensors();

        driveTargets = new ArrayList<DriveForwardCommand>();
        driveTargets.add(new DriveForwardCommand(2.0));
        driveTargets.add(new DriveForwardCommand(-1.0));
        driveTargets.add(new DriveForwardCommand(0.0));

        skip = 10;
    }

    @Override
    public void autonomousPeriodic() {
        double newTime = Timer.getFPGATimestamp();
        dt = newTime - time;
        time = newTime;
//        System.out.println("ur mum");

        if (driveTargets.size() > 0) {
            DriveForwardCommand activeCommand = driveTargets.get(0);
            this.drive.varUpdate();
            activeCommand.go();

            if (activeCommand.arewethereyet()) {
                driveTargets.remove(0);
            }
        } else {
            this.drive.setMotorSpeeds(0, 0);
        }
    }

    @Override
    public void disabledInit() {
        drive.resetSensors();
        SmartDashboard.putNumber("profile position", 0);
        SmartDashboard.putNumber("profile velocity", 0);
    }

    @Override
    public void disabledPeriodic() {
        drive.varUpdate();
        double newTime = Timer.getFPGATimestamp();
        dt = newTime - time;
        time = newTime;
    }
}
