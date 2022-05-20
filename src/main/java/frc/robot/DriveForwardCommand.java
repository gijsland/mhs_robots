package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveForwardCommand {

    double m_meters;
    Drivetrain drive;

    public DriveForwardCommand(double meters) {
        this.m_meters = meters;
        this.drive = Drivetrain.getInstance();
    }

    public boolean arewethereyet() {
        SmartDashboard.putNumber("ehh", Math.abs(this.drive.getPosition() - this.m_meters));
        return (Math.abs(this.drive.getPosition() - this.m_meters) < 0.02) && (Math.abs(this.drive.getVelocity()) < 0.1);
    }

    public void go() {
        this.drive.setPosition(this.m_meters);
    }
}
