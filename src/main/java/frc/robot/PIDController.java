package frc.robot;

public class PIDController {
    private double m_p;
    private double m_i;
    private double m_d;

    private double m_target;

    private double last_error;
    private boolean first_run;

    private double min_volt = -12.0;
    private double max_volt = 12.0;

    public PIDController(double p, double i, double d) {
        this.m_p = p;
        this.m_i = i;
        this.m_d = d;

        this.first_run = true;
    }

    public double calc(double position) {
        double out = 0;
        double error = this.m_target - position;

        out += this.m_p * error;

        // out += stuff for I

        if (!first_run) {
            out -= (last_error - error) / Robot.PERIOD * this.m_d;
        }

        this.first_run = false;
        this.last_error = error;

        if (out > this.max_volt) {
            out = this.max_volt;
        } else if (out < this.min_volt) {
            out = this.min_volt;
        }

        return out;
    }

    public void setTarget(double newTarget) {
        this.m_target = newTarget;
        this.first_run = true;
    }

    public void setMinMax(double min_out, double max_out) {
        this.min_volt = min_out;
        this.max_volt = max_out;
    }
}
