package frc.robot.swerve;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class SwerveDrive {
    private WheelModule m_frontLeft = new WheelModule(0, 1, 0);
    private WheelModule m_frontRight = new WheelModule(2, 3, 0);
    private WheelModule m_rearLeft = new WheelModule(4, 5, 0);
    private WheelModule m_rearRight = new WheelModule(6, 7, 0);
    
    private boolean m_fieldRelative = true;

    private ADXRS450_Gyro m_gyro;
    private double m_startingHeading = 0;

    // Max Chassis Velocities
    private double m_maximumVX = 5.0; // feet per second
    private double m_maximumVY = 5.0; // feet per second
    private double m_maximumOmega = 12.56; // radians per second

    // Wheelbase Dimensions
    private double m_wheelbaseLength = 32;
    private double m_wheelbaseWidth = 32;

    public SwerveDrive() {
    }

    public void Periodic() {

    }


    // Robot Coordinate system:
    // X is +right
    // Y is +forward
    // Rot is +CW (turning right)
    // vx, vy, and omega are in [-1, 1]
    public void Drive(double vx, double vy, double omega) {
        // Factor in field relativity
        // check tablet notes
        if (m_fieldRelative) {
            // vx and vy are relative to the field so we must convert to robot relative coordinates
            double currentHeading = GetHeading();
            // get robot relative components of vx and vy
            double vx_robotX = Math.cos(currentHeading) * vx;
            double vx_robotY = Math.sin(currentHeading) * vx;
            double vy_robotX = Math.sin(currentHeading) * vy;
            double vy_robotY = Math.cos(currentHeading) * vy;

            // combine field relative components
            vx = vx_robotX + vy_robotX;
            vy = vx_robotY + vy_robotY;
        }

        // Derivation of Inverse Kinematics Swerve Drive: https://www.chiefdelphi.com/uploads/default/original/3X/8/c/8c0451987d09519712780ce18ce6755c21a0acc0.pdf
        // Calculate Inverse Kinematics equations
        double A = vx - omega * (m_wheelbaseLength / 2);
        double B = vx + omega * (m_wheelbaseLength / 2);
        double C = vy - omega * (m_wheelbaseWidth / 2);
        double D = vy + omega * (m_wheelbaseWidth / 2);

        // Define Wheel velocities from IK equations
        double vx_fl = B;
        double vy_fl = D;
        double vx_fr = B;
        double vy_fr = C;
        double vx_rl = A;
        double vy_rl = D;
        double vx_rr = A;
        double vy_rr = C;


        // Calculate speeds and angles
        double v_fl = Math.sqrt(Math.pow(vx_fl, 2) + Math.pow(vy_fl, 2));
        double theta_fl = Math.atan2(vx_fl, vy_fl);
        double v_fr = Math.sqrt(Math.pow(vx_fr, 2) + Math.pow(vy_fr, 2));
        double theta_fr = Math.atan2(vx_fr, vy_fr);
        double v_rl = Math.sqrt(Math.pow(vx_rl, 2) + Math.pow(vy_rl, 2));
        double theta_rl = Math.atan2(vx_rl, vy_rl);
        double v_rr = Math.sqrt(Math.pow(vx_rr, 2) + Math.pow(vy_rr, 2));
        double theta_rr = Math.atan2(vx_rr, vy_rr);

        // Normalize speeds
        double maxV = Math.max(v_fl, Math.max(v_fr, Math.max(v_rl, v_rr)));
        if (maxV > 1) {
            v_fl = v_fl / maxV;
            v_fr = v_fr / maxV;
            v_rl = v_rl / maxV;
            v_rr = v_rr / maxV;
        }

        m_frontLeft.Drive(v_fl, theta_fl);
        m_frontRight.Drive(v_fr, theta_fr);
        m_rearLeft.Drive(v_rl, theta_rl);
        m_rearRight.Drive(v_rr, theta_rr);
    }

    double GetHeading() {
        return (m_gyro.getAngle() - m_startingHeading) * Math.PI / 180;
    }
}
