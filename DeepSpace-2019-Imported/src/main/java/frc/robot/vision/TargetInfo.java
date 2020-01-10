package frc.robot.vision;

/**
 * A container class for Targets detected by the vision system, containing the location in three-dimensional space.
 */
public class TargetInfo {
    protected double x;
    protected double y;
    protected double z;
    protected double angle;

    public TargetInfo(double y, double z, double x, double angle) {
        this.y = y;
        this.z = z;
        this.x = x;
        this.angle = angle;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public double getAngle() {
        return angle;
    }
}