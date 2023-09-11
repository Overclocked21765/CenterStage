package org.firstinspires.ftc.teamcode.common;

import static org.firstinspires.ftc.teamcode.common.Constants.Drive.ROTATION_CONSTANT;

public class Algorithms {
    public static Vector joystickMapping(double x, double y){
        double m_x = x * Math.sqrt(1 - y * y / 2);
        double m_y = y * Math.sqrt(1 - x * x / 2);
        double theta = Math.atan2(m_y, m_x);
        double r = Math.hypot(m_x, m_y);
        return new Vector(new Vector.VectorPolar(r, theta));
    }

    public static double[] returnMecanumValues(double forward, double strafe, double rotation, double heading, double scalePower){
        Vector dir = joystickMapping(strafe, forward);
        double angle = dir.theta;
        double power = dir.r;

        double sin = Math.sin(Math.toRadians(angle));
        double cos = Math.cos(Math.toRadians(angle));
        double maxTrig = Math.max(Math.abs(sin), Math.abs(Math.cos(angle)));
        double xPower = power * cos;
        double yPower = power * sin;

        if (maxTrig != 0){
            xPower /= maxTrig;
            yPower /= maxTrig;
        }

        double frontLeft = xPower;
        double frontRight = yPower;
        double backLeft = yPower;
        double backRight = xPower;
        frontRight -= (rotation * ROTATION_CONSTANT);
        backLeft += (rotation * ROTATION_CONSTANT);
        frontLeft += (rotation * ROTATION_CONSTANT);
        backRight -= (rotation * ROTATION_CONSTANT);
        double maxPower = Math.max(Math.max(Math.abs(frontLeft), Math.abs(frontRight)), Math.max(Math.abs(backLeft), Math.abs(backRight)));

        if (maxPower > 1){
            frontLeft /= maxPower;
            frontRight /= maxPower;
            backLeft /= maxPower;
            backRight /= maxPower;
        }

        frontLeft *= scalePower;
        frontRight *= scalePower;
        backLeft *= scalePower;
        backRight *= scalePower;
        double[] values = {frontLeft, frontRight, backLeft, backRight};
        return values;
    }
}
