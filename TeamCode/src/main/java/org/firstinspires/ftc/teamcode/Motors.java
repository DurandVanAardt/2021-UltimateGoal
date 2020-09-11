package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Initialization.Variables;

public class Motors {
    private Variables var;

    public Motors(Variables var) {
        this.var = var;
    }

    public void driveStrafe(double left_stick_x, double left_stick_y, double right_stick_x, double speedControl) {

        double Magnitude = Math.hypot(left_stick_x, left_stick_y);
        double Direction = Math.atan2(left_stick_y, left_stick_x) - Math.PI / 4;

        double v1 = Magnitude * Math.sin(Direction) - right_stick_x;
        double v2 = Magnitude * Math.cos(Direction) + right_stick_x;
        double v3 = Magnitude * Math.cos(Direction) - right_stick_x;
        double v4 = Magnitude * Math.sin(Direction) + right_stick_x;

        var.robot.leftFront.setPower(v1 * speedControl);
        var.robot.rightFront.setPower(v2 * speedControl);
        var.robot.leftBack.setPower(v3 * speedControl);
        var.robot.rightBack.setPower(v4 * speedControl);
    }

    public void strafeR(double speed, double speedControl, double target) {
        speed *= speedControl;
        speed -= 0.1;
        double v1 = speed;
        double v2 = -speed;
        double v3 = -speed;
        double v4 = speed;

        double diff = (var.getGlobalAngle() - target) * 0.01;

        v1 -= diff;
        v2 += diff;
        v3 -= diff;
        v4 += diff;

        var.robot.leftFront.setPower(v1);
        var.robot.rightFront.setPower(v2);
        var.robot.leftBack.setPower(v3);
        var.robot.rightBack.setPower(v4);
    }

    public void strafeL(double speed, double speedControl, double target) {
        speed *= speedControl;
        speed -= 0.1;
        double v1 = speed;
        double v2 = -speed;
        double v3 = -speed;
        double v4 = speed;

        double diff = (var.getGlobalAngle() - target) * 0.01;

        v1 += diff;
        v2 -= diff;
        v3 += diff;
        v4 -= diff;

        var.robot.leftFront.setPower(v1);
        var.robot.rightFront.setPower(v2);
        var.robot.leftBack.setPower(v3);
        var.robot.rightBack.setPower(v4);
    }

    public void stop() {
        var.robot.leftFront.setPower(0);
        var.robot.rightFront.setPower(0);
        var.robot.leftBack.setPower(0);
        var.robot.rightBack.setPower(0);
    }
}
