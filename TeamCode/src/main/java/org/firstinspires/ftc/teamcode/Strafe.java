package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Initialization.Variables;

public class Strafe {
    private Variables var;

    public Strafe(Variables var) {
        this.var = var;
    }

    public void drive(double r, double robotAngle, double rightX, double speedControl) {
        double v1 = r * Math.sin(robotAngle) - rightX;
        double v2 = r * Math.cos(robotAngle) + rightX;
        double v3 = r * Math.cos(robotAngle) - rightX;
        double v4 = r * Math.sin(robotAngle) + rightX;

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
