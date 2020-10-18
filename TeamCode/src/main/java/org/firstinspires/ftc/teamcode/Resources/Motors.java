package org.firstinspires.ftc.teamcode.Resources;

import org.firstinspires.ftc.teamcode.Initialization.Variables;

@SuppressWarnings("unused")
public class Motors {
    private Variables var;
    PIDController           pidRotate, pidDrive, pidStrafe;

    public Motors(Variables var) {
        pidRotate = new PIDController(.003, .00003, 0);
        this.var = var;
        pidDrive = new PIDController(.05, 0, 0);
        pidStrafe = new PIDController(.05,0,0);



        pidStrafe.setSetpoint(0);
        pidStrafe.setOutputRange(0, 0.3);
        pidStrafe.setInputRange(-90, 90);
        pidStrafe.enable();

    }


    public void driveStrafe(double left_stick_x, double left_stick_y, double right_stick_x, double speedControl) {
        left_stick_x *= -1;
        left_stick_y *= -1;
        right_stick_x *= -1;
        if (left_stick_x == 0 && left_stick_y == 0) {
            var.resetAngle();
        }

        double correction = pidStrafe.performPID(var.getAngle());
        right_stick_x += correction;

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
    
    public void driveStrafe(double angle, double speed, boolean check) {

        if (!check) {
            var.resetAngle();
        }

        double correction = pidStrafe.performPID(var.getAngle());

        double v1 = speed * Math.sin(angle) - correction;
        double v2 = speed * Math.cos(angle) + correction;
        double v3 = speed * Math.cos(angle) - correction;
        double v4 = speed * Math.sin(angle) + correction;

        var.robot.leftFront.setPower(v1);
        var.robot.rightFront.setPower(v2);
        var.robot.leftBack.setPower(v3);
        var.robot.rightBack.setPower(v4);

    }

    public void strafeR(double speed, double speedControl) {
        speed *= speedControl;
        speed -= 0.1;
        double v1 = speed;
        double v2 = -speed;
        double v3 = -speed;
        double v4 = speed;

        double correction = pidStrafe.performPID(var.getTrueAngle());

        v1 -= correction;
        v2 += correction;
        v3 -= correction;
        v4 += correction;

        var.robot.leftFront.setPower(v1);
        var.robot.rightFront.setPower(v2);
        var.robot.leftBack.setPower(v3);
        var.robot.rightBack.setPower(v4);
    }

    public void strafeL(double speed, double speedControl) {
        speed *= speedControl;
        speed -= 0.1;
        double v1 = -speed;
        double v2 = speed;
        double v3 = speed;
        double v4 = -speed;

        double correction = pidStrafe.performPID(var.getTrueAngle());

        v1 += correction;
        v2 -= correction;
        v3 += correction;
        v4 -= correction;

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
