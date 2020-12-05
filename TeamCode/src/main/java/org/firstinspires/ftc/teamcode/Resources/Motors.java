package org.firstinspires.ftc.teamcode.Resources;

import org.firstinspires.ftc.teamcode.Initialization.Variables;

//@SuppressWarnings("unused")
public class Motors {
    private Variables var;
    private RobotHardwareMap robot;
    public PIDController pidRotate, pidStrafe;


    public Motors(Variables var) {
        this.var = var;
        robot = var.robot;

        pidRotate = new PIDController(.003, .00003, 0);
        pidStrafe = new PIDController(.05,0,0);

        pidStrafe.setSetpoint(0);
        pidStrafe.enable();

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

    public boolean rotate(double SP) {
        double PV = var.getTrueAngle();
//        double degrees = SP;
        double power = 1.0;

        // if degrees > 359 we cap at 359 with same sign as original degrees.
//        if (Math.abs(SP) > 359)
//            SP = (int) Math.copySign(359, SP);

        pidRotate.setSetpoint(SP);
        pidRotate.setInputRange(0, SP);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();
        pidRotate.performPID(PV);

        // power will be + on left turn.

        if (!pidRotate.onTarget()) {

            power = (pidRotate.performPID(PV)); // power will be - on right turn.

            robot.leftFront.setPower(-power);
            robot.rightFront.setPower(power);
            robot.leftBack.setPower(-power);
            robot.rightBack.setPower(power);

            return true;
        }

        else if (pidRotate.onTarget()) {
            // turn the motors off.

            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);

            // reset angle tracking on new heading.
            var.resetAngle();
            return false;
        }
        else if (!pidRotate.onTarget() && var.getAngle() > SP)

            power = (pidRotate.performPID(PV)) / 5;

        robot.leftFront.setPower(-power);
        robot.rightFront.setPower(power);
        robot.leftBack.setPower(-power);
        robot.rightBack.setPower(power);

        return true;
    }

    public void strafe(double angle, double speed, boolean check) {

        // correct the angle
        angle += 45;
        // convert to radials
        angle *= Math.PI / 180;

        if (check) {
            var.resetAngle();
//            pidStrafe.enable();
        }

        double correction = pidStrafe.performPID(var.getAngle());

        double v1 = speed * Math.sin(angle) - correction;
        double v2 = speed * Math.cos(angle) + correction;
        double v3 = speed * Math.cos(angle) - correction;
        double v4 = speed * Math.sin(angle) + correction;

        robot.leftFront.setPower(v1);
        robot.rightFront.setPower(v2);
        robot.leftBack.setPower(v3);
        robot.rightBack.setPower(v4);

    }

    public double mecanum(double Strafe, double Forward, double Turn) {
        //Find the magnitude of the controller's input
        double r = Math.hypot(Strafe, Forward);

        //returns point from +X axis to point (forward, strafe)
        double robotAngle = Math.atan2(Forward, Strafe) - Math.PI / 4;

        double correction = 0;
        if (Turn != 0) {
            var.resetAngle();
        } else {
            correction = pidStrafe.performPID(var.getAngle());
        }

        //Quantity to turn by (turn)
        double rightX = Turn - correction;

        //double vX represents the velocities sent to each motor
        double v1 = (r * Math.cos(robotAngle)) + rightX;
        double v2 = (r * Math.sin(robotAngle)) - rightX;
        double v3 = (r * Math.sin(robotAngle)) + rightX;
        double v4 = (r * Math.cos(robotAngle)) - rightX;

        if (Strafe == 0) {
            v1 = Forward + rightX;
            v2 = Forward - rightX;
            v3 = Forward + rightX;
            v4 = Forward - rightX;
        }

        if (Forward == 0) {
            v1 = Strafe + rightX;
            v2 = -Strafe - rightX;
            v3 = -Strafe + rightX;
            v4 = Strafe - rightX;
        }

        robot.leftFront.setPower(v1);
        robot.rightFront.setPower(v2);
        robot.leftBack.setPower(v3);
        robot.rightBack.setPower(v4);

        return rightX;
    }


    public void drive(double left_stick_y, double right_stick_x, boolean check) {

        if (check) {
            var.resetAngle();
            pidStrafe.reset();
        }

        double correction = 0;

        if (right_stick_x == 0)
            correction = pidStrafe.performPID(var.getAngle());

        double v1 = left_stick_y - right_stick_x;
        double v2 = left_stick_y + right_stick_x;
        double v3 = left_stick_y - right_stick_x;
        double v4 = left_stick_y + right_stick_x;

        robot.leftFront.setPower(v1 - correction);
        robot.rightFront.setPower(v2 + correction);
        robot.leftBack.setPower(v3 - correction);
        robot.rightBack.setPower(v4 + correction);
    }

    public void stop() {
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
    }


}
