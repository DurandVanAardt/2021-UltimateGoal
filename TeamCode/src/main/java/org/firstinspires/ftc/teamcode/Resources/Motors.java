package org.firstinspires.ftc.teamcode.Resources;

import org.firstinspires.ftc.teamcode.Initialization.Variables;

@SuppressWarnings("unused")
public class Motors {
    private Variables var;
    private RobotHardwareMap robot;
    public PIDController           pidRotate, pidDrive, pidStrafe, pidMecanum, pidStrafe2;


    public Motors(Variables var) {
        pidRotate = new PIDController(.003, .00003, 0);
        pidDrive = new PIDController(.05, 0, 0);
        pidStrafe = new PIDController(.05,0,0);
        pidStrafe2 = new PIDController(.05,0,0);

        this.var = var;
        robot = var.robot;



        pidStrafe.setSetpoint(0);
        pidStrafe.setOutputRange(-0.01, 0.01);
        pidStrafe.setInputRange(-90, 90);
        pidStrafe.enable();


        pidStrafe2.setSetpoint(0);
        pidStrafe2.setOutputRange(0, 0.3);
        pidStrafe2.setInputRange(-90, 90);
        pidStrafe2.enable();

//        double SPstrafe = var.getAngle();
//
//        pidMecanum.setSetpoint(0);
//        pidMecanum.setOutputRange(-1, 1);
//        pidMecanum.enable();


//        pidStrafe.setInputRange(-90, 90);

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, 0.3);
        pidDrive.setInputRange(-90, 90);

    }


    /*public void driveStrafe(double left_stick_x, double left_stick_y, double right_stick_x, double speedControl) {
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

    }*/
    public void driveStrafe(double angle, double speed, boolean check) {

        if (!check) {
            var.resetAngle();
        }

        double correction = pidStrafe2.performPID(var.getAngle());

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
        // restart imu angle tracking.


        double PV1 = var.getTrueAngle();
//        telemetry.addData("Current angle",PV1);

        double degrees = SP;


        double power = 1.0;

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();
        pidRotate.performPID(PV1);

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        // power will be + on left turn.


//        if (degrees < 0)
//        {
        // On right turn we have to get off zero first.
//           while (opModeIsActive() && getAngle() == 0)
//if (getAngle() ==0)
//
//            {
//                robot.leftFront.setPower(power);
//                robot.rightFront.setPower(-power);
//                robot.leftBack.setPower(power);
//                robot.rightBack.setPower(-power);
//            }


        if (!pidRotate.onTarget()) {

//            loopCount += 1;
//            telemetry.addData("OnTarget", pidRotate.onTarget());

//            Loop = true;

            power = (pidRotate.performPID(PV1)); // power will be - on right turn.

            robot.leftFront.setPower(-power);
            robot.rightFront.setPower(power);
            robot.leftBack.setPower(-power);
            robot.rightBack.setPower(power);

            return true;
        }

//        while (opModeIsActive() && !pidRotate3.onTarget());
        else if (pidRotate.onTarget()) {


            // turn the motors off.

            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);

            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);


            // wait for rotation to stop.


            // reset angle tracking on new heading.
            var.resetAngle();
            return false;
        }
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
        if (Turn != 0 || (Strafe == 0 && Forward == 0)) {
            var.resetAngle();
        } else {
            correction = pidStrafe.performPID(var.getAngle()) * 7.5;
        }

        //Quantity to turn by (turn)
        double rightX = Turn - correction;

        //double vX represents the velocities sent to each motor
        double v1 = (r * Math.cos(robotAngle)) + rightX;
        double v2 = (r * Math.sin(robotAngle)) - rightX;
        double v3 = (r * Math.sin(robotAngle)) + rightX;
        double v4 = (r * Math.cos(robotAngle)) - rightX;

        if (robotAngle == 45 * Math.PI / 180 || robotAngle == -135 * Math.PI / 180) {
            v1 = Math.copySign(1, robotAngle);
            v2 = v1;
            v3 = v1;
            v4 = v1;
        }

        robot.leftFront.setPower(v1);
        robot.rightFront.setPower(v2);
        robot.leftBack.setPower(v3);
        robot.rightBack.setPower(v4);

        return robotAngle * 180 / Math.PI;
    }





    public void drive(double left_stick_y, double right_stick_x, boolean check) {

        if (check) {
            var.resetAngle();
            pidStrafe.reset();
            pidDrive.enable();
        }

        double correction = 0;

        if (right_stick_x == 0)
            correction = pidDrive.performPID(var.getAngle());

        double v1 = left_stick_y - right_stick_x;
        double v2 = left_stick_y + right_stick_x;
        double v3 = left_stick_y - right_stick_x;
        double v4 = left_stick_y + right_stick_x;

        robot.leftFront.setPower(v1 - correction);
        robot.rightFront.setPower(v2 + correction);
        robot.leftBack.setPower(v3 - correction);
        robot.rightBack.setPower(v4 + correction);
    }


    /*
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
    }*/

    public void stop() {
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);

        pidStrafe.reset();
        pidDrive.reset();
    }


}
