package org.firstinspires.ftc.teamcode.Resources;

import android.provider.Settings;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Initialization.Variables;

import java.util.Base64;

@SuppressWarnings("unused")
public class Motors {
    private Variables var;
    private RobotHardwareMap robot;
    public PIDController           pidRotate, pidDrive, pidStrafe, pidStrafe2;
    double ShooterloopCount = 0;

    public Motors(Variables var) {
       pidRotate = new PIDController(.004, .00001, 0);
       // pidRotate = new PIDController(.006, .00001, 0);
        pidDrive = new PIDController(.05, 0, 0);
//        pidStrafe = new PIDController(.09,0.0003,0);
        pidStrafe = new PIDController(.002,0,0);
        pidStrafe2 = new PIDController(.05,0,0);

        this.var = var;
        robot = var.robot;



        pidStrafe.setSetpoint(0);
        pidStrafe.setOutputRange(-0.035, 0.035);
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

    public boolean rotate(double SP)  {
        // restart imu angle tracking.


        double PV1 = var.getTrueAngle();
//        telemetry.addData("Current angle",PV1);

        double degrees = SP -1;


        double power = 1.0;
        double power2 = 0.1;

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
        pidRotate.setTolerance(0.00000000001);
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

            return false;

        }
        else if (!pidRotate.onTarget() && var.getAngle()>SP )



            power2 = ((pidRotate.performPID(PV1)));


        robot.leftFront.setPower(power);
        robot.rightFront.setPower(-power);
        robot.leftBack.setPower(power);
        robot.rightBack.setPower(-power);

        return true;



    }

    public void strafe(double angle, double speed, boolean check) {

        // correct the angle
        angle += 45;
        // convert to radials
        angle *= Math.PI / 180;

        if (check) {
            var.resetAngle();
            pidStrafe.enable();
        }
        //else if (!check) {

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

        public void Fire(double EncoderTurn)
        {

            robot.shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.shooterMotor.setTargetPosition((int) EncoderTurn);

        }

public void adjustAngle(boolean gamepad)

{
 double shooterAngle;

 double cameraDistance = 120;

 double angleHeight = 80;

 shooterAngle = Math.atan2(angleHeight,cameraDistance);

robot.shooterAngleMotor.setPower(shooterAngle);
}
    public void robotShooter(boolean gamepad)
    {

    robot.shooterMotor.setPower(1);


        }


        public void intakeStop() {

            robot.sucker.setPower(0);
        }

    public void suckerOut()
    {
            robot.sucker.setPower(1);

    }

    public void suckerIn(boolean gamepad)
    {

            robot.sucker.setPower(-1);



    }

    public void RIGHTTOPOS(boolean gamepad)
    {

        double loopCount;

        loopCount = 0;

        double speed = 1;

        loopCount= loopCount + 1;

        if ((loopCount % 2) == 0) {
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
        }


        if  ((loopCount % 2) != 0) {

            if (robot.colourL.alpha() < 500) {

                robot.leftFront.setPower(-1);
                robot.leftBack.setPower(-1);
            }
            if (robot.colourR.alpha() < 500) {

                robot.rightFront.setPower(-1);
                robot.rightBack.setPower(-1);

            } else stop();

            if ((robot.colourL.alpha() > 500) && (robot.colourR.alpha() > 500)) {

                driveStrafe(45 * Math.PI / 180, 1, true);

                if (robot.distanceL.getDistance(DistanceUnit.MM) < 400) {
                    stop();

                }
            }
        }


    }


    public void LEFTTOPOS(boolean gamepad)
    {

        double loopCount;

        loopCount = 0;

        double speed = 1;

        loopCount= loopCount + 1;

        if ((loopCount % 2) == 0) {
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
        }


        if  ((loopCount % 2) != 0)
        {

                if (robot.colourL.alpha() < 500) {

                    robot.leftFront.setPower(-1);
                    robot.leftBack.setPower(-1);
            }
                if (robot.colourR.alpha() < 500) {

                    robot.rightFront.setPower(-1);
                    robot.rightBack.setPower(-1);

            } else stop();

                if ((robot.colourL.alpha() > 500) && (robot.colourR.alpha() > 500)) {

                    driveStrafe(-45 * Math.PI / 180, 1, true);

                if  (robot.distanceL.getDistance(DistanceUnit.MM) < 400) {
                   stop();

                }
            }
        }





    }

    public double mecanum(double Strafe, double Forward, double Turn, boolean check2) {
        //Find the magnitude of the controller's input
        if (!check2 || Turn != 0 || (Strafe == 0 && Forward == 0)) {
            var.resetAngle();
        }
        double r = Math.hypot(Strafe, Forward);

        //returns point from +X axis to point (forward, strafe)
        double robotAngle = Math.atan2(Forward, Strafe) - Math.PI / 4;


//            var.resetAngle();
//        }

        //Quantity to turn by (turn)
       double correction = pidStrafe.performPID(var.getAngle());
       double rightX = Turn- correction;


        //double vX represents the velocities sent to each motor
        final double v1 = ((r * Math.cos(robotAngle)) + rightX);
        final double v2 = ((r * Math.sin(robotAngle)) - rightX);
        final double v3 = ((r * Math.sin(robotAngle)) + rightX);
        final double v4 = ((r * Math.cos(robotAngle)) - rightX);

        robot.leftFront.setPower(v1);
        robot.rightFront.setPower(v2);
        robot.leftBack.setPower(v3);
        robot.rightBack.setPower(v4);


        return rightX;
    }








    public void drive(double left_stick_y, double right_stick_x, boolean check) {

        if (check) {
            var.resetAngle();
            pidDrive.reset();
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
