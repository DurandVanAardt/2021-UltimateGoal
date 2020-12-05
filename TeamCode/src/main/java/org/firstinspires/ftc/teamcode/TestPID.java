package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMU;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Initialization.Initialize;
import org.firstinspires.ftc.teamcode.Initialization.Variables;
import org.firstinspires.ftc.teamcode.Resources.Motors;
import org.firstinspires.ftc.teamcode.Resources.PIDController;
import org.firstinspires.ftc.teamcode.Resources.RobotHardwareMap;

import java.util.Locale;

import static java.lang.Double.valueOf;

@TeleOp(name = "TestPID", group = "TeleOp")
public class TestPID extends LinearOpMode {

    Variables var;
    Motors motors;
    RobotHardwareMap robot;
    private Orientation angles;
    private boolean start = true;
    private boolean startL = true;
    private boolean startR = true;
    private boolean stopL = true;
    private boolean stopR = true;
    private boolean Loop = false;
    private boolean turning = false;
    private double SP;
    private double PV;
    private boolean strafeL;
    private boolean strafeR;
    private boolean turningA;
    private boolean turningB;
    private boolean turningX;
    private boolean turningY;
    PIDController pidRotate, pidRotate2, pidDrive, pidRotate3, pidStrafe;
    double gain = 0.0003;
    double rate = 0.0;
    double resetTime = 0.0;
    private double angle;
    private double angle2;
    private boolean first;
    private double power;
    double loopCount = 0;
    private double power2;

    @Override
    public void runOpMode() {

        boolean turnFirst = true;

        var = new Initialize().Init(hardwareMap);
        motors = var.motors;
        robot = var.robot;

        pidStrafe = new PIDController(.05, 0, 0);
        pidStrafe.setOutputRange(-1, 1);
        pidStrafe.setInputRange(-90, 90);
        pidStrafe.enable();

        pidRotate3 = new PIDController(.003, .00003, 0);
        composeTelemetry();

        waitForStart();

        while (opModeIsActive()) {

           // mecanum(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);


            if (turning || gamepad1.left_stick_button) {
                if (turnFirst) {
                    var.resetAngle();
                    turnFirst = false;
                    motors.pidRotate.reset();
                    motors.pidRotate.enable();
                    turning = motors.rotate(90);

                }
                turning = motors.rotate(90);
            }


//                    if (gamepad1.b) {
//            // right
//            motors.driveStrafe(135 * Math.PI / 180, 1, true);
//        }else if (gamepad1.y) {
//            // forward
//            motors.driveStrafe(45 * Math.PI / 180, 1, true);
//        }else if (gamepad1.x) {
//            // left
//            motors.driveStrafe(-45 * Math.PI / 180, 1, true);
//        }else if (gamepad1.a) {
//            // reverse
//            motors.driveStrafe(-135 * Math.PI / 180, 1, true);
//        }else {
//            motors.driveStrafe(0, 0, false);
//        }

//            PIDTurn(90);

//                telemetry.addData("FM", robot.distanceFM.getDistance(DistanceUnit.MM));
//                telemetry.addData("B", robot.distanceB.getDistance(DistanceUnit.MM));
//                telemetry.addData("L", robot.distanceL.getDistance(DistanceUnit.MM));
//                telemetry.addData("R", robot.distanceR.getDistance(DistanceUnit.MM));
            telemetry.addData("gain", gain);
            telemetry.addData("resetTime", resetTime);
            telemetry.addData("rate", rate);
            telemetry.addData("IMU",SP-(var.getTrueAngle()));
            telemetry.update();
        }

    }

    public void mecanum(double Strafe, double Forward, double Turn) {
        //Find the magnitude of the controller's input
        double r = Math.hypot(Strafe, Forward);

        //returns point from +X axis to point (forward, strafe)
        double robotAngle = Math.atan2(Forward, Strafe) - Math.PI / 4;

        //Quantity to turn by (turn)
        double rightX = Turn;

        //double vX represents the velocities sent to each motor
        final double v1 = (r * Math.cos(robotAngle)) + rightX;
        final double v2 = (r * Math.sin(robotAngle)) - rightX;
        final double v3 = (r * Math.sin(robotAngle)) + rightX;
        final double v4 = (r * Math.cos(robotAngle)) - rightX;

        robot.leftFront.setPower(v1);
        robot.rightFront.setPower(v2);
        robot.leftBack.setPower(v3);
        robot.rightBack.setPower(v4);
    }
    private boolean PIDStrafeLToWall(double power, double wallDistance) {
        PV/*Process Variable*/ = -getAngle();
        boolean distanceReached = robot.distanceL.getDistance(DistanceUnit.MM) <= wallDistance;
        if (start) {
            SP/*SetPoint*/ = -getAngle();
            start = false;
        }
        double correction = (SP - PV) * 0.02;

        double v1 = power - correction;
        double v2 = power - correction;
        double v3 = power + correction;
        double v4 = power + correction;

        robot.leftFront.setPower(-v1);
        robot.rightFront.setPower(v2);
        robot.leftBack.setPower(v3);
        robot.rightBack.setPower(-v4);

        if (distanceReached) {
            start = true;

            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightFront.setPower(0);
        }

        return !distanceReached;
    }

    private boolean PIDStrafeLFromWall(double power, double wallDistance) {
        PV/*Process Variable*/ = -getAngle();
        boolean distanceReached = robot.distanceL.getDistance(DistanceUnit.MM) >= wallDistance;
        if (start) {
            SP/*SetPoint*/ = -getAngle();
            start = false;
        }
        double correction = (SP - PV) * 0.02;

        double v1 = power - correction;
        double v2 = power - correction;
        double v3 = power + correction;
        double v4 = power + correction;

        robot.leftFront.setPower(-v1);
        robot.rightFront.setPower(v2);
        robot.leftBack.setPower(v3);
        robot.rightBack.setPower(-v4);

        if (distanceReached) {
            start = true;

            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightFront.setPower(0);
        }

        return !distanceReached;
    }

    private boolean PIDStrafeRToWall(double power, double wallDistance) {
        PV/*Process Variable*/ = -getAngle();
        boolean distanceReached = robot.distanceR.getDistance(DistanceUnit.MM) <= wallDistance;
        if (start) {
            SP/*SetPoint*/ = -getAngle();
            start = false;
        }
        double correction = (SP - PV) * 0.02;

        double v1 = power + correction;
        double v2 = power + correction;
        double v3 = power - correction;
        double v4 = power - correction;

        robot.leftFront.setPower(v1);
        robot.rightFront.setPower(-v2);
        robot.leftBack.setPower(-v3);
        robot.rightBack.setPower(v4);

        if (distanceReached) {
            start = true;

            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightFront.setPower(0);
        }

        return !distanceReached;
    }

    private boolean PIDStrafeRFromWall(double power, double wallDistance) {
        PV/*Process Variable*/ = -getAngle();
        boolean distanceReached = robot.distanceR.getDistance(DistanceUnit.MM) >= wallDistance;
        if (start) {
            SP/*SetPoint*/ = -getAngle();
            start = false;
        }
        double correction = (SP - PV) * 0.02;

        double v1 = power + correction;
        double v2 = power + correction;
        double v3 = power - correction;
        double v4 = power - correction;

        robot.leftFront.setPower(v1);
        robot.rightFront.setPower(-v2);
        robot.leftBack.setPower(-v3);
        robot.rightBack.setPower(v4);

        if (distanceReached) {
            start = true;

            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightFront.setPower(0);
        }

        return !distanceReached;
    }

    private void PIDStrafeLTrigger(double power, boolean strafe) {
        if (!strafe) {
            if (stopL) {
                startL = true;
                stopL = false;
                strafeL = false;
            }
            return;
        }
        strafeL = true;
        PV/*Process Variable*/ = -getAngle();
        if (startL) {
            SP/*SetPoint*/ = -getAngle();
            pidStrafe.setSetpoint(SP);
            startL = false;
            stopL = true;
        }

        double correction = pidStrafe.performPID(PV);

        double v1 = power - correction;
        double v2 = power - correction;
        double v3 = power + correction;
        double v4 = power + correction;

        robot.leftFront.setPower(-v1);
        robot.rightFront.setPower(v2);
        robot.leftBack.setPower(v3);
        robot.rightBack.setPower(-v4);

    }

    private void PIDStrafeRTrigger(double power, boolean strafe) {
        if (!strafe) {
            if (stopR) {
                startR = true;
                stopR = false;
                strafeR = false;
            }
            return;
        }
        strafeR = true;
        PV/*Process Variable*/ = -getAngle();
        if (startR) {
            SP/*SetPoint*/ = -getAngle();
            pidStrafe.setSetpoint(SP);
            startR = false;
            stopR = true;
        }
        double correction = pidStrafe.performPID(PV);

        double v1 = power + correction;
        double v2 = power + correction;
        double v3 = power - correction;
        double v4 = power - correction;

        robot.leftFront.setPower(v1);
        robot.rightFront.setPower(-v2);
        robot.leftBack.setPower(-v3);
        robot.rightBack.setPower(v4);

    }

    private boolean rotate(double SP) {
        // restart imu angle tracking.


        double PV1 = getAngle();
        telemetry.addData("Current angle",PV1);

        double degrees = SP;


        power2 = 1.0;

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
        pidRotate3.setSetpoint(degrees);
        pidRotate3.setInputRange(0, degrees);
        pidRotate3.setOutputRange(0, power2);
        pidRotate3.setTolerance(0.000000001);
        pidRotate3.enable();
        pidRotate3.performPID(PV1);

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


        if (!pidRotate3.onTarget()) {

            loopCount = loopCount + 1;
            telemetry.addData("OnTarget", pidRotate3.onTarget());

            Loop = true;

            power2 = (pidRotate3.performPID(PV1)); // power will be - on right turn.

            robot.leftFront.setPower(-power2);
            robot.rightFront.setPower(power2);
            robot.leftBack.setPower(-power2);
            robot.rightBack.setPower(power2);

            return true;
        }

//        while (opModeIsActive() && !pidRotate3.onTarget());
        else if (pidRotate3.onTarget()) {


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


/*
    private boolean PIDTurn(double SP)
    {

        double PV = getAngle();

        pidRotate.setTolerance(1);

        pidRotate.setSetpoint(SP);
        pidRotate.setInput(PV);

        power = (pidRotate.performPID());

        robot.leftFront.setPower(-power);
        robot.rightFront.setPower(power);
        robot.leftBack.setPower(-power);
        robot.rightBack.setPower(power);


        return !pidRotate.onTarget();
    }

    */
/*

    private boolean PIDTurn(double SP){
        double power;
        pidRotate.setTolerance(1);

        double PV = -getAngle();

        double angle = PV - SP;
        pidRotate.setSetpoint(SP);

        double SPSign = SP / Math.abs(SP);
        double PVSign = PV / Math.abs(PV);
        double SPFlip = SP - (180 * SPSign);
        double PVFlip = PV - (180 * PVSign);

        double angle2 = PVFlip - SPFlip;
        pidRotate2.setSetpoint(SPFlip);

        double errorPower = ((SP - PV)/SP);
        double errorPowerFlip = ((SPFlip - PVFlip)/SPFlip);

        if (Math.abs(angle) <= 180){
            power = pidRotate.performPID(PV);
            //power = pidRotate.performPID(errorPower);
        }else {
           power = pidRotate2.performPID(PVFlip);
            //power = pidRotate2.performPID(errorPowerFlip);
        }

//        boolean turned = (SP <= PV + 1 && SP >= PV - 1);

        telemetry.addData("Power" , power);
        telemetry.addData("PV", PV);
        telemetry.addData("PVFlip", PVFlip);
        robot.leftFront.setPower(-power);
        robot.rightFront.setPower(power);
        robot.leftBack.setPower(-power);
        robot.rightBack.setPower(power);

        return !pidRotate.onTarget();
    }
    */
/*
    boolean PIDTurn2(double SP, boolean start) {

        pidRotate.setTolerance(1);
        pidRotate2.setTolerance(1);

        double curAngle = getAngle();

        if (start) {
//            telemetry.addData("HI", "hi");
            pidRotate.reset();
            pidRotate2.reset();

            double SPSign = SP / Math.abs(SP);
            double SPFlip = SP - (180 * SPSign);

//            angle = curAngle - SP;
            pidRotate.setSetpoint(curAngle - SP);
//            angle2 = PVFlip - SPFlip;
            pidRotate2.setSetpoint(curAngle - SPFlip);
            var.resetAngle();
        }

        double PV = var.getAngle();
        double PVSign = curAngle / Math.abs(curAngle);
        double PVFlip = curAngle - (180 * PVSign);

        double power;

        if (Math.abs(pidRotate.getSetpoint()) <= 180){
            power = pidRotate.performPID(PV);
        }else {
            power = pidRotate2.performPID(PVFlip);
        }

        telemetry.addData("Power", power);
        telemetry.addData("PV", PV);
        telemetry.addData("PVFlip", PVFlip);
//        telemetry.addData("Power", power);



//        robot.leftFront.setPower(-power);
//        robot.rightFront.setPower(power);
//        robot.leftBack.setPower(-power);
//        robot.rightBack.setPower(power);

        return !pidRotate.onTarget();

    }*/

    private void drive(double rightStick, double leftStick){
        if (!strafeR && !strafeL && !turning){
            rightStick = -rightStick;
            leftStick = -leftStick;

            robot.leftFront.setPower(leftStick - rightStick);
            robot.rightFront.setPower(leftStick + rightStick);
            robot.leftBack.setPower(leftStick - rightStick);
            robot.rightBack.setPower(leftStick + rightStick);
        }
    }

    private void composeTelemetry() {

        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("DistanceR", robot.distanceR.getDistance(DistanceUnit.MM))
                .addData("DistanceL", robot.distanceL.getDistance(DistanceUnit.MM))
                .addData("DistanceB", robot.distanceB.getDistance(DistanceUnit.MM))
                .addData("DistanceFM", robot.distanceFM.getDistance(DistanceUnit.MM));

    }

    public double getAngle() {
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
    }

    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    private String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
