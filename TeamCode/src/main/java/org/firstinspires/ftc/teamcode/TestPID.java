package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    private double SP;
    private double PV;
    private double errorPower;
    private boolean strafeL;
    private boolean strafeR;
    private boolean turning;
    private boolean turningA;
    private boolean turningB;
    private boolean turningX;
    private boolean turningY;
    private double rotation;
    PIDController pidRotate, pidRotate3, pidRotate2, pidDrive, pidStrafe;


    @Override
    public void runOpMode() {
        var = new Initialize().Init(hardwareMap);
        motors = var.motors;
        robot = var.robot;

        pidRotate = new PIDController(.003, .00003, 0);
        pidRotate.setOutputRange(-1, 1);
        pidRotate.setInputRange(-180, 180);
        pidRotate.enable();

        pidRotate2 = new PIDController(.003, .00003, 0);
        pidRotate2.setOutputRange(-1, 1);
        pidRotate2.setInputRange(-180, 180);
        pidRotate2.enable();

        pidRotate3 = new PIDController(.003, .00003, 0);
        pidRotate3.setOutputRange(-1, 1);
        pidRotate3.setInputRange(-180, 180);
        pidRotate3.enable();

        pidStrafe = new PIDController(.05, 0, 0);
        pidStrafe.setOutputRange(0, 0.3);
        pidStrafe.setInputRange(-90, 90);
        pidStrafe.enable();

        composeTelemetry();
        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.left_trigger!= 0 ) {
                motors.driveStrafe(45 * Math.PI / 180, 1, true);
            }
            if(gamepad1.right_trigger!= 0 ) {
                motors.driveStrafe(-45 * Math.PI / 180, 1 , true);
            }
            PIDStrafeRTrigger(gamepad1.right_trigger, gamepad1.right_trigger > 0);

            PIDStrafeLTrigger(gamepad1.left_trigger, gamepad1.left_trigger > 0);

            drive(gamepad1.right_stick_x, gamepad1.left_stick_y);

            errorPower = ((SP - PV)/SP);

            boolean stop;
            if ((gamepad1.a || turningA) && !turningB && !turningX && !turningY) {
                stop = PIDTurn(180);
                turningA = stop;
            }
            if ((gamepad1.b || turningB) && !turningA && !turningX && !turningY) {
                stop = PIDTurn(90);
                turningB = stop;
            }
            if ((gamepad1.x || turningX) && !turningA && !turningB && !turningY) {
                stop = PIDTurn(-90);
                turningX = stop;
            }
            if ((gamepad1.y || turningY) && !turningA && !turningB && !turningX) {
                stop = PIDTurn(0);
                turningY = stop;
            }


            double liftPower = gamepad2.left_stick_y;

            turning = (turningA || turningB || turningX || turningY);

            robot.liftMotor.setPower(liftPower);

            telemetry.addData("FM", robot.distanceFM.getDistance(DistanceUnit.MM));
            telemetry.addData("B", robot.distanceB.getDistance(DistanceUnit.MM));
            telemetry.addData("L", robot.distanceL.getDistance(DistanceUnit.MM));
            telemetry.addData("R", robot.distanceR.getDistance(DistanceUnit.MM));
            telemetry.update();
        }

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

//    private boolean  pidRotate3(int degrees, double power) {
//        // restart imu angle tracking.
//      //resetAngle();
//
//        // if degrees > 359 we cap at 359 with same sign as original degrees.
//        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);
//
//        // start pid controller. PID controller will monitor the turn angle with respect to the
//        // target angle and reduce power as we approach the target angle. This is to prevent the
//        // robots momentum from overshooting the turn after we turn off the power. The PID controller
//        // reports onTarget() = true when the difference between turn angle and target angle is within
//        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
//        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
//        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
//        // turning the robot back toward the setpoint value.
//
//        pidRotate.reset();
//        pidRotate.setSetpoint(degrees);
//        pidRotate.setInputRange(0, degrees);
//        pidRotate.setOutputRange(0, power);
//        pidRotate.setTolerance(1);
//        pidRotate.enable();
//
//        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
//        // clockwise (right).
//
//        // rotate until turn is completed.
//
//        if (degrees < 0) {
//            // On right turn we have to get off zero first.
//            while (/*opModeIsActive()*/ getAngle() == 0) {
//                robot.leftFront.setPower(power);
//                robot.rightFront.setPower(-power);
//                sleep(100);
//            }
//
//            do {
//                power =  pidRotate3.performPID(getAngle()); // power will be - on right turn.
//                robot.leftFront.setPower(-power);
//                robot.rightFront.setPower(power);
//            } while ( /*opModeIsActive()*/! pidRotate3.onTarget());
//        } else    // left turn.
//            do {
//                power =  pidRotate3.performPID(getAngle()); // power will be + on left turn.
//                robot.leftFront.setPower(-power);
//                robot.rightFront.setPower(power);
//            } while (/*opModeIsActive()*/  ! pidRotate3.onTarget());
//
//        // turn the motors off.
//        robot.leftFront.setPower(0);
//        robot.rightFront.setPower(0);
//
//        rotation = getAngle();
//
//        // wait for rotation to stop.
//        sleep(500);
//
//        boolean turned = (SP <= PV + 1 && SP >= PV - 1);
//
//        robot.leftFront.setPower(-power);
//        robot.rightFront.setPower(power);
//        robot.leftBack.setPower(-power);
//        robot.rightBack.setPower(power);
//
//        return ! pidRotate3.onTarget();
//    }





    private boolean PIDTurn(double SP/*SetPoint*/){
        double power;
        pidRotate.setTolerance(1);

        double PV/*Process Variable*/ = -getAngle();

        double angle = PV - SP;
        pidRotate.setSetpoint(SP);

        double SPSign = SP / Math.abs(SP);
        double PVSign = PV / Math.abs(PV);
        double SPFlip = SP - (180 * SPSign);
        double PVFlip = PV - (180 * PVSign);

        double angle2 = PVFlip - SPFlip;
        pidRotate2.setSetpoint(SPFlip);

        if (Math.abs(angle) <= 180){
            //power = pidRotate.performPID(PV);
            power = pidRotate.performPID(errorPower);
        }else {
            //power = pidRotate2.performPID(PV);
            power = pidRotate2.performPID(errorPower);
        }

//        boolean turned = (SP <= PV + 1 && SP >= PV - 1);

        robot.leftFront.setPower(-power);
        robot.rightFront.setPower(power);
        robot.leftBack.setPower(-power);
        robot.rightBack.setPower(power);

        return !pidRotate.onTarget();
    }

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
