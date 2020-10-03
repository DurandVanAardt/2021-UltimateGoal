package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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
    private boolean strafeL;
    private boolean strafeR;
    private boolean turning;
    private boolean turningA;
    private boolean turningB;
    private boolean turningX;
    private boolean turningY;
    PIDController pidRotate, pidRotate2, pidDrive, pidStrafe;


    @Override
    public void runOpMode() {
        var = new Initialize().Init(hardwareMap);
        motors = var.motors;
        robot = var.robot;

        pidRotate = new PIDController(.003, 1, 0);
        pidRotate.setOutputRange(-1, 1);
        pidRotate.setInputRange(-180, 180);
        pidRotate.enable();

        pidRotate2 = new PIDController(.003, .00003, 0);
        pidRotate2.setOutputRange(-1, 1);
        pidRotate2.setInputRange(-180, 180);
        pidRotate2.enable();

        pidStrafe = new PIDController(.05,0,0);
        pidStrafe.setOutputRange(0, 0.3);
        pidStrafe.setInputRange(-90, 90);
        pidStrafe.enable();

        composeTelemetry();
        waitForStart();

        while (opModeIsActive()){
                PIDStrafeRTrigger(gamepad1.right_trigger, gamepad1.right_trigger > 0);

                PIDStrafeLTrigger(gamepad1.left_trigger, gamepad1.left_trigger > 0);

                drive(gamepad1.right_stick_x, gamepad1.left_stick_y);

            boolean stop;
            if ((gamepad1.a || turningA) && !turningB && !turningX && !turningY){
                    stop = PIDTurn(180);
                    turningA = stop;
                }
                if ((gamepad1.b || turningB) && !turningA && !turningX && !turningY){
                    stop = PIDTurn(90);
                   turningB = stop;
                }
                if ((gamepad1.x || turningX) && !turningA && !turningB && !turningY){
                    stop = PIDTurn(-90);
                    turningX = stop;
                }
                if ((gamepad1.y || turningY) && !turningA && !turningB && !turningX){
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

    private boolean PIDStrafeLToWall(double power, double wallDistance){
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
        
        if (distanceReached){
            start = true;

            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightFront.setPower(0);
        }

        return !distanceReached;
    }

    private boolean PIDStrafeLFromWall(double power, double wallDistance){
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

        if (distanceReached){
            start = true;

            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightFront.setPower(0);
        }

        return !distanceReached;
    }

    private boolean PIDStrafeRToWall(double power, double wallDistance){
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

        if (distanceReached){
            start = true;

            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightFront.setPower(0);
        }

        return !distanceReached;
    }

    private boolean PIDStrafeRFromWall(double power, double wallDistance){
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

        if (distanceReached){
            start = true;

            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightFront.setPower(0);
        }

        return !distanceReached;
    }

    private void PIDStrafeLTrigger(double power, boolean strafe){
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

    private void PIDStrafeRTrigger(double power, boolean strafe){
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
            power = pidRotate.performPID(PV);
        }else {
            power = pidRotate2.performPID(PV);
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
