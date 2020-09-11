package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Resources.RobotHardwareMap;

import java.util.Locale;

@TeleOp(name = "Strafe_1", group = "TeleOp")
@Disabled
public class Strafe_1 extends LinearOpMode {

    private RobotHardwareMap robot = new RobotHardwareMap(hardwareMap);

    private double speedControl;
    //    private double angle;
//    private double correction;
//    private double gain = 10;
    private BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle;

    {
        speedControl = 1;
    }

    public Strafe_1() {
    }

    public void runOpMode() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode              = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit         = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit         = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled    = false;

        imu =  hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.left_bumper) {
                speedControl = 0.75;
            }
            if (gamepad1.right_bumper) {
                speedControl = 1;
            }

//            getAngle();
//            telemetry.update();
//            double speed = 0.4;
//            if (gamepad1.a) {
//
//                if (getAngle() < 0) {
//                    while (!(getAngle() <= 1 && getAngle() >= -1) || gamepad1.b) {
//                        robot.leftBack.setPower(speed);
//                        robot.leftFront.setPower(speed);
//                        robot.rightBack.setPower(-speed);
//                        robot.rightFront.setPower(-speed);
//                    }
//                } else {
//                    if (getAngle() > 0) {
//                        while (!(getAngle() <= 1 && getAngle() >= -1) || gamepad1.b) {
//                            robot.leftBack.setPower(-speed);
//                            robot.leftFront.setPower(-speed);
//                            robot.rightBack.setPower(speed);
//                            robot.rightFront.setPower(speed);
//
//                        }
//
//                    }
//                }
//            }

            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            double v1 = r * Math.sin(robotAngle) - rightX;
            double v2 = r * Math.cos(robotAngle) + rightX;
            double v3 = r * Math.cos(robotAngle) - rightX;
            double v4 = r * Math.sin(robotAngle) + rightX;



            robot.leftFront.setPower(v1 * speedControl);
            robot.rightFront.setPower(v2 * speedControl);
            robot.leftBack.setPower(v3 * (speedControl));
            robot.rightBack.setPower(v4 * speedControl);

            if (gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0 && gamepad1.right_stick_y == 0 && gamepad1.right_stick_y == 0) {
                robot.leftFront.setPower(0);
                robot.rightFront.setPower(0);
                robot.leftBack.setPower(0);
                robot.rightBack.setPower(0);
            }

        }
    }

    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.

        double angle;
        double gain = 10;

        angle = getAngle();

        double correction;
        if (angle == 0)
            correction = 0;             // no adjustment.
        else {
            correction = -angle;        // reverse sign of angle for correction.
        }

        correction = correction * gain;

        return correction;
    }

    String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        telemetry.addData("Heading R-", formatAngle(lastAngles.angleUnit, lastAngles.firstAngle));
        telemetry.addData("Roll L+", formatAngle(lastAngles.angleUnit, lastAngles.secondAngle));
        telemetry.addData("Pitch U-", formatAngle(lastAngles.angleUnit, lastAngles.thirdAngle));
        telemetry.addData("Global Heading", formatAngle(lastAngles.angleUnit, globalAngle));
        telemetry.update();
        return globalAngle;
    }

    public void turn(double angle){
            if (angle < getAngle()) {
                while (!(getAngle() == angle)) {
                    robot.leftFront.setPower(-0.4);
                    robot.leftBack.setPower(-0.4);
                    robot.rightFront.setPower(0.4);
                    robot.rightBack.setPower(0.4);
                }
            }else if(angle > getAngle()){
                while (!(getAngle() == angle)) {
                    robot.leftFront.setPower(0.4);
                    robot.leftBack.setPower(0.4);
                    robot.rightFront.setPower(-0.4);
                    robot.rightBack.setPower(-0.4);

                }

            }
    }

}

