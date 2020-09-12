package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Initialization.Initialize;
import org.firstinspires.ftc.teamcode.Initialization.Variables;

//import Teamcode.Control.Motor;
//import Teamcode.Control.Sensors;
//import Teamcode.Control.Variables;

@TeleOp(name = "ThreadTest", group = "TeleOp")
public class ThreadTest extends LinearOpMode {
    //private RobotHardwareMap robot = new RobotHardwareMap();
//    private Sensors sensors = new Sensors();
//    private Variables var = sensors.getVar();
//    private RobotHardwareMap robot = var.getRobot();
//    private Motor motor = new Motor(var);
//    private PIDController pidTurn, pidDrive, pidStrafe;

    Variables var = new Initialize().Init(hardwareMap);

    private boolean turning;
    private boolean turningA;
    private boolean turningB;
    private boolean turningX;
    private boolean turningY;
    private boolean begin = true;
    private double beginAngle;
    private boolean beginFlip = true;
    private double beginAngleFlip;
    private boolean reset = true;
    double degrees;
        private boolean turnBegin;

        @Override
        public void runOpMode() {
////        robot.init(hardwareMap);
////        var.init();
//        var.hardware = hardwareMap;

        //Pas the Hardware Map config and initialize it
        var.robot.init(hardwareMap);

        Thread Tsensors = new Thread(var.sensors);
        Tsensors.start();

//        pidTurn = new PIDController(.003, .00003, 0);

        telemetry.addData("Press play to start opmode", 1);
        telemetry.update();
//        composeTelemetry();
        waitForStart();
        telemetry.update();

        while (opModeIsActive()){
//            motor.PIDStrafeRTrigger(gamepad1.right_trigger, gamepad1.right_trigger > 0);
//
//            motor.PIDStrafeLTrigger(gamepad1.left_trigger, gamepad1.left_trigger > 0);
//
//            tankDrive(gamepad1.right_stick_y, gamepad1.left_stick_y);
//
//            boolean stop;
//            if ((gamepad1.a || turningA) && !turningB && !turningX && !turningY){
//                stop = motor.PIDTurn(180,1);
//                turningA = stop;
//            }
//            if ((gamepad1.b || turningB) && !turningA && !turningX && !turningY){
//                stop = motor.PIDTurn(90,1);
//                turningB = stop;
//            }
//            if ((gamepad1.x || turningX) && !turningA && !turningB && !turningY){
//                stop = motor.PIDTurn(-90,1);
//                turningX = stop;
//            }
//            if ((gamepad1.y || turningY) && !turningA && !turningB && !turningX){
//                stop = motor.PIDTurn(0,1);
//                turningY = stop;
//            }
//
//            double liftPower = gamepad2.left_stick_y;
//
//            var.turning = (turningA || turningB || turningX || turningY);
//
////            robot.liftMotor.setPower(liftPower);

//            while (rotate(90, 1))

            telemetry.addLine()

                    .addData("DistanceR", var.getDistanceR())

                    .addData("DistanceL", var.getDistanceL())

                    .addData("DistanceFM", var.getDistanceFM())

                    .addData("DistanceB", var.getDistanceB());

            telemetry.addLine()

                    .addData("Encoders", var.getAvgEncoder());

//            telemetry.addData("power", PIDTurn(-90));
            telemetry.update();

        }
        //To End the Thread
//        var.opModeActive = false;
//
////        var.opModeActive = false;
//    }
//    public void tankDrive(double rightStick, double leftStick){
//        if (!var.strafeR && !var.strafeL && !var.turning){
//            rightStick = -rightStick;
//            leftStick = -leftStick;
//
//            robot.leftFront.setPower(leftStick);
//            robot.rightFront.setPower(rightStick);
//            robot.leftBack.setPower(leftStick);
//            robot.rightBack.setPower(rightStick);
//
//        }
//    }

//    private void composeTelemetry() {

//        telemetry.addAction(new Runnable() { @Override public void run()
//        {
//            // Acquiring the angles is relatively expensive; we don't want
//            // to do that in each of the three items that need that info, as that's
//            // three times the necessary expense.
//            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        }
//        });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return Double.toString(var.getHeading());
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return Double.toString(var.getRoll());
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return Double.toString(var.getPitch());
                    }
                });


    }

//    public double PIDTurn(double SP/*SetPoint*/){
//
////        getVar().PV/*Process Variable*/ = -getVar().heading;
//        double PV = -var.heading;
//
//        double SPFlip = SP - Math.copySign(180, SP);
//        double PVFlip = PV - Math.copySign(180, PV);
//
//        PV = angleFrom0(PV);
//        SP = targetAngleFrom0(SP);
//        PVFlip = angleFrom0Flip(PVFlip);
//        SPFlip = targetAngleFrom0Flip(SPFlip);
//
//        double angle = PV - SP;
//
////        double SPSign = SP / Math.abs(SP);
////        double PVSign = var.PV / Math.abs(var.PV);
//
//        if (reset){
//            pidTurn.reset();
//            if (Math.abs(angle) <= 180){
//                pidTurn.setSetpoint(SP);
//            }else {
//                pidTurn.setSetpoint(SPFlip);
//            }
//            pidTurn.setInputRange(0, SP);
//            pidTurn.setOutputRange(0, 1);
//            pidTurn.setTolerance(1);
//            pidTurn.enable();
//            reset = false;
//        }
//
//        double power;
//        if (Math.abs(angle) <= 180){
//            power = pidTurn.performPID(PV);
//        }else {
//            power = pidTurn.performPID(PVFlip);
//        }
//
//        boolean turned = pidTurn.onTarget();
//
////        robot.leftFront.setPower(power);
////        robot.rightFront.setPower(-power);
////        robot.leftBack.setPower(power);
////        robot.rightBack.setPower(-power);
//
//        reset = turned;
//        return power;
//    }

    private double angleFrom0(double curAngle){
        if (begin){
            beginAngle = curAngle;
            begin = false;
        }
        return curAngle - beginAngle;
    }

    private double getDegrees(double SP) {
            double PV = -var.getHeading();
            // calculate the degrees the robot has to turn
            degrees = PV - SP;
            // flip the circle if the degrees the robot has to turn is greater than 180 degrees
            if (degrees > 180) {
                degrees = (PV - Math.copySign(180, PV)) - (SP - Math.copySign(180, SP));
            }
        return degrees;
    }

    /* This method works with the IMU to rotate the robot to a specific point.
    *  We also used the PidController class.
    * */

//    private boolean rotate(double degrees, double power)
//    {
//
//        if (reset) {
//            // convert the target degrees to degrees the robot has to turn
//            // if the robot has to turn over the 180 -180 point, flip the whole turning circle
//            // so the math is easier.
//            degrees = getDegrees(degrees);
//            // restart imu angle tracking.
//            sensors.resetAngle();
//
//            // if degrees > 359 we cap at 359 with same sign as original degrees.
//
//            // start pid controller. PID controller will monitor the turn angle with respect to the
//            // target angle and reduce power as we approach the target angle. This is to prevent the
//            // robots momentum from overshooting the turn after we turn off the power. The PID controller
//            // reports onTarget() = true when the difference between turn angle and target angle is within
//            // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
//            // dependant on the motor and gearing configuration, starting power, weight of the robot and the
//            // on target tolerance. If the controller overshoots, it will reverse the sign of the output
//            // turning the robot back toward the setpoint value.
//
//            pidTurn.reset();
//            pidTurn.setSetpoint(degrees);
//            pidTurn.setInputRange(0, degrees);
//            pidTurn.setOutputRange(0, power);
//            pidTurn.setTolerance(1);
//            pidTurn.enable();
//            reset = false;
//        }
//
//        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
//        // clockwise (right).
//
//        // rotate until turn is completed.
//
//        boolean turned = pidTurn.onTarget();
//
//        if (!turned) {
//            power = pidTurn.performPID(var.getGlobalAngle());
//
//            robot.leftFront.setPower(-power);
//            robot.rightFront.setPower(power);
//            robot.leftBack.setPower(-power);
//            robot.rightBack.setPower(power);
//        }else {
//            // turn the motors off.
//            robot.leftFront.setPower(0);
//            robot.rightFront.setPower(0);
//            robot.leftBack.setPower(0);
//            robot.rightBack.setPower(0);
//
//            // reset angle tracking on new heading.
//            sensors.resetAngle();
//            reset = false;
//        }
//
//        return turned;
//    }

    private double targetAngleFrom0(double target){
        return target - beginAngle;
    }

    private double angleFrom0Flip(double curAngle){
        if (beginFlip){
            beginAngleFlip = curAngle;
            beginFlip = false;
        }
        return curAngle - beginAngleFlip;
    }

    private double targetAngleFrom0Flip(double target){
        return target - beginAngleFlip;
    }


//    public RobotHardwareMap getRobot() {
//        return robot;
//    }

}
