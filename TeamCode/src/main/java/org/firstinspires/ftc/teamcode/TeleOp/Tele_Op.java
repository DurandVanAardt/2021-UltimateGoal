package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Initialization.Initialize;
import org.firstinspires.ftc.teamcode.Initialization.Variables;
import org.firstinspires.ftc.teamcode.Resources.Motors;
import org.firstinspires.ftc.teamcode.Resources.RobotHardwareMap;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Tele_Op", group = "TeleOp")
public class Tele_Op extends OpMode {

    Variables var;
    Motors motors;
    RobotHardwareMap robot;

    private boolean turnFirst = true;
    private boolean begin;

    private boolean prevx;
    private boolean prevy;

    private Shooter curShooterState = Shooter.SHOOTERREST;
    private Tapper curTapperState = Tapper.TAPDEFAULT;
    private DriveTrain curDriveTrainState = DriveTrain.STOP;
    private DriveTrain prevDriveState = DriveTrain.STOP;
    private Shooter curCollectionState = Shooter.INTAKEREST;
    boolean targetVisible = false;

    // Vuforia
    VuforiaTrackable lastTrackable;
    private boolean preva;

    @Override
    public void init() {
        var = new Initialize().Init(hardwareMap);
        motors = var.motors;

        robot = var.robot;

        var.targetsUltimateGoal.activate();
        telemetry.addData("Press Play to start the program", "");
    }

    @Override
    public void loop() {
        // check all the trackable targets to see which one (if any) is visible.
        for (VuforiaTrackable trackable : var.allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;
                lastTrackable = trackable;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    var.lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = var.lastLocation.getTranslation();
            telemetry.addData("Pos (mm)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0), translation.get(1), translation.get(2));

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(var.lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        } else {
            telemetry.addData("Visible Target", "none");
        }


        if (var.tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = var.tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                }
            } else
                telemetry.addData("# Object Detected", 0);
        }
        telemetry.addData("Imu", var.getAngle());
        telemetry.addData("Colour", robot.colourF.alpha());
        readInputs();

        telemetry.update();
    }

    @Override
    public void stop() {
    }

    public void readInputs() {


        // Move robot to the designated shooting area
//        if (gamepad1.dpad_up) {
//            while (robot.distanceR.getDistance(DistanceUnit.MM) > 450) {
//                motors.driveStrafe(135 * Math.PI / 180, 0.4, true);
//
//            }
//            motors.stop();
////            telemetry.addData("Right", robot.distanceR.getDistance(DistanceUnit.MM));
//
//            if (robot.distanceB.getDistance(DistanceUnit.MM) < 450) {
//                while (robot.distanceB.getDistance(DistanceUnit.MM) < 450) {
//                    robot.leftFront.setPower(0.75);
//                    robot.leftBack.setPower(0.75);
//                    robot.rightFront.setPower(0.75);
//                    robot.rightBack.setPower(0.75);
//                }
//            }
//            else {
//                while (robot.distanceB.getDistance(DistanceUnit.MM) > 450) {
//                    robot.leftFront.setPower(-0.75);
//                    robot.leftBack.setPower(-0.75);
//                    robot.rightFront.setPower(-0.75);
//                    robot.rightBack.setPower(-0.75);
//                }
//            }
//            motors.stop();

//        }


        telemetry.addData("DistanceL", robot.distanceL.getDistance(DistanceUnit.MM));
        telemetry.addData("DistanceB", robot.distanceB.getDistance(DistanceUnit.MM));
        telemetry.addData("DistanceR", robot.distanceR.getDistance(DistanceUnit.MM));

        if (gamepad2.left_stick_button) {

            robot.magazineLifter.setPosition(0);
            robot.shooterMotor.setPower(-1);

            if (curTapperState == Tapper.TAPDEFAULT && gamepad2.left_stick_button) {

                curTapperState = (Tapper.TAPRIGHT);
                Tapperstate(Tapper.TAPRIGHT);
            }

        }

        if (gamepad2.right_stick_button) {

            robot.shooterMotor.setPower(0);
            robot.magazineLifter.setPosition(0.4);
        }

        if (gamepad2.dpad_up)
            robot.magazineLifter.setPosition(0);

        if (gamepad2.dpad_down)
            robot.magazineLifter.setPosition(0.4);

        if (gamepad2.right_trigger != 0) {
            robot.wobbleMotor.setPower(gamepad2.right_trigger * 0.4);
        }
        if (gamepad2.left_trigger != 0) {
            robot.wobbleMotor.setPower(-gamepad2.left_trigger * 0.4);
        }

        telemetry.addData("Hallo", robot.wobbleMotor.getCurrentPosition());

        if (gamepad2.y) {
            while (robot.wobbleMotor.getCurrentPosition() > 200) {
                robot.wobbleMotor.setPower(-0.5);
            }
                robot.wobbleMotor.setPower(0);
        }
        if (gamepad2.b) {
            while (robot.wobbleMotor.getCurrentPosition() < 350) {
                robot.wobbleMotor.setPower(0.5);
            }
                robot.wobbleMotor.setPower(0);

        }
//
//        if (gamepad2.dpad_right)
//            robot.Tap.setPosition(0.4);
//
//        if (gamepad2.left_trigger != 0)
//            robot.Tap.setPosition(0);
//
//        if (gamepad2.left_trigger == 0)
//            robot.Tap.setPosition(0.4);
        if (gamepad1.a) {
            robot.wobbleF.setPosition(0.3);
            robot.wobbleB.setPosition(0.2);

        }

        if (gamepad1.b) {
            robot.wobbleF.setPosition(0);
            robot.wobbleB.setPosition(-0.3);

        }



        if (gamepad2.dpad_left)
            robot.Tap.setPosition(0);
        else
            robot.Tap.setPosition(0.4);

if (gamepad1.dpad_down)
{
    while (robot.distanceR.getDistance(DistanceUnit.MM) >=440)
    {
        motors.driveStrafe(135 * Math.PI / 180, 0.4, true);
    }
    var.resetAngle();
    motors.pidRotate.reset();
    motors.pidRotate.enable();


    motors.rotate(-3.5);

    while (!motors.pidRotate.onTarget())
        motors.rotate(-3.5);

    telemetry.addData("IMU", var.getAngle());

    while (robot.distanceB.getDistance(DistanceUnit.MM) < 700) {
        robot.leftFront.setPower(0.1);
        robot.leftBack.setPower(0.1);
        robot.rightFront.setPower(0.1);
        robot.rightBack.setPower(0.1);
    }

}



        if (gamepad2.a && curShooterState == Shooter.SHOOTERREST && !preva) {
            curShooterState = Shooter.FIRE;
            shooterState(Shooter.FIRE);

        } else if (gamepad2.a && curShooterState == Shooter.FIRE && !preva) {

            curShooterState = Shooter.SHOOTERREST;
            shooterState(Shooter.SHOOTERREST);
        }

        preva = gamepad2.a;

        if (gamepad1.y && curCollectionState == Shooter.INTAKEREST && !prevy) {
            curCollectionState = Shooter.SUCKERIN;
            shooterState(Shooter.SUCKERIN);
        } else if (gamepad1.y && curCollectionState == Shooter.SUCKERIN && !prevy) {
            curCollectionState = Shooter.INTAKEREST;
            shooterState(Shooter.INTAKEREST);
        }

        prevy = gamepad1.y;

        if (gamepad1.x && curCollectionState == Shooter.INTAKEREST && !prevx) {
            curCollectionState = Shooter.SUCKEROUT;
            shooterState(Shooter.SUCKERIN);
        } else if (gamepad1.x && curCollectionState == Shooter.SUCKEROUT && !prevx) {
            curCollectionState = Shooter.INTAKEREST;
            shooterState(Shooter.INTAKEREST);
        }

        prevx = gamepad1.x;


        if ((gamepad1.left_stick_x != 0) || (gamepad1.left_stick_y != 0) || (gamepad1.right_stick_x != 0)) {

            curDriveTrainState = DriveTrain.DRIVE;

        } else if (gamepad1.left_trigger != 0)

            curDriveTrainState = DriveTrain.STRAFEL;

        else if (gamepad1.right_trigger != 0)
            curDriveTrainState = DriveTrain.STRAFER;

        else motors.stop();


        driveTrainState(curDriveTrainState, prevDriveState);


//        telemetry.addData("TAP", robot.Tap.getPosition());
//        telemetry.addData("Colour", robot.colourF.alpha());
    }

    private void driveTrainState(DriveTrain driveTrain, DriveTrain prevState) {

        switch (driveTrain) {

            case MOVEMIDDLE:
                // I want to go to the middle of the picture I'm seeing. I need to get the current position, and determine what direction to strafe.
                // First, I need to determine which VuMarker I'm seeing. Then I need to get the coordinate from a case statement
                // input the angle to our fancy strafe program "motors.strafe()" and voila

                double curX = var.lastLocation.getTranslation().get(0);
                double curY = var.lastLocation.getTranslation().get(1);

                double targetX = 0;
                double targetY = 0;

                Direction direction = Direction.FRONT;
                switch (lastTrackable.getName()) {
                    case "Trackable 1": // Front wall, so ignore y value
                        targetX = 100; // Just an example. Will be fixed in the future
                        direction = Direction.FRONT;
                        break;
                    case "Trackable 2": // Back wall, so ignore y value
                        targetX = 200; // Again, just an example. Don't kill me
                        direction = Direction.BACK;
                        break;
                    case "Trackable 3": // Left wall, so ignore x value
                        targetY = 100;
                        direction = Direction.LEFT;
                        break;
                    case "Trackable 4": // Right wall, so ignore x value
                        targetY = 200;
                        direction = Direction.RIGHT;
                        break;
                    default:
                        targetX = 0; // Just to keep the warnings at bay
                        targetY = 0;
                }

                // This might change, I don't know if the Math.atan function returns values in the same way our strafe program uses it.
                // Might need to do some post-processing
                // Also need to take into account the angle of the robot. Will do some further thinking, come back tomorrow with fresh eyes.
                /*2021/6/11*/

                double deltaX = (targetX - curX);
                double deltaY = (targetY - curY);

                double angle = 0;
                switch (direction) {
                    case FRONT:
                        angle = 0;
                        break;
                    case BACK:
                        angle = 180;
                        break;
                    case RIGHT:
                        angle = -90;
                        break;
                    case LEFT:
                        angle = 90;
                        break;
                }

                int tolerance = 1; // the tolerance when we want the robot to stop
                boolean destination = curX < (targetX + tolerance) && curX > (targetX - tolerance) && curY < (targetY + tolerance) && curY > (targetY - tolerance);

                if (!destination) {
                    motors.strafe(angle, 0.5, begin);
                }

                if (begin) {
                    begin = false;
                }
                break;


            case STRAFEL:
                motors.strafe(-90, gamepad1.left_trigger, begin);

                if (begin)
                    begin = false;
                break;

            case STRAFER:
                motors.strafe(90, gamepad1.right_trigger, begin);

                if (begin)
                    begin = false;

                break;

            case DRIVE:

                boolean startDrive = prevState != DriveTrain.STOP;

                motors.mecanum(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, startDrive);


//                if (begin)
//                    begin = false;

                break;


            case TURNUP:

                boolean turningUp;
                if (turnFirst) {

                    var.resetAngle();

                    turnFirst = false;
                    motors.pidRotate.reset();
                    motors.pidRotate.enable();
                }

                turningUp = motors.rotate(0);


                turnFirst = !turningUp;

                break;

            case TURNDOWN:

                boolean turningDown;
                if (turnFirst) {
                    var.resetAngle();
                    turnFirst = false;
                    motors.pidRotate.reset();
                    motors.pidRotate.enable();
                }

                turningDown = motors.rotate(180);

                turnFirst = !turningDown;

                break;

            case TURNLEFT:

                boolean turningLeft;
                if (turnFirst) {
                    var.resetAngle();
                    turnFirst = false;
                    motors.pidRotate.reset();
                    motors.pidRotate.enable();
                }

                turningLeft = motors.rotate(90);

                turnFirst = !turningLeft;

                break;

            case TURNRIGHT:

                boolean turningRight;
                if (turnFirst) {
                    var.resetAngle();
                    turnFirst = false;
                    motors.pidRotate.reset();
                    motors.pidRotate.enable();
                }

                turningRight = motors.rotate(-90);

                turnFirst = !turningRight;

                break;

            case LEFTTOPOS:

                motors.LEFTTOPOS(gamepad1.left_bumper);

                break;

            case RIGHTTOPOS:

                motors.RIGHTTOPOS(gamepad1.right_bumper);

                break;

            default:

                motors.stop();

                begin = true;

                break;
        }
    }

    private void Tapperstate(Tapper tapper) {

        switch (tapper) {
            case TAPRIGHT:
                robot.Tap.setPosition(0);

            case TAPDEFAULT:
                robot.Tap.setPosition(0.4);
        }
    }

    private void
    shooterState(Shooter shooter) {

        switch (shooter) {
            case TAPDEFAULT:
                robot.Tap.setPosition(0.4);
                break;

            case TAPRIGHT:
                robot.Tap.setPosition(0);
                break;

            case FIRE:

                robot.shooterMotor.setPower(-1);

                break;

            case ADJUSTANGLE:

                motors.adjustAngle(gamepad2.b);


                break;

            case SUCKERIN:


                motors.suckerIn(gamepad1.left_bumper);


                break;

            case SUCKEROUT:

                motors.suckerOut();


                break;

            case SHOOTERREST:

                robot.shooterMotor.setPower(0);
                break;

            case INTAKEREST:

                motors.intakeStop();

        }
    }
}
