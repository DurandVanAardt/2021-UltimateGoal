package org.firstinspires.ftc.teamcode.TeleOp;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Initialization.Variables;
import org.firstinspires.ftc.teamcode.Initialization.Initialize;
import org.firstinspires.ftc.teamcode.Resources.Motors;
import org.firstinspires.ftc.teamcode.Resources.RobotHardwareMap;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;


@TeleOp(name = "Test", group = "TeleOp")

public class Test extends OpMode {

    Variables var;
    Motors motors;
    RobotHardwareMap robot;

//    DriveTrain driveTrain = DriveTrain.STOP;
//    Shooter shooter = Shooter.SHOOTERREST;

    private boolean turnFirst = true;
    private boolean begin;
//    private boolean begin1;

//    private boolean turning = false;
//    private boolean rotationShooter;
//    private boolean turningUp;
//    private boolean turningDown;
//    private boolean turningLeft;
//    private boolean turningRight;

    private Shooter curCollectionState = Shooter.INTAKEREST;
//    private Shooter prevCollectionState =Shooter.INTAKEREST;

    private Shooter curShooterState = Shooter.SHOOTERREST;
//    private Shooter prevShooterState = Shooter.SHOOTERREST;
    private DriveTrain curDriveTrainState = DriveTrain.STOP;

//    int Counter= 0;
//    boolean shooterLoop = true;

    // Vuforia
    VuforiaTrackable lastTrackable;

    boolean targetVisible = false;

    // testing
    int HI = 0;

    @Override
    public void init() {
        var = new Initialize().Init(hardwareMap);
        motors = var.motors;

        robot = var.robot;

        var.targetsUltimateGoal.activate();
        telemetry.addData("Press Play to start the program","");
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
//         waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.


        // check all the trackable targets to see which one (if any) is visible.
        for (VuforiaTrackable trackable : var.allTrackables) {
            HI ++;
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;
                lastTrackable = trackable;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
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
        }
        else {
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
            }else
                telemetry.addData("# Object Detected", 0);
        }

        telemetry.update();


        // Disable Tracking when we are done;

//        if (gamepad1.b) {
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
        readInputs();
        //        if (gamepad1.a){
//            robot.leftFront.setPower(-0.5);
//            robot.leftBack.setPower(-0.5);
//            robot.rightFront.setPower(0.5);
//            robot.rightBack.setPower(0.5);
//
//        }

//        readInputs();

//        telemetry.addData("Current state", curShooterState);

//
//        else if (gamepad1.dpad_up || turningUp)
//            driveTrain = DriveTrain.TURNUP;
//
//        else if (gamepad1.dpad_down || turningDown)
//            driveTrain = DriveTrain.TURNDOWN;
//
//        else if (gamepad1.dpad_left || turningLeft)
//            driveTrain = DriveTrain.TURNLEFT;
//
//        else if (gamepad1.dpad_right || turningRight)
//            driveTrain = DriveTrain.TURNRIGHT;
//
//        if (gamepad1.right_trigger != 0)
//            driveTrain = DriveTrain.STRAFER;
//        else
//        if (gamepad1.left_trigger != 0)
//            driveTrain = DriveTrain.STRAFEL;
//        else
//        if (gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0 || gamepad1.right_stick_x != 0)
//            driveTrain = DriveTrain.DRIVE;
//        else
//            driveTrain = DriveTrain.STOP;

//        stateMachine(driveTrain, shooter);

//        telemetry.addData("State", driveTrain);
//
    }

    @Override
    public void stop() {
        var.targetsUltimateGoal.deactivate();

        if (var.tfod != null) {
            var.tfod.shutdown();
        }
    }

    public void readInputs() {
//robot.wobbleLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//if ((gamepad2.b) && curShooterState == Shooter.SHOOTERREST)
//{
//
//    robot.wobbleLifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//robot.wobbleLifter.setTargetPosition(1000);
//robot.wobbleLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//}

//        if ((gamepad2.x) && curShooterState == Shooter.SHOOTERREST)
//        {
//            robot.wobbleLifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.wobbleLifter.setTargetPosition(1200);
//            robot.wobbleLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        }
//
//        if ((gamepad2.y) && curShooterState == Shooter.SHOOTERREST)
//        {
//            robot.wobbleLifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.wobbleLifter.setTargetPosition(0);
//            robot.wobbleLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        }


        robot.wobbleMotor.setPower(gamepad2.left_trigger);

        robot.wobbleMotor.setPower(-gamepad2.right_trigger);


        if (gamepad2.dpad_up) {
            robot.wobbleLifter.setPosition(0);
        }
        if (gamepad2.dpad_down) {
            robot.wobbleLifter.setPosition(0.45);
        }

//        prevShooterState = curShooterState;

         if(gamepad2.a &&  curShooterState == Shooter.SHOOTERREST) 

        {
            curShooterState = Shooter.FIRE;
            shooterState(Shooter.FIRE);

        }


        else if(gamepad2.x && curShooterState == Shooter.FIRE){

            curShooterState = Shooter.SHOOTERREST;
            shooterState(Shooter.SHOOTERREST);
        }

//        if (gamepad2.b)
//        {
//
//         curShooterState = Shooter.ADJUSTANGLE;
//            shooterState(Shooter.ADJUSTANGLE);
//
//        }

        if(gamepad2.left_bumper && curCollectionState == Shooter.INTAKEREST)
        {
            curCollectionState = Shooter.SUCKERIN;
            shooterState(Shooter.SUCKERIN);
        }

        else if(gamepad2.y) {
            curCollectionState = Shooter.INTAKEREST;
            shooterState(Shooter.INTAKEREST);
        }

        if(gamepad2.right_bumper && curCollectionState == Shooter.INTAKEREST)
        {
            curCollectionState = Shooter.SUCKEROUT;
            shooterState(Shooter.SUCKEROUT);
        }

        else if(gamepad2.y)
        {
            curCollectionState = Shooter.INTAKEREST;
            shooterState(Shooter.INTAKEREST);
        }

    if ((gamepad1.left_stick_x !=0) || (gamepad1.left_stick_y!=0) || (gamepad1.right_stick_x!=0))
        {

            curDriveTrainState = DriveTrain.DRIVE;

        }

    else if (gamepad1.left_trigger!=0)

        curDriveTrainState = DriveTrain.STRAFEL;

    else if (gamepad1.right_trigger!=0)
            curDriveTrainState = DriveTrain.STRAFER;

    else
        curDriveTrainState = DriveTrain.STOP;

        driveTrainState(curDriveTrainState, prevDriveState);
// else if ((gamepad1.dpad_up &&  (turnFirst==true))) {
//
//            driveTrainState(DriveTrain.TURNUP);
//        }
//else if ((gamepad1.dpad_left &&  (turnFirst==true)))
//
//    driveTrainState(DriveTrain.TURNLEFT);
//
//else if ((gamepad1.dpad_right &&  (turnFirst==true)))
//
//    driveTrainState(DriveTrain.TURNRIGHT);
//
//else if ((gamepad1.dpad_down &&  (turnFirst==true)))
//
//    driveTrainState(DriveTrain.TURNDOWN);
//
//else {
//    driveTrainState((driveTrain.TURNUP.STOP));
//    driveTrainState((driveTrain.TURNDOWN.STOP));
//    driveTrainState((driveTrain.TURNLEFT.STOP));
//    driveTrainState((driveTrain.TURNRIGHT.STOP));
//
//}


//        if (gamepad1.left_trigger!=0)
//        {
//            driveTrainState(DriveTrain.STRAFEL);
//
//        }
//        else  driveTrainState(DriveTrain.STRAFEL.STOP);
//
//
//        if (gamepad1.right_trigger!=0)
//        {
//            driveTrainState(DriveTrain.STRAFER);
//        }
//        else  driveTrainState(DriveTrain.STRAFER.STOP);
    }

//    private void stateMachine(DriveTrain driveTrain, Shooter shooter) {
//        driveTrainState(driveTrain);
//        shooterState(shooter);
//    }

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

                double deltaX = (targetX-curX);
                double deltaY = (targetY-curY);

                double angle=0;
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
                boolean destination = curX < (targetX +tolerance) && curX > (targetX -tolerance) && curY < (targetY +tolerance) && curY > (targetY -tolerance);

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

                telemetry.addData("Angle",motors.mecanum(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x,startDrive));
                telemetry.addData("Angle", var.getAngle());
                telemetry.addData("LeftFront",robot.leftFront.getPower());
                telemetry.addData("RightFront",robot.rightFront.getPower());
                telemetry.addData("LeftBack",robot.leftBack.getPower());
                telemetry.addData("RightBack",robot.rightBack.getPower());

                telemetry.update();
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

    private void shooterState(Shooter shooter) {

        switch (shooter)

        {

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
