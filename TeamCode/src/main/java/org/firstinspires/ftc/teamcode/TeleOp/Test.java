package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Initialization.Variables;
import org.firstinspires.ftc.teamcode.Initialization.Initialize;
import org.firstinspires.ftc.teamcode.Resources.Motors;
import org.firstinspires.ftc.teamcode.Resources.RobotHardwareMap;


@TeleOp(name = "Test", group = "TeleOp")

public class Test extends OpMode {

    Variables var;
    Motors motors;
    RobotHardwareMap robot;

    DriveTrain driveTrain = DriveTrain.STOP;
    Shooter shooter = Shooter.SHOOTERREST;


    private boolean turnFirst = true;
    private boolean begin;
    private boolean begin1;

    private boolean turning = false;
    private boolean rotationShooter;
    private boolean turningUp;
    private boolean turningDown;
    private boolean turningLeft;
    private boolean turningRight;
    private Shooter curShooterState = Shooter.SHOOTERREST;
    private DriveTrain curDriveTrainState = DriveTrain.STOP;

    int Counter= 0;
    boolean shooterLoop = true;

    @Override
    public void init() {
        var = new Initialize().Init(hardwareMap);
        motors = var.motors;

        robot = var.robot;

    }

    @Override
    public void loop() {

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

        telemetry.addData("Current state", curShooterState);

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
    }

    public void readInputs() {
robot.wobbleLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

if ((gamepad2.b) && curShooterState == Shooter.SHOOTERREST)
{

    robot.wobbleLifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
robot.wobbleLifter.setTargetPosition(1000);
robot.wobbleLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

}

        if ((gamepad2.x) && curShooterState == Shooter.SHOOTERREST)
        {
            robot.wobbleLifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.wobbleLifter.setTargetPosition(1200);
            robot.wobbleLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        if ((gamepad2.y) && curShooterState == Shooter.SHOOTERREST)
        {
            robot.wobbleLifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.wobbleLifter.setTargetPosition(0);
            robot.wobbleLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
         if(gamepad2.a &&  curShooterState == Shooter.SHOOTERREST)

        {
            curShooterState = Shooter.FIRE;
            shooterState(Shooter.FIRE);

        }


        else if(gamepad2.a && curShooterState == Shooter.FIRE)

        {

            curShooterState = Shooter.SHOOTERREST;
            shooterState(Shooter.SHOOTERREST);
        }

        if (gamepad2.b)
        {

         curShooterState = Shooter.ADJUSTANGLE;
            shooterState(Shooter.ADJUSTANGLE);

        }




        if(gamepad2.left_bumper && curShooterState == Shooter.INTAKEREST)
        {
            curShooterState = Shooter.SUCKERIN;
            shooterState(Shooter.SUCKERIN);
        }

        else if(gamepad2.left_bumper) {
            curShooterState = Shooter.INTAKEREST;
            shooterState(Shooter.INTAKEREST);
        }

        if(gamepad2.right_bumper && curShooterState == Shooter.INTAKEREST)
        {
            curShooterState = Shooter.SUCKEROUT;
            shooterState(Shooter.SUCKEROUT);
        }

        else if(gamepad2.right_bumper)
        {
            curShooterState = Shooter.INTAKEREST;
            shooterState(Shooter.INTAKEREST);
        }

    if ((gamepad1.left_stick_x !=0) || (gamepad1.left_stick_y!=0) || (gamepad1.right_stick_x!=0))
        {

            driveTrainState(DriveTrain.DRIVE);

        }

    else if (gamepad1.left_trigger!=0)

        driveTrainState(DriveTrain.STRAFEL);

    else if (gamepad1.right_trigger!=0)
        driveTrainState(DriveTrain.STRAFER);

    else
        driveTrainState(DriveTrain.STOP);


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

    private void stateMachine(DriveTrain driveTrain, Shooter shooter) {
        driveTrainState(driveTrain);
        shooterState(shooter);
    }

    private void driveTrainState(DriveTrain driveTrain) {

        switch (driveTrain) {

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

                motors.mecanum(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x,true);
//
//                if (begin)
//                    begin = false;

                break;

            case TURNUP:

                if (turnFirst) {

                    var.resetAngle();

                    turnFirst = false;
                    motors.pidRotate.reset();
                    motors.pidRotate.enable();
                    turningUp = motors.rotate(0);
                }

                turningUp = motors.rotate(0);


                turnFirst = !turningUp;

                break;

            case TURNDOWN:

                if (turnFirst) {
                    var.resetAngle();
                    turnFirst = false;
                    motors.pidRotate.reset();
                    motors.pidRotate.enable();
                    turningDown = motors.rotate(180);
                }

                turningDown = motors.rotate(180);

                turnFirst = !turningDown;

                break;

            case TURNLEFT:

                if (turnFirst) {
                    var.resetAngle();
                    turnFirst = false;
                    motors.pidRotate.reset();
                    motors.pidRotate.enable();
                    turningLeft = motors.rotate(90);
                }

                turningLeft = motors.rotate(90);

                turnFirst = !turningLeft;

                break;

            case TURNRIGHT:

                if (turnFirst) {
                    var.resetAngle();
                    turnFirst = false;
                    motors.pidRotate.reset();
                    motors.pidRotate.enable();
                    turningRight = motors.rotate(-90);
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

            case STOP:

                motors.stop();

                begin = true;

                break;
        }
    }
    private void shooterState(Shooter shooter) {

        switch (shooter)

        {

            case FIRE:

               robot.shooterMotor.setPower(1);

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
