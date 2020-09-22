package org.firstinspires.ftc.teamcode.Resources;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class RobotHardwareMap {
    private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;
    public DcMotor liftMotor;
    public Servo Claw;
    public DistanceSensor distanceR;
    public DistanceSensor distanceL;
    public DistanceSensor distanceFL;
    public DistanceSensor distanceFR;
    public DistanceSensor distanceFM;
    public DistanceSensor distanceB;
    public DigitalChannel downLimit;
    public BNO055IMU imu;
    public Servo RBG;
    public Servo LBG;

    public RobotHardwareMap(HardwareMap hardwareMap) {
        init(hardwareMap);
    }

    public void init(HardwareMap hardwareMap){

        Claw = hardwareMap.get(Servo.class,"claw");
        RBG = hardwareMap.get(Servo.class, "RBG");
        LBG = hardwareMap.get(Servo.class, "LBG");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        initializeIMU();

        distanceR = hardwareMap.get(DistanceSensor.class, "distanceR");
        distanceL = hardwareMap.get(DistanceSensor.class, "distanceL");
        distanceB = hardwareMap.get(DistanceSensor.class, "distanceB");
        distanceFL = hardwareMap.get(DistanceSensor.class, "distanceFR");
        distanceFR = hardwareMap.get(DistanceSensor.class, "distanceFL");
        distanceFM = hardwareMap.get(DistanceSensor.class, "distanceFM");

        leftFront = hardwareMap.get(DcMotor.class, "leftmotor");
        rightFront = hardwareMap.get(DcMotor.class, "rightmotor");
        leftBack = hardwareMap.get(DcMotor.class, "leftback");
        rightBack = hardwareMap.get(DcMotor.class, "rightback");
        liftMotor = hardwareMap.get(DcMotor.class, "motorHeight");

        downLimit = hardwareMap.get(DigitalChannel.class, "downLimit");
        downLimit.setMode(DigitalChannel.Mode.INPUT);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
//       rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
//        rightBack.setDirection(DcMotor.Direction.REVERSE);


        liftMotor.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        enableEncoders();
    }

    private void enableEncoders(){

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void initializeIMU(){
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }
}
