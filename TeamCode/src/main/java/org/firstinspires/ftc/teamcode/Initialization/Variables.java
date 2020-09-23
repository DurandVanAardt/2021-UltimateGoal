package org.firstinspires.ftc.teamcode.Initialization;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Resources.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.Resources.Sensors;
import org.firstinspires.ftc.teamcode.Resources.Motors;

public class Variables {
    public RobotHardwareMap robot;
    public Motors motors;
    private boolean OpModeActive = false;
    public Sensors sensors;
    private double heading;
    private double roll;
    private double pitch;
//    Orientation lastAngles;
    public double angle;
    double trueAngle;
    double avgEncoder;
    double distanceL;
    double distanceR;
    double distanceB;
    private Orientation lastAngles;
    private double relativeAngle;

    public Variables(HardwareMap hardware) {
        robot = new RobotHardwareMap(hardware);
    }

    public void init(Variables var) {
        motors = new Motors(var);
        sensors = new Sensors(var);
    }

    public double getTrueAngle() {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        trueAngle = angles.firstAngle;
        return trueAngle;
    }

    public double getAvgEncoder() {
        return avgEncoder;
    }

    public void setAvgEncoder(double avgEncoder) {
        this.avgEncoder = avgEncoder;
    }

    public double getDistanceL() {
        return distanceL;
    }

    public void setDistanceL(double distanceL) {
        this.distanceL = distanceL;
    }

    public double getDistanceR() {
        return distanceR;
    }

    public void setDistanceR(double distanceR) {
        this.distanceR = distanceR;
    }

    public double getDistanceB() {
        return distanceB;
    }

    public void setDistanceB(double distanceB) {
        this.distanceB = distanceB;
    }

    public double getDistanceFM() {
        return distanceFM;
    }

    public void setDistanceFM(double distanceFM) {
        this.distanceFM = distanceFM;
    }

    double distanceFM;

    public double getHeading() {
        return heading;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    public double getRoll() {
        return roll;
    }

    public void setRoll(double roll) {
        this.roll = roll;
    }

    public double getPitch() {
        return pitch;
    }

    public void setPitch(double pitch) {
        this.pitch = pitch;
    }

//    public double getAngle() {
//        return angle;
//    }

    public void setAngle(double angle) {
        this.angle = angle;
    }


    public double getAngle() {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        relativeAngle += deltaAngle;

        lastAngles = angles;

        return relativeAngle;
    }

    public void resetAngle() {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        relativeAngle = 0;
    }

//    public double getAngle2() {
//        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
//    }

    public void setOpModeActive(boolean b) {
        this.OpModeActive = b;
    }

    public boolean isOpModeActive() {
        return OpModeActive;
    }
}
