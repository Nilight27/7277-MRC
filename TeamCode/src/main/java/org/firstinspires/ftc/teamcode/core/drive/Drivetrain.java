package org.firstinspires.ftc.teamcode.core.drive;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Arrays;

public class Drivetrain {
    public final IMU gyro;
    private final DcMotorEx _0, _1, _2, _3;
    private final LinearOpMode opMode;
    private double speed = 1;
    private static double error;
    private static double heading;
    private static double gain = .015;

    public Drivetrain(LinearOpMode linearOpMode) {
        // Define every motor.
        _0 = linearOpMode.hardwareMap.get(DcMotorEx.class, "0"); // frontLeft
        _1 = linearOpMode.hardwareMap.get(DcMotorEx.class, "1"); // frontRight
        _2 = linearOpMode.hardwareMap.get(DcMotorEx.class, "2"); // backRight
        _3 = linearOpMode.hardwareMap.get(DcMotorEx.class, "3"); // backLeft

        gyro = linearOpMode.hardwareMap.get(IMU.class, "imu");

        gyro.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                new Orientation(
                                        AxesReference.INTRINSIC,
                                        AxesOrder.ZYX,
                                        AngleUnit.DEGREES,
                                        0,
                                        0,
                                        90,
                                        0
                                )
                        )
                )
        );

        // Define the LinearOpMode that's going to be used.
        opMode = linearOpMode;

        // Loop through each motor and configure that motor.
        for (DcMotorEx motor : Arrays.asList(_0, _1, _3, _2)) {
            // Stop each motor, and reset the encoder of each motor.
            // This isn't necessary, but it's here anyway!
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // We'll want each motor to brake when the robot is put to a stop.
            // This way if a surrounding robot starts to push us out of our location, we'll stay in the same spot.
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private void normalize(double[] speeds, double magnitude) {
        double maxMagnitude = Math.abs(speeds[0]);

        for (int i = 1; i < speeds.length; i++) {
            double temp = Math.abs(speeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }

        for (int i = 0; i < speeds.length; i++) {
            speeds[i] = (speeds[i] / maxMagnitude) * magnitude;
        }
    }

    public void rotate(double speedLimit, double degrees) {
        double speed;

        while (opMode.opModeIsActive() && (Math.abs(error) > 0)) {
            speed = getSteeringCorrection(-degrees, gain) * speedLimit;

            // Loop through each motor and set the speed of that motor.
            for (DcMotorEx motor : Arrays.asList(_0, _1, _3, _2)) {
                // Due to the orientation of each motor, movement will have to be reversed.
                // This way the robot should steer in the proper direction.
                motor.setPower(-speed);
            }
        }

        gyro.resetYaw();
        this.stop();
    }

    private double getSteeringCorrection(double target, double gain) {
        heading = getHeading();
        error = target - heading;

        return Range.clip(error * gain, -1, 1);
    }


    private double getHeading() {
        YawPitchRollAngles angles = gyro.getRobotYawPitchRollAngles();
        return AngleUnit.normalizeDegrees(
                angles.getYaw(AngleUnit.DEGREES)
        );
    }

    public void align() {
        heading = getHeading();
        rotate(0.5, Math.round(heading / 45) * 45);
    }

    public void stop() {
        // Loop through each motor and stop that motor.
        for (DcMotorEx motor : Arrays.asList(_0, _1, _3, _2)) {
            // Stop each motor, and reset the encoder of each motor.
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void moveBy(double forward, double strafe) {
        double tick = 480 / (Math.PI * 10);

        int forwardTarget = (int) Math.abs(forward * tick);
        int strafeTarget = (int) Math.abs(strafe * tick);
        stop();

        // Set the target position for each motor based on the provided input.
        if (forward != 0) {
            if (forward > 0) {
                // Move the robot forwards.
                _0.setTargetPosition(forwardTarget);
                _3.setTargetPosition(forwardTarget);
                _1.setTargetPosition(-forwardTarget);
                _2.setTargetPosition(-forwardTarget);
            } else {
                // Move the robot backwards.
                _0.setTargetPosition(-forwardTarget);
                _3.setTargetPosition(-forwardTarget);
                _1.setTargetPosition(forwardTarget);
                _2.setTargetPosition(forwardTarget);
            }
        } else {
            if (strafe > 0) {
                // Move the robot right.
                _0.setTargetPosition(-strafeTarget);
                _3.setTargetPosition(strafeTarget);
                _1.setTargetPosition(-strafeTarget);
                _2.setTargetPosition(strafeTarget);
            } else {
                // Move the robot left.
                _0.setTargetPosition(strafeTarget);
                _3.setTargetPosition(-strafeTarget);
                _1.setTargetPosition(strafeTarget);
                _2.setTargetPosition(-strafeTarget);
            }
        }

        // Loop through each motor and set the speed of that motor.
        for (DcMotorEx motor : Arrays.asList(_2, _3, _0, _1)) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(speed);
        }

        // Wait for the robot to stop moving.
        while (_0.isBusy() && opMode.opModeIsActive() && !opMode.isStopRequested())
            opMode.idle();

        // Stop the robot.
        stop();
    }

    public void forwards(double power) {
        power = Math.abs(power) * speed;

        _0.setPower(power);
        _3.setPower(power);
        _1.setPower(-power);
        _2.setPower(-power);
    }


    public void backwards(double power) {
        power = Math.abs(power) * speed;

        _0.setPower(-power);
        _3.setPower(-power);
        _1.setPower(power);
        _2.setPower(power);
    }

    public void left(double power) {
        power = Math.abs(power) * speed;

        _0.setPower(power);
        _3.setPower(-power);
        _1.setPower(power);
        _2.setPower(-power);
    }

    public void right(double power) {
        power = Math.abs(power) * speed;

        _0.setPower(-power);
        _3.setPower(power);
        _1.setPower(-power);
        _2.setPower(power);
    }

    public void move(double strafe, double forward, double twist) {
        strafe = -Range.clip(strafe, -1, 1);
        forward = Range.clip(forward, -1, 1);
        twist = -Range.clip(twist, -1, 1);

        Vector2d vector = new Vector2d(strafe, forward);
        vector = vector.rotateBy(
                -getHeading()
        );

        double theta = vector.angle();
        double[] speeds = new double[4];

        speeds[0] = Math.sin(theta + Math.PI / 4);
        speeds[1] = Math.sin(theta - Math.PI / 4);
        speeds[3] = Math.sin(theta - Math.PI / 4);
        speeds[2] = Math.sin(theta + Math.PI / 4);

        normalize(speeds, vector.magnitude());

        speeds[0] += twist;
        speeds[1] -= twist;
        speeds[3] += twist;
        speeds[2] -= twist;

        speeds[0] = Range.clip(speeds[0], -1, 1);
        speeds[1] = Range.clip(speeds[1], -1, 1);
        speeds[3] = Range.clip(speeds[3], -1, 1);
        speeds[2] = Range.clip(speeds[2], -1, 1);

        _0.setPower(speeds[0] * speed);
        _1.setPower(-(speeds[1] * speed));
        _3.setPower(speeds[3] * speed);
        _2.setPower(-(speeds[2] * speed));
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }
}

    


