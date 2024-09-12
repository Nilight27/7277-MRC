package org.firstinspires.ftc.teamcode.core.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.core.drive.Drivetrain;

import java.util.List;

public class Robot {
    private static Robot instance = null;
    private LinearOpMode linearOpMode;

    public Drivetrain drivetrain;
    public List<LynxModule> hubs;

    public static Robot getInstance() {
        return instance;
    }

    public Robot(LinearOpMode linearOpMode) {
        drivetrain = new Drivetrain(linearOpMode);
        hubs = linearOpMode.hardwareMap.getAll(
                LynxModule.class
        );

        for (LynxModule module : hubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        instance = this;
        this.linearOpMode = linearOpMode;
    }

    public Robot(OpMode opMode) {
        this((LinearOpMode) opMode);
    }

    public void setSpeed(double speed) {
        drivetrain.setSpeed(speed);
    }

    public void moveBy(double forward, double strafe) {
        drivetrain.moveBy(forward, strafe);
    }

    public void move(double strafe, double forward, double twist) {
        drivetrain.move(strafe, forward, twist);
    }

    public void rotate(double speedLimit, double degrees) {
        drivetrain.rotate(speedLimit, degrees);
    }
}

