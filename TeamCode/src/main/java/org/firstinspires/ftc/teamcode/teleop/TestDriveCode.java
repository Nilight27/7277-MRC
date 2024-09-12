package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.hardware.Robot;


@TeleOp(name="TestDriveCode")
public class TestDriveCode extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Initialize hardware.
        GamepadEx gamepad = new GamepadEx(gamepad1);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Robot robot = new Robot(this);

        telemetry = new MultipleTelemetry(
                telemetry,
                dashboard.getTelemetry()
        );

            waitForStart();
            telemetry.addData("Drive", "READY");
            telemetry.update();


        // Open a new loop that will run until the end of the op-mode.
        while (opModeIsActive()) {
            double x = gamepad.getLeftX();
            double y = gamepad.getLeftY();
            double x2 = gamepad.getRightX() * 0.5;
            // Move the robot's drivetrain based on user gamepad input.
            robot.setSpeed(0.5);
            robot.move(x, y, x2);

            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("x2", x2);
            telemetry.update();

        }
    }
}

