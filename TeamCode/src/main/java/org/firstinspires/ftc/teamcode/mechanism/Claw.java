package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanism.mechanismconstants.ClawConstants;

public class Claw {
    public Servo claw;

    public enum Action{
        OPEN,
        CLOSE,
        NORMAL

    }

    public Claw(HardwareMap hardwareMap){
        claw = hardwareMap.get(Servo.class, "claw");
    }

    public void move(Action action) {

        if (action == Action.OPEN) {
            claw.setPosition(ClawConstants.open);
        }
        if (action == Action.CLOSE) {
            claw.setPosition(ClawConstants.close);
        }
        if (action == Action.NORMAL) {
            claw.setPosition(ClawConstants.neutral);
        }
    }
}
