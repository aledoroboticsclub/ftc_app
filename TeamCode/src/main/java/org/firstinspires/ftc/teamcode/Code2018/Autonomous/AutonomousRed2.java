package org.firstinspires.ftc.teamcode.Code2018.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Code2018.Scorpion;
//@Disabled

@Autonomous(name="AutonomousRed2", group="Linear OpMode")
public class AutonomousRed2 extends LinearOpMode {
    Scorpion r = new Scorpion();
    int a = 0;
    @Override
    public void runOpMode() {
        r.initRobot(hardwareMap, telemetry);
        AutoTransitioner.transitionOnStop(this, "TeleopScorpion");

        waitForStart();

        r.driveLeftEncoder(1,6);
        r.pushJewel("Red");
        r.driveBackwardEncoder(1,12);
        r.turnCounterwiseEncoder(1,12);
        r.driveLeftEncoder(1,12 + a);
        r.setLiftToPosition0();
    }
}