package org.firstinspires.ftc.teamcode.Code9161_2017.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Code9161_2017.Scorpion;
@Autonomous(name="AutonomousRed1", group="Linear OpMode")
public class AutonomousRed1 extends LinearOpMode {
    Scorpion r = new Scorpion();
    int a =0;
    @Override
    public void runOpMode() {
        r.initRobot(hardwareMap, telemetry);
        AutoTransitioner.transitionOnStop(this, "TeleopScorpion");

        waitForStart();

        r.driveLeftEncoder(1,6);
        r.pushJewel("Red");
        r.driveBackwardEncoder(1,12);
        r.turnCounterwiseEncoder(1,12);
        a = r.decodePictograph();
        r.driveBackwardEncoder(1,18 + a);
        r.turnCounterwiseEncoder(1,12);
        r.driveForwardEncoder(1,12);
        r.setLiftToPosition0();
    }
}