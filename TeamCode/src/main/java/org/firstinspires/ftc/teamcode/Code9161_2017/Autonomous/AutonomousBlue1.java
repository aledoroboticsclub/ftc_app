package org.firstinspires.ftc.teamcode.Code9161_2017.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Code9161_2017.Scorpion;

@Autonomous(name="AutonomousBlue1", group="Linear Opmode")
public class AutonomousBlue1 extends LinearOpMode {
    Scorpion r = new Scorpion();

    @Override
    public void runOpMode() {
        r.initRobot(hardwareMap, telemetry);
        AutoTransitioner.transitionOnStop(this, "TeleopScorpion");

        waitForStart();

        r.driveLeftEncoder(1,6);
        //r.pushJewel("Blue");
        r.driveBackwardEncoder(1,12);
        r.turnCounterwiseEncoder(1,12);
        int a = r.decodePictograph();
        r.driveBackwardEncoder(1,18 + a);
        r.turnClockwiseEncoder(1,12);
        r.driveForwardEncoder(1,36);
        r.setLiftToPosition0();
        r.setTrayToPlace();
    }
}