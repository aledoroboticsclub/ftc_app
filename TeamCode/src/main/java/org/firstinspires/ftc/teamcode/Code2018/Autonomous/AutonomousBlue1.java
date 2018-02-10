package org.firstinspires.ftc.teamcode.Code2018.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Code2018.Scorpion;
//@Disabled

@Autonomous(name="AutonomousBlue1", group="Linear OpMode")
public class AutonomousBlue1 extends LinearOpMode {

    Scorpion r = new Scorpion();

    @Override
    public void runOpMode() {
        AutoTransitioner.transitionOnStop(this, "TeleopScorpion");
        r.initRobot(hardwareMap, telemetry);
        r.initJewelDetector();

        waitForStart();

        r.relicTrackables.activate();
        r.driveLeftEncoder(.05,1);
        r.setPhoneToJewelPosition();
        r.pushJewel("Blue");
        r.setPhoneToUpPosition();
        r.driveRightEncoder(.25,40);
        r.jewelDetector.disable();
        r.initVuforia();
        r.driveForwardEncoder(.25,11);
        r.turnClockwiseEncoder(.25,21);
        RelicRecoveryVuMark bob= r.waitUntilVuMarkIsFound();
        r.driveLeftEncoder(.25,48);
        //r.driveBackwardEncoder(.5,pictographDistance);
        r.turnClockwiseEncoder(.25,21);
        r.setTrayToPlace();
        r.driveBackwardEncoder(.25,3);
        r.setTrayToIntake();
    }
}