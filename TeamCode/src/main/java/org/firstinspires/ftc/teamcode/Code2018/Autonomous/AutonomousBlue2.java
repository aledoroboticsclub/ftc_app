package org.firstinspires.ftc.teamcode.Code2018.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Code2018.Scorpion;

@Autonomous(name="AutonomousBlue2", group="Linear OpMode")
//@Disabled

public class AutonomousBlue2 extends LinearOpMode {
    Scorpion r = new Scorpion();
    RelicRecoveryVuMark pictograph;
    int pictographDistance = 0;
    @Override
    public void runOpMode() {
        AutoTransitioner.transitionOnStop(this, "TeleopScorpion");
        r.initRobot(hardwareMap, telemetry);
        r.initJewelDetector();
        r.waiter(2500);

        waitForStart();

        r.setJewelPusherToDown();
        r.driveRightEncoder(.1,3);
        r.waiter(1000);
        r.setPhoneToJewelPosition();
        r.pushJewel("Blue");
        r.setJewelPusherToUp();
        r.setPhoneToUpPosition();
        r.waiter(1000);

        r.jewelDetector.disable();
        telemetry.addData("jeweldetector", "off");
        telemetry.update();

        r.driveLeftEncoder(.5,24);
        telemetry.addData("driveleft", "now");
        telemetry.update();

        telemetry.addData("going forward", "now");
        telemetry.update();
        r.driveBackwardEncoder(.5,-34);
        r.turnClockwiseEncoder(.25,24);
        r.waiter(500);
//        r.driveBackwardEncoder(.25,-20);
//        r.waiter(500);
//        r.leftIntake(1);
//        r.rightIntake(1);
//        r.waiter(2000);
//        r.rightIntake(0);
//        r.leftIntake(0);
//        r.driveBackwardEncoder(.5,24);
//        r.driveBackwardEncoder(.1,12);
//        r.turnCounterwiseEncoder(.25,30);
//        r.driveRightEncoder(.5,30);
//        r.turnClockwiseEncoder(.25,24);
//        r.driveBackwardEncoder(.1,-8);

//        r.initVuforia();
//        r.waiter(2500);
//        pictograph = r.waitUntilVuMarkIsFound();
//        telemetry.addData("VuMark: ",pictograph);
//        //defaults to left
//        if(pictograph.equals(RelicRecoveryVuMark.CENTER))
//            pictographDistance = 7;
//        if(pictograph.equals(RelicRecoveryVuMark.RIGHT))
//            pictographDistance = 14;
//        r.deactivateVuforia();
//        r.setPhoneToUpPosition();
        //r.driveForwardEncoder(.25, 12);
        //r.turnClockwiseEncoder(.25,24);
        //r.driveBackwardEncoder(1,30);

//        r.setTrayToPlace();
//        r.driveBackwardEncoder(.25,-3);
//        r.setTrayToIntake();
    }
}