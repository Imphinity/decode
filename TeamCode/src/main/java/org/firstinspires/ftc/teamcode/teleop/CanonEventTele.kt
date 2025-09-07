package org.firstinspires.ftc.teamcode.teleop

import com.commonlibs.units.Pose
import com.commonlibs.units.cm
import com.commonlibs.units.deg
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.robot.Robot

@TeleOp
class CanonEventTele : LinearOpMode(){
    override fun runOpMode() {
        val robot = Robot(hardwareMap,Pose(0.0.cm, 0.0.cm, 0.0.deg))

        waitForStart()

        while (opModeIsActive()) {

            robot.drive.isSlowMode = gamepad1.right_trigger >= 0.2
            robot.drive.driveFieldCentric(
                -gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble()
            )
            if (gamepad1.y) {
                robot.drive.resetFieldCentric()
            }

            robot.canon.power = gamepad2.left_stick_y.toDouble()
        }
    }
}