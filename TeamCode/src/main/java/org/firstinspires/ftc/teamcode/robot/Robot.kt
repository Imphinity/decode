package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.commonlibs.units.Duration
import com.commonlibs.units.Pose
import com.commonlibs.units.SleepAction
import com.commonlibs.units.cm
import com.commonlibs.units.deg
import com.commonlibs.units.s
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import kotlin.jvm.java

class Robot(
    hardwareMap: HardwareMap,
    pose: Pose = Pose(0.0.cm, 0.0.cm, 0.0.deg),
    resetEncoders: Boolean = true
) {
    val drive: Drive
    val canon: Canon

    init {
        val mecanumDrive = MecanumDrive(hardwareMap, pose.pose2d)


        val motorUp = hardwareMap.get(DcMotorEx::class.java, "motorUp")
        val motorDown = hardwareMap.get(DcMotorEx::class.java, "motorDown")

        motorUp.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorDown.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        motorUp.direction = DcMotorSimple.Direction.REVERSE
        motorDown.direction = DcMotorSimple.Direction.FORWARD

        motorUp.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motorDown.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE


        drive = Drive(mecanumDrive)
        canon = Canon(
            motorUp = motorUp,
            motorDown = motorDown
        )
    }
}