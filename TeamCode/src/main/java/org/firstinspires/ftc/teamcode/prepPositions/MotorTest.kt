package org.firstinspires.ftc.teamcode.teleop.prepPositions

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.now
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.library.controller.PIDController
import org.firstinspires.ftc.teamcode.library.controller.LowPassFilter
import org.firstinspires.ftc.teamcode.teleop.prepPositions.OuttakeTest.outtakeConfig
import kotlin.math.abs
import kotlin.math.sign

@TeleOp
class MotorTest : LinearOpMode() {

    @Config
    data object motorConfig {
        @JvmField var basePower1 = 0.0
        @JvmField var basePower2 = 0.0

        @JvmField var sampleWindow = 0.1
        @JvmField var TICKS_PER_REV = 8192.0

        @JvmField var targetRPM = 3400.0

        var controller = PIDController(
            kP = 0.0006,
            kI = 0.0001,
            kD = 0.0000,
            stabilityThreshold = 100.0
        )

        @JvmField var integralLimit = 5000.0

        // optional feedforward (power bias)
        @JvmField var kF = 0.0

        // filtering
        @JvmField var rpmFilterGain = 0.3
    }

    override fun runOpMode() {
        val motor = hardwareMap.get(DcMotorEx::class.java, "motor")
        val motor2 = hardwareMap.get(DcMotorEx::class.java, "motor2")
        val encoder = hardwareMap.get(DcMotor::class.java, "encoder")

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.direction = DcMotorSimple.Direction.FORWARD
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

        motor2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor2.direction = DcMotorSimple.Direction.REVERSE
        motor2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

        encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        encoder.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        var lastTime = now()
        var lastResetTime = now()
        var rpm = 0.0
        val timeKeep = TimeKeep()

        waitForStart()

        while (opModeIsActive()) {
            timeKeep.resetDeltaTime()
            val currentTime = now()
            val dt = currentTime - lastTime

            if (currentTime - lastResetTime >= outtakeConfig.sampleWindow) {
                val pos = encoder.currentPosition
                val elapsed = currentTime - lastResetTime
                val revs = pos / outtakeConfig.TICKS_PER_REV
                rpm = (revs / elapsed) * 60.0

                encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                encoder.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                lastResetTime = currentTime
            }

            motor.power= motorConfig.basePower1
            motor2.power= motorConfig.basePower1

            telemetry.addData("Motor1 Power", motor.power)
            telemetry.addData("Motor2 Power", motor2.power)
            telemetry.addData("rpm", rpm)
            telemetry.update()
        }
    }
}
