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
import com.qualcomm.robotcore.hardware.Servo

@TeleOp
class SystemTest : LinearOpMode() {

    @Config
    data object systemConfig {
        @JvmField var motorPower1 = 0.0
        @JvmField var motorPower2 = 0.0
        @JvmField var servoPos = 0.5

        // how often to compute and reset in seconds
        @JvmField var sampleWindow = 0.1
    }

    override fun runOpMode() {
        val motor = hardwareMap.get(DcMotorEx::class.java, "motor")
        val motor2 = hardwareMap.get(DcMotorEx::class.java, "motor2")
        val encoder = hardwareMap.get(DcMotor::class.java, "encoder")
        val servo = hardwareMap.get(Servo::class.java, "servo")

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.direction = DcMotorSimple.Direction.FORWARD
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        motor2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor2.direction = DcMotorSimple.Direction.FORWARD
        motor2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        encoder.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        val TICKS_PER_REV = 8192.0

        var lastTime = now()
        var lastResetTime = now()
        var lastPos = 0
        var rpm = 0.0

        waitForStart()

        while (opModeIsActive()) {
            val currentTime = now()
            val deltaTime = currentTime - lastTime

            motor.power = systemConfig.motorPower1
            motor2.power = systemConfig.motorPower2

            servo.position = systemConfig.servoPos

            // calculate rpm only every sampleWindow seconds
            if (currentTime - lastResetTime >= systemConfig.sampleWindow) {
                val pos = encoder.currentPosition
                val elapsed = currentTime - lastResetTime

                val revs = pos / TICKS_PER_REV
                rpm = (revs / elapsed) * 60.0

                // reset encoder for next window
                encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                encoder.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

                lastResetTime = currentTime
                lastPos = 0
            }

            telemetry.addData("Motor1 Power", motor.power)
            telemetry.addData("Motor2 Power", motor2.power)
            telemetry.addData("Encoder Position", encoder.currentPosition)
            telemetry.addData("last reset time", lastResetTime)
            telemetry.addData("current time", currentTime)
            telemetry.addData("ΔTime Loop (s)", "%.4f".format(deltaTime))
            telemetry.addData("Sample Window (s)", "%.3f".format(systemConfig.sampleWindow))
            telemetry.addData("RPM", "%.2f".format(rpm))
            telemetry.update()

            lastTime = currentTime
        }
    }
}
