package org.firstinspires.ftc.teamcode.teleop.prepPositions

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.jvm.java

@TeleOp
class MotorTest: LinearOpMode() {
    @Config
    data object motorConfig {
        @JvmField
        var motorPower1 = 0.0
        @JvmField
        var motorPower2 = 0.0
    }

    override fun runOpMode() {
        val hardwareMap : HardwareMap = hardwareMap
        val motor = hardwareMap.get(DcMotorEx::class.java, "motor")
        val motor2 = hardwareMap.get(DcMotorEx::class.java, "motor2")
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.direction = DcMotorSimple.Direction.FORWARD
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE


        motor2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor2.direction = DcMotorSimple.Direction.REVERSE
        motor2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        waitForStart()

        while (opModeIsActive()){
            motor.power= motorConfig.motorPower1
            motor2.power = motorConfig.motorPower2
            /*motor.power= -gamepad2.left_stick_y.toDouble()
            motor2.power = -gamepad2.right_stick_y.toDouble()
            */telemetry.addData("motor", motor.power)

            telemetry.update()
        }
    }
}