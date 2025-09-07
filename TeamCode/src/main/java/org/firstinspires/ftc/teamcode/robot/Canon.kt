package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.RaceAction
import com.acmerobotics.roadrunner.ftc.Encoder
import com.commonlibs.units.Duration
import com.commonlibs.units.SleepAction
import com.commonlibs.units.s
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.library.controller.PIDController
import kotlin.math.abs

class Canon(
    val motorUp: DcMotorEx,
    val motorDown: DcMotorEx
)
{
    enum class Mode {
        PID,
        RAW_POWER
    }

    private var currentMode = Mode.RAW_POWER
    private var offset = 0.0

    var power
        get() = motorUp.power
        set(value) {
            motorUp.power = value
            motorDown.power= value
            currentMode = Mode.RAW_POWER
        }

    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("sistem power", power)
        //telemetry.addData("lift current", rightMotor.getCurrent(CurrentUnit.AMPS) + leftMotor.getCurrent(CurrentUnit.AMPS))
    }
}