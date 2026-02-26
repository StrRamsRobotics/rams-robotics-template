package org.firstinspires.ftc.ramsrobotics.util

import org.firstinspires.ftc.ramsrobotics.Robot

object Logger {
    fun add(key: String?, format: String?, vararg values: Any?) {
        Robot.telemetry.addData(key, format, *values)
    }

    fun add(key: String?, value: Any?) {
        Robot.telemetry.addData(key, value)
    }

    fun add(text: String?) {
        Robot.telemetry.addLine(text)
    }

    fun update() {
        Robot.telemetry.update()
    }
}
