package org.firstinspires.ftc.ramsrobotics.util

import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.ramsrobotics.Robot
import org.firstinspires.ftc.ramsrobotics.actions.Action
import org.firstinspires.ftc.ramsrobotics.actions.ActionType
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sign
import kotlin.math.sin

object Util {
    /** trent drive */
    fun trentDriveAngleDelta(driverPad: Gamepad, headingOffset: Double): Double {
        val x = smooth(-driverPad.right_stick_x.toDouble())
        val y = smooth(-driverPad.right_stick_y.toDouble())

        if(x == 0.0 && y == 0.0) {
            return 0.0
        }

        val targetHeading = normalizeAngle(atan2(y, x) + headingOffset)
        Logger.add("target heading", Math.toDegrees(targetHeading))
        val currentHeading = Robot.pose.heading
        Logger.add("current heading", Math.toDegrees(currentHeading))
        val delta = atan2(
            sin(targetHeading - currentHeading),
            cos(targetHeading - currentHeading)
        )
        val kP = 0.1
        Logger.add("detla", delta * kP)
        if(abs(delta) < 0.05) {
            return 0.0
        }
        return delta * kP
    }

    fun logActions(actions: HashMap<ActionType, Action>) {
        Logger.add("Active States", actions.keys.toString())
        Logger.add("Running Actions:")
        for(action in actions.values) {
            val debugData = action.submitDebugData()
            for(line in debugData) {
                Logger.add(line)
            }
        }
    }

    fun normalizeAngle(radians: Double): Double {
        var normalizedAngle = radians % (2 * Math.PI)
        if(normalizedAngle >= Math.PI) {
            normalizedAngle -= (2 * Math.PI)
        }

        if(normalizedAngle < -Math.PI) {
            normalizedAngle += (2 * Math.PI)
        }

        return normalizedAngle
    }

    fun getAngleBetweenPoses(from: Pose, to: Pose): Double {
        return atan2(to.y - from.y, to.x - from.x)
    }

    // curveBase 13 gives very similar to original smooth function
    // https://www.desmos.com/calculator/pe3ezqys0r
    private const val minPower = 0.0
    private const val curveBase = 16.7
    private const val controllerDeadzone = 0.1

    fun smooth(n: Double): Double {
        if(abs(n) <= controllerDeadzone || n == 0.0) return 0.0
        val sign = sign(n)
        val normalizedValue = (abs(n) - controllerDeadzone) / (1 - controllerDeadzone)
        val curve = curveBase.pow(normalizedValue - 1) * normalizedValue
        return sign * (minPower + (1 - minPower) * curve)
    }
}
