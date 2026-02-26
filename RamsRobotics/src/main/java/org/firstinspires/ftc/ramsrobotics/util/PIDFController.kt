package org.firstinspires.ftc.ramsrobotics.util

import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.max

class PIDFController(kP: Double, kI: Double, kD: Double, kV: Double, kA: Double) {
    private var kP = 0.0
    private var kI = 0.0
    private var kD = 0.0
    private var kV = 0.0
    private var kA = 0.0
    private var integral = 0.0
    private var prevError = 0.0
    private val time = ElapsedTime()

    init {
        setCoefficients(kP, kI, kD, kV, kA)
    }

    fun setCoefficients(kP: Double, kI: Double, kD: Double, kV: Double, kA: Double) {
        this.kP = kP
        this.kI = kI
        this.kD = kD
        this.kV = kV
        this.kA = kA
    }

    fun update(
        currentVelocity: Double,
        targetVelocity: Double,
        targetAcceleration: Double
    ): Double {
        val dt = max(time.seconds(), MIN_DT_SECONDS)
        val error = targetVelocity - currentVelocity
        integral += error * dt
        val p = kP * error
        val i = kI * integral
        val d = kD * (error - prevError) / dt
        val f = kV * targetVelocity + kA * targetAcceleration
        prevError = error
        time.reset()

        return p + i + d + f
    }

    fun update(error: Double): Double {
        val dt = max(time.seconds(), MIN_DT_SECONDS)
        integral += error * dt
        val p = kP * error
        val i = kI * integral
        val d = kD * (error - prevError) / dt
        prevError = error
        time.reset()

        return p + i + d
    }

    fun reset() {
        this.prevError = 0.0
        this.integral = 0.0
        this.time.reset()
    }

    companion object {
        private const val MIN_DT_SECONDS = 1e-4
    }
}
