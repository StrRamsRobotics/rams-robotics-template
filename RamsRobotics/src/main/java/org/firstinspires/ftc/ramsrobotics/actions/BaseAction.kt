package org.firstinspires.ftc.ramsrobotics.actions

abstract class BaseAction : Action {
    private var startTime = -1.0

    /** seconds */
    private var timeout = -1.0

    fun timeout(seconds: Double): BaseAction {
        timeout = seconds
        return this
    }

    override fun start() {
    }

    override fun shouldTick(): Boolean {
        if (timeout < 0) {
            return shouldContinue()
        }
        val elapsedTime: Double
        if (startTime < 0) {
            startTime = currentTime()
            elapsedTime = 0.0
        } else {
            elapsedTime = currentTime() - startTime
        }

        return elapsedTime < timeout && shouldContinue()
    }

    open fun shouldContinue(): Boolean {
        return false
    }

    override fun tick() {
    }

    override fun stop() {
    }

    protected fun startTime(): Double {
        return startTime
    }

    protected fun currentTime(): Double {
        return System.nanoTime() * 1e-9
    }
}
