package org.firstinspires.ftc.ramsrobotics.actions

fun interface Callback : Action, Runnable {
    /**
     * the functional interface method
     */
    override fun run()

    /**
     * only runs once
     */
    override fun start() {
        run()
    }

    override fun shouldTick(): Boolean {
        return false
    }

    override fun tick() {
    }

    override fun stop() {
    }
}
