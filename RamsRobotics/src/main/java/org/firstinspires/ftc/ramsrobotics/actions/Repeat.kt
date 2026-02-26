package org.firstinspires.ftc.ramsrobotics.actions

import java.util.function.Supplier

class Repeat(private val action: Supplier<Action?>) : BaseAction() {
    private var currentAction: Action? = null
    private var repetitions = 0
    private var maxRepetitions = Int.Companion.MAX_VALUE

    fun repetitions(times: Int): BaseAction {
        maxRepetitions = times
        return this
    }

    override fun start() {
        repetitions = 0
        currentAction = null
        if (maxRepetitions <= 0) {
            return
        }
        currentAction = action.get()
        currentAction!!.start()
    }

    override fun shouldContinue(): Boolean {
        return repetitions < maxRepetitions
    }

    override fun tick() {
        if (currentAction != null && currentAction!!.tickOrStop().stopped()) {
            repetitions++
            if (repetitions >= maxRepetitions) {
                return
            }
            currentAction = action.get()
            currentAction!!.start()
        }
    }

    override fun stop() {
        if (currentAction != null) {
            currentAction!!.stop()
        }
    }
}
