package org.firstinspires.ftc.ramsrobotics.actions

/**
 * When a certain action type is ran, a new action instance is created. This way, the
 * variables contained within each action can be modified without influencing other instances.
 */
interface Action {
    /**
     * @return if the action is running or if it stopped
     */
    fun tickOrStop(): TickResult {
        if (!this.shouldTick()) {
            this.stop()
            return TickResult.STOPPED
        }
        this.tick()
        return TickResult.RUNNING
    }

    /**
     * called once when the action is started
     */
    fun start()

    /**
     * @return true if the action should continue to run and false if the action has completed
     */
    fun shouldTick(): Boolean

    /**
     * called every tick, after [Action.shouldTick]
     */
    fun tick()

    /**
     * called once when the action is stopped
     */
    fun stop()

    /**
     * @return lines that should be added to the logger
     */
    fun submitDebugData(): ArrayList<String> {
        val list = ArrayList<String>()
        list.add("-> " + this.javaClass.simpleName)
        return list
    }

    enum class TickResult {
        RUNNING,
        STOPPED;

        fun stopped(): Boolean {
            return this == STOPPED
        }

        fun running(): Boolean {
            return this == RUNNING
        }
    }
}
