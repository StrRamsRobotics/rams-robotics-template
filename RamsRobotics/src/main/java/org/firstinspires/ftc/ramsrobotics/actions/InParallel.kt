package org.firstinspires.ftc.ramsrobotics.actions

/**
 * This action will stop when all of its main actions have finished running.
 * Secondary actions have no effect on whether this action should stop.
 */
class InParallel(vararg actions: Action) : BaseAction() {
    private val actions: ArrayList<Action> = ArrayList(listOf(*actions))
    private var secondaryActions: ArrayList<Action>? = null
    private var onStopCallback = Callback {}

    fun onStop(callback: Callback): InParallel {
        onStopCallback = callback
        return this
    }

    /**
     * secondary actions are run in parallel but do not force the group action to continue
     */
    fun setSecondaryActions(vararg actions: Action): InParallel {
        secondaryActions = ArrayList(listOf(*actions))
        return this
    }

    override fun start() {
        actions.forEach { obj: Action? -> obj!!.start() }
        secondaryActions?.forEach { action -> action.start() }
    }

    /**
     * @return true if any of its child actions are still running, false otherwise
     */
    override fun shouldContinue(): Boolean {
        return !actions.isEmpty()
    }

    override fun tick() {
        actions.removeIf { action: Action? -> action!!.tickOrStop().stopped() }
        secondaryActions?.removeIf { action -> action.tickOrStop().stopped() }
    }

    override fun stop() {
        actions.forEach { action -> action.stop() }
        secondaryActions?.forEach { action -> action.stop() }
        onStopCallback.run()
    }

    override fun submitDebugData(): ArrayList<String> {
        val data = ArrayList<String>()
        data.add("-> ${this.javaClass.getSimpleName()}")
        for(action in actions) {
            val subActionData = action.submitDebugData()
            for(line in subActionData) {
                data.add("   $line")
            }
        }
        if(secondaryActions != null) {
            for(action in secondaryActions) {
                val subActionData = action.submitDebugData()
                for(line in subActionData) {
                    data.add("   $line")
                }
            }
        }
        return data
    }
}
