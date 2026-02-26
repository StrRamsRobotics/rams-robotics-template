package org.firstinspires.ftc.ramsrobotics.actions

import java.util.LinkedList

class InSequence(vararg actions: Action) : BaseAction() {
    private val actions: LinkedList<Action> = LinkedList<Action>(listOf(*actions))
    private var onStopCallback = Callback {}

    fun onStop(callback: Callback): InSequence {
        onStopCallback = callback
        return this
    }

    /**
     * @return true if it hasn't run every one of its child actions, false otherwise
     */
    override fun shouldContinue(): Boolean {
        return !actions.isEmpty()
    }

    override fun start() {
        val newAction = actions.peek()
        newAction?.start()
    }

    override fun tick() {
        val action = actions.peek()
        if(action != null && action.tickOrStop().stopped()) {
            actions.pop()
            val newAction = actions.peek()
            newAction?.start()
        }
    }

    override fun stop() {
        if(!actions.isEmpty()) {
            actions.pop()!!.stop()
        }
        onStopCallback.run()
    }

    override fun submitDebugData(): ArrayList<String> {
        val data = ArrayList<String>()
        data.add("-> ${this.javaClass.getSimpleName()}")
        val curAction = actions.peek()
        if(curAction != null) {
            val subActionData = curAction.submitDebugData()
            for(line in subActionData) {
                data.add("   $line")
            }
        }
        return data
    }
}
