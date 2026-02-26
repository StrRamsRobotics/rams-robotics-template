package org.firstinspires.ftc.ramsrobotics.opmodes.auto

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.util.Timer
import org.firstinspires.ftc.ramsrobotics.Robot
import org.firstinspires.ftc.ramsrobotics.opmodes.BaseOpMode
import org.firstinspires.ftc.ramsrobotics.util.AutonSequence
import org.firstinspires.ftc.ramsrobotics.util.Constants
import org.firstinspires.ftc.ramsrobotics.util.Logger
import org.firstinspires.ftc.ramsrobotics.util.Util

abstract class BaseAuto : BaseOpMode() {
    private var follower: Follower? = null
    protected var pathTimer: Timer? = null
    protected var opmodeTimer: Timer? = null
    private var autonSequence: AutonSequence? = null
    private var pathState = 0

    abstract fun getStartPose(): Pose

    abstract fun defineAutonSequence(builder: AutonSequence.Builder): AutonSequence.Builder

    abstract fun definePaths(follower: Follower)

    fun tickAuto(follower: Follower, pathState: Int) {
        autonSequence!!.tick(follower, pathState)
    }

    fun advancePathState() {
        setPathState(pathState + 1)
    }

    fun setEndPathState() {
        setPathState(-1)
    }

    fun setPathState(state: Int) {
        pathState = state
        pathTimer!!.resetTimer()
    }

    public override fun initialize() {
        super.initialize()
        pathTimer = Timer()
        opmodeTimer = Timer()
        pathState = 0

        follower = Constants.createFollower(hardwareMap)
        follower!!.setStartingPose(getStartPose())
        Robot.pose = follower!!.pose
        definePaths(follower!!)
        autonSequence = defineAutonSequence(AutonSequence.Builder(this)).build()
    }

    override fun start() {
        opmodeTimer!!.resetTimer()
        setPathState(0)
    }

    override fun tick() {
        follower!!.update()
        Robot.pose = follower!!.pose
        tickAuto(follower!!, pathState)
    }

    override fun stop() {
        stopAllActions()
    }

    public override fun logging() {
        Logger.add("------------------------")
        Logger.add("Path Index", pathState)
        Logger.add("------------------------")
        Logger.add("X Pos", "%.2f", Robot.pose.x)
        Logger.add("Y Pos", "%.2f", Robot.pose.y)
        Logger.add("Heading", "%.2f", Math.toDegrees(Robot.pose.heading))
        Logger.add("------------------------")
        Util.logActions(runningActions)
        Logger.add("------------------------")
    }
}
