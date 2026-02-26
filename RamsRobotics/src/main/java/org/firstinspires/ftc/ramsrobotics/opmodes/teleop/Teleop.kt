package org.firstinspires.ftc.ramsrobotics.opmodes.teleop

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.ramsrobotics.Robot
import org.firstinspires.ftc.ramsrobotics.opmodes.BaseOpMode
import org.firstinspires.ftc.ramsrobotics.util.Alliance
import org.firstinspires.ftc.ramsrobotics.util.Constants
import org.firstinspires.ftc.ramsrobotics.util.Logger
import org.firstinspires.ftc.ramsrobotics.util.Util

abstract class Teleop : BaseOpMode() {
    private var follower: Follower? = null
    private var fieldCentric = true

    abstract fun getStartPose(): Pose?

    open fun cachePoseFromAuto(): Boolean {
        return false
    }

    public override fun initialize() {
        super.initialize()
        follower = Constants.createFollower(hardwareMap)
        if(cachePoseFromAuto()) {
            follower!!.setStartingPose(Robot.pose)
        }
        else {
            follower!!.setStartingPose(getStartPose())
        }
        Robot.pose = follower!!.pose
    }

    override fun start() {
        follower!!.startTeleopDrive()
        follower!!.update()
        Robot.pose = follower!!.pose
    }

    fun tickInput() {
        if(driverPad.y) {
            // set field centric
            if(driverPad.aWasPressed()) {
                fieldCentric = true
                driverPad.rumble(1.0, 1.0, 500)
            }

            // set robot centric
            if(driverPad.bWasPressed()) {
                fieldCentric = false
                driverPad.rumble(1.0, 1.0, 500)
            }
        }
    }

    fun tickDrive() {
        val slowScale = 0.8
        var leftY = -driverPad.left_stick_y.toDouble()
        var leftX = -driverPad.left_stick_x.toDouble()
        var rightX = -driverPad.right_stick_x.toDouble()
        if(driverPad.left_bumper) {
            leftY *= slowScale
            leftX *= slowScale
            rightX *= slowScale
        }
        if(fieldCentric) {
            follower!!.setTeleOpDrive(
                Util.smooth(leftY) * Alliance.map(-1, 1),
                Util.smooth(leftX) * Alliance.map(-1, 1),
                Util.smooth(rightX),
                false
            )
        }
        else {
            follower!!.setTeleOpDrive(
                Util.smooth(leftY),
                Util.smooth(leftX),
                Util.smooth(rightX),
                true
            )
        }
        follower!!.update()
        Robot.pose = follower!!.pose
    }

    override fun tick() {
        tickInput()
        tickDrive()
    }

    public override fun logging() {
        Logger.add("------------------------")
        Logger.add("X Pos", "%.2f", Robot.pose.x)
        Logger.add("Y Pos", "%.2f", Robot.pose.y)
        Logger.add("Heading", "%.2f°", Math.toDegrees(Robot.pose.heading))
        Logger.add("------------------------")
        Util.logActions(runningActions)
        Logger.add("------------------------")
    }
}
