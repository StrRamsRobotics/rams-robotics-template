package org.firstinspires.ftc.ramsrobotics.util

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.control.FilteredPIDFCoefficients
import com.pedropathing.control.PIDFCoefficients
import com.pedropathing.follower.Follower
import com.pedropathing.follower.FollowerConstants
import com.pedropathing.ftc.FollowerBuilder
import com.pedropathing.ftc.drivetrains.MecanumConstants
import com.pedropathing.ftc.localization.constants.PinpointConstants
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathConstraints
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@Configurable
object Constants {
    // -------------- robot dimensions --------------
    /** inches */
    const val LENGTH: Double = 18.0
    const val HALF_LENGTH: Double = LENGTH / 2

    /** inches */
    const val WIDTH: Double = 18.0
    const val HALF_WIDTH: Double = WIDTH / 2

    // -------------- positions --------------
    val START_BLUE_FAR: Pose = Pose(48 + HALF_WIDTH, HALF_LENGTH, Math.PI / 2)
    val START_RED_FAR: Pose = Pose(96 - HALF_WIDTH, HALF_LENGTH, Math.PI / 2)
    val START_BLUE_CLOSE: Pose = Pose(24 + HALF_WIDTH, 142.8 - HALF_LENGTH, Math.PI / 2)
    val START_RED_CLOSE: Pose = Pose(120 - HALF_WIDTH, 142.8 - HALF_LENGTH, Math.PI / 2)
    val BLUE_GOAL_POSE: Pose = Pose(0.0, 144.0)
    val RED_GOAL_POSE: Pose = Pose(144.0, 144.0)

    // -------------- hardware map names --------------
    /**
     * CONTROL HUB: <br/>
     * Motors: <br/>
     * 0 - leftBack <br/>
     * 1 - rightBack <br/>
     * 2 - leftFront <br/>
     * 3 - rightFront <br/>
     *
     * EXPANSION HUB: <br/>
     * Motors: <br/>
     * 0 - intake (intake motor) <br/>
     * 1 - turntable (turntable motor) <br/>
     * 2 - flywheel1 (left flywheel motor) <br/>
     * 3 - flywheel2 (right flywheel motor)<br/>
     *
     * Servos: <br/>
     * 0 - blocker1 (left blocker servo) <br/>
     * 1 - blocker2 (right blocker servo) <br/>
     * 2 - hood1 (left hood servo) <br/>
     * 3 - hood2 (right hood servo)
     */
    // motors
    const val INTAKE_NAME: String = "intake"
    const val TURNTABLE_NAME: String = "turntable"
    const val FLYWHEEL_1_NAME: String = "flywheel1"
    const val FLYWHEEL_2_NAME: String = "flywheel2"

    // servos
    const val BLOCKER_1_NAME: String = "blocker1"
    const val BLOCKER_2_NAME: String = "blocker2"
    const val HOOD_1_NAME: String = "hood1"
    const val HOOD_2_NAME: String = "hood2"

    // -------------- april tag ids --------------
    const val BLUE_GOAL_TAG_ID: Int = 20
    const val RED_GOAL_TAG_ID: Int = 24
    val OBELISK_TAG_IDS: IntArray = intArrayOf(21, 22, 23)

    // -------------- pedro pathing constants --------------
    val followerConstants: FollowerConstants = FollowerConstants()
        .mass(10.4) // kg
        .forwardZeroPowerAcceleration(-42.41265)
        .lateralZeroPowerAcceleration(-70.77611)
        .drivePIDFCoefficients(FilteredPIDFCoefficients(0.01, 0.0, 0.0, 0.6, 0.0))
        .headingPIDFCoefficients(PIDFCoefficients(0.75, 0.0, 0.0, 0.0))
        .translationalPIDFCoefficients(PIDFCoefficients(0.07, 0.0, 0.0, 0.0))

    val driveConstants: MecanumConstants = MecanumConstants()
        .maxPower(1.0)
        .leftFrontMotorName("leftFront")
        .rightFrontMotorName("rightFront")
        .leftRearMotorName("leftBack")
        .rightRearMotorName("rightBack")
        .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
        .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
        .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
        .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
        .useBrakeModeInTeleOp(true)
        .xVelocity(80.03668)
        .yVelocity(65.60769)

    val localizerConstants: PinpointConstants = PinpointConstants()
        .hardwareMapName("pinpoint")
        .forwardPodY(4.76378)
        .strafePodX(2.16535)
        .distanceUnit(DistanceUnit.INCH)
        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)

    val pathConstraints: PathConstraints = PathConstraints(
        0.99,
        100.0,
        1.0,
        1.0
    )

    fun createFollower(hardwareMap: HardwareMap?): Follower? {
        return FollowerBuilder(followerConstants, hardwareMap)
            .mecanumDrivetrain(driveConstants)
            .pinpointLocalizer(localizerConstants)
            .pathConstraints(pathConstraints)
            .build()
    }
}


