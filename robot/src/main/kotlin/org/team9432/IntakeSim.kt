package org.team9432

import kotlinx.coroutines.CancellableContinuation
import kotlinx.coroutines.suspendCancellableCoroutine
import org.team9432.lib.RobotPeriodicManager
import org.team9432.lib.unit.Translation2d
import org.team9432.lib.unit.inches
import org.team9432.resources.intake.Intake
import org.team9432.resources.swerve.mapleswerve.utils.CompetitionFieldUtils.Simulations.IntakeSimulation
import java.util.*
import kotlin.coroutines.resume

private val intakeWidth = 18.inches
private val intakeX = -14.325.inches /** The extra bit is bc the sim doesn't let notes go under the bumpers */- 1.5.inches
private val intakeStart = Translation2d(intakeX, intakeWidth / 2)
private val intakeEnd = Translation2d(intakeX, -intakeWidth / 2)

object IntakeSim: IntakeSimulation(intakeStart, intakeEnd, capacity = 1, { Intake.isIntaking }) {
    private val awaitingNoteQueue: Queue<CancellableContinuation<Unit>> = LinkedList()

    init {
        RobotPeriodicManager.startPeriodic { update() }
    }

    private fun update() {
        if (super.gamePieceCount == 1) {
            while (awaitingNoteQueue.isNotEmpty()) {
                awaitingNoteQueue.poll().resume(Unit)
            }

            if (Beambreaks.hasNoNote) {
                println("Clearing Note!")
                super.gamePieceCount = 0
            }
        }
    }

    suspend fun awaitPickup() = suspendCancellableCoroutine { continuation ->
        awaitingNoteQueue.add(continuation)
    }
}