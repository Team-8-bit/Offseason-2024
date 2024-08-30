package org.team9432.auto.types

import org.team9432.auto.paths.AutoFieldConstants.CenterNote
import org.team9432.auto.paths.AutoFieldConstants.CloseNote
import org.team9432.auto.paths.FourNotePaths
import org.team9432.choreogenerator.ChoreoTrajectory
import org.team9432.lib.dashboard.AutoSelector

sealed interface Auto {
    fun getTrajectoryNames(): List<String>
}

class AutoSegment(val name: String, generateTrajectory: ChoreoTrajectory.ChoreoTrajectoryBuilder.() -> Unit) {
    val builtTrajectory by lazy { ChoreoTrajectory.new(name, block = generateTrajectory) }
}

data class FourNote(
    val startingPosition: StartingPosition,
    val ampFirst: Boolean,
    val endAction: EndAction,
    val centerNote: CenterNote,
): Auto {
    init {
        assert(centerNote in validCenterNotes)
    }

    override fun getTrajectoryNames() = FourNotePaths.getSegmentsFor(this).map { it.name }

    companion object {
        val validCenterNotes = setOf(CenterNote.ONE, CenterNote.TWO, CenterNote.THREE, CenterNote.FOUR)

        val options = buildSet {
            for (ampFirst in setOf(false, true))
                for (centerNote in validCenterNotes)
                    for (endAction in EndAction.entries)
                        for (startPosition in StartingPosition.entries)
                            add(FourNote(startPosition, ampFirst, endAction, centerNote))
        }

        fun addOptionToSelector(selector: AutoSelector.AutoSelectorOptionScope<Auto>) = selector.apply {
            var startingPosition: StartingPosition = StartingPosition.CENTER
            var ampFirst: Boolean = false
            var centerNote: CenterNote = CenterNote.ONE
            var endAction: EndAction = EndAction.DO_NOTHING
            val getAuto = { FourNote(startingPosition, ampFirst, endAction, centerNote) }

            addOption("FourNote", getAuto) {
                addQuestion("Starting Position", { startingPosition = it }) {
                    addOption("Center Subwoofer", { StartingPosition.CENTER })
                    addOption("Amp Subwoofer", { StartingPosition.AMP })
                    addOption("Stage Subwoofer", { StartingPosition.STAGE })
                }

                addQuestion("Start Note", { ampFirst = it == CloseNote.AMP }) {
                    addOption("Amp", { CloseNote.AMP })
                    addOption("Stage", { CloseNote.STAGE })
                }

                addQuestion("End Action", { endAction = it }) {
                    addOption("Nothing", { EndAction.DO_NOTHING })
                    addOption("Drive To Center", { EndAction.DRIVE_TO_CENTER })

                    addOption("Score Centerline", { EndAction.SCORE_CENTERLINE }) {
                        addQuestion("Which Note?", { centerNote = it }) {
                            validCenterNotes.forEach {
                                addOption(it.readableName, { it })
                            }
                        }
                    }
                }
            }
        }
    }

    enum class EndAction {
        DO_NOTHING, DRIVE_TO_CENTER, SCORE_CENTERLINE
    }

    enum class StartingPosition {
        AMP, STAGE, CENTER;

        val readableName = name.lowercase().replaceFirstChar { it.uppercase() }
    }
}