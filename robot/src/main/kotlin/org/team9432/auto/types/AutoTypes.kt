package org.team9432.auto.types

import org.team9432.auto.paths.AmpsideCenterlinePaths
import org.team9432.auto.paths.AutoFieldConstants.CenterNote
import org.team9432.auto.paths.AutoFieldConstants.CloseNote
import org.team9432.auto.paths.FarsideCenterlinePaths
import org.team9432.auto.paths.FourNotePaths
import org.team9432.choreogenerator.ChoreoTrajectory
import org.team9432.lib.dashboard.AutoSelector

sealed interface Auto {
    fun getTrajectoryNames(): List<String>
}

class AutoSegment(val name: String, generateTrajectory: ChoreoTrajectory.() -> Unit) {
    val builtTrajectory by lazy { ChoreoTrajectory(name).apply(generateTrajectory) }
}

enum class StartingPosition {
    AMP, STAGE, CENTER;

    val readableName = name.lowercase().replaceFirstChar { it.uppercase() }
}

data class AmpsideCenterline(
    val startingPosition: StartingPosition,
    val scoreCloseAmpNote: Boolean,
    val notes: List<CenterNote>,
): Auto {
    init {
        assert(notes.all { it in validCenterNotes })
    }

    override fun getTrajectoryNames() = AmpsideCenterlinePaths.getSegmentsFor(this).map { it.name }

    companion object {
        val validCenterNotes = setOf(CenterNote.ONE, CenterNote.TWO)

        val options by lazy {
            buildSet {
                for (startPosition in StartingPosition.entries)
                    for (notes in listOf(listOf(CenterNote.ONE, CenterNote.TWO), listOf(CenterNote.TWO, CenterNote.ONE), listOf(CenterNote.TWO), listOf(CenterNote.ONE)))
                        for (scoreCloseAmpNote in listOf(true, false)) {
                            add(AmpsideCenterline(startPosition, scoreCloseAmpNote, notes))
                        }
            }
        }

        fun addOptionToSelector(selector: AutoSelector.AutoSelectorOptionScope<Auto>) = selector.apply {
            var startingPosition: StartingPosition = StartingPosition.CENTER
            var notes: List<CenterNote> = emptyList()
            var scoreCloseAmpNote: Boolean = false
            val getAuto = { AmpsideCenterline(startingPosition, scoreCloseAmpNote, notes) }

            addOption("Ampside Centerline", getAuto) {
                addQuestion("Starting Position", { startingPosition = it }) {
                    addOption("Center Subwoofer", { StartingPosition.CENTER })
                    addOption("Amp Subwoofer", { StartingPosition.AMP })
                    addOption("Stage Subwoofer", { StartingPosition.STAGE })
                }

                addQuestion("Score Amp Note?", { scoreCloseAmpNote = it }) {
                    addOption("Yes", { true })
                    addOption("No", { false })
                }

                addQuestion("Notes", { notes = it }) {
                    addOption("One then Two", { listOf(CenterNote.ONE, CenterNote.TWO) })
                    addOption("Two then One", { listOf(CenterNote.TWO, CenterNote.ONE) })
                    addOption("Just One", { listOf(CenterNote.ONE) })
                    addOption("Just Two", { listOf(CenterNote.TWO) })
                }
            }
        }
    }
}

data class FarsideCenterline(
    val startingPosition: StartingPosition,
    val notes: List<CenterNote>,
): Auto {
    init {
        assert(notes.all { it in validCenterNotes })
    }

    override fun getTrajectoryNames() = FarsideCenterlinePaths.getSegmentsFor(this).map { it.name }

    companion object {
        val validCenterNotes = setOf(CenterNote.FOUR, CenterNote.FIVE)

        val options by lazy {
            buildSet {
                for (startPosition in StartingPosition.entries)
                    for (notes in listOf(listOf(CenterNote.FIVE, CenterNote.FOUR), listOf(CenterNote.FOUR, CenterNote.FIVE), listOf(CenterNote.FOUR), listOf(CenterNote.FIVE)))
                        add(FarsideCenterline(startPosition, notes))
            }
        }

        fun addOptionToSelector(selector: AutoSelector.AutoSelectorOptionScope<Auto>) = selector.apply {
            var startingPosition: StartingPosition = StartingPosition.CENTER
            var notes: List<CenterNote> = emptyList()
            val getAuto = { FarsideCenterline(startingPosition, notes) }

            addOption("Farside Centerline", getAuto) {
                addQuestion("Starting Position", { startingPosition = it }) {
                    addOption("Center Subwoofer", { StartingPosition.CENTER })
                    addOption("Amp Subwoofer", { StartingPosition.AMP })
                    addOption("Stage Subwoofer", { StartingPosition.STAGE })
                }

                addQuestion("Notes", { notes = it }) {
                    addOption("Five then Four", { listOf(CenterNote.FIVE, CenterNote.FOUR) })
                    addOption("Four then Five", { listOf(CenterNote.FOUR, CenterNote.FIVE) })
                    addOption("Just Four", { listOf(CenterNote.FOUR) })
                    addOption("Just Five", { listOf(CenterNote.FIVE) })
                }
            }
        }
    }
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

        val options by lazy {
            buildSet {
                for (ampFirst in setOf(false, true))
                    for (centerNote in validCenterNotes)
                        for (endAction in EndAction.entries)
                            for (startPosition in StartingPosition.entries)
                                add(FourNote(startPosition, ampFirst, endAction, centerNote))
            }
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
}