package org.team9432.auto.paths

import org.team9432.choreogenerator.Position
import org.team9432.lib.constants.EvergreenFieldConstants
import org.team9432.lib.unit.feet
import org.team9432.lib.unit.inches
import org.team9432.lib.unit.meters

object AutoFieldConstants {
    /* -------- Note Positions -------- */

    /** X coordinate of the spike notes. */
    private val spikeNoteXCoordinate = 9.0.feet + 6.0.inches

    /** Y spacing between spike notes. */
    private val spikeNoteYSpacing = 4.0.feet + 9.0.inches

    /** Y spacing between center notes. */
    private val centerNoteYSpacing = 5.0.feet + 6.0.inches

    val ampNote = Position(spikeNoteXCoordinate, EvergreenFieldConstants.centerY + (spikeNoteYSpacing * 2))
    val speakerNote = Position(spikeNoteXCoordinate, EvergreenFieldConstants.centerY + spikeNoteYSpacing)
    val stageNote = Position(spikeNoteXCoordinate, EvergreenFieldConstants.centerY)
    val centerNoteOne = Position(EvergreenFieldConstants.centerX, EvergreenFieldConstants.centerY + (centerNoteYSpacing * 2))
    val centerNoteTwo = Position(EvergreenFieldConstants.centerX, EvergreenFieldConstants.centerY + (centerNoteYSpacing * 1))
    val centerNoteThree = Position(EvergreenFieldConstants.centerX, EvergreenFieldConstants.centerY + (centerNoteYSpacing * 0))
    val centerNoteFour = Position(EvergreenFieldConstants.centerX, EvergreenFieldConstants.centerY + (centerNoteYSpacing * -1))
    val centerNoteFive = Position(EvergreenFieldConstants.centerX, EvergreenFieldConstants.centerY + (centerNoteYSpacing * -2))

    val speakerYCoordinate = EvergreenFieldConstants.centerY + spikeNoteYSpacing

    val speakerAimPosition = Position(0.35.meters, speakerYCoordinate)

    fun getNotePose(note: CloseNote) = when (note) {
        CloseNote.AMP -> ampNote
        CloseNote.SPEAKER -> speakerNote
        CloseNote.STAGE -> stageNote
    }

    fun getNotePose(note: CenterNote) = when (note) {
        CenterNote.ONE -> centerNoteOne
        CenterNote.TWO -> centerNoteTwo
        CenterNote.THREE -> centerNoteThree
        CenterNote.FOUR -> centerNoteFour
        CenterNote.FIVE -> centerNoteFive
    }

    enum class CloseNote {
        AMP, SPEAKER, STAGE;

        val readableName = name.lowercase().replaceFirstChar { it.uppercase() }
    }

    enum class CenterNote {
        ONE, TWO, THREE, FOUR, FIVE;

        val readableName = name.lowercase().replaceFirstChar { it.uppercase() }
    }
}