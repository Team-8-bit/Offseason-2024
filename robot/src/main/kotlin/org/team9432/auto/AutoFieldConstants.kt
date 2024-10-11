//package org.team9432.auto
//
//import org.team9432.lib.unit.feet
//import org.team9432.lib.unit.inches
//
//object AutoFieldConstants {
//    /* -------- Note Positions -------- */
//
//    /** X coordinate of the spike notes. */
//    private val spikeNoteXCoordinate = 9.0.feet + 6.0.inches
//
//    /** Y spacing between spike notes. */
//    private val spikeNoteYSpacing = 4.0.feet + 9.0.inches
//
//    /** Y spacing between center notes. */
//    private val centerNoteYSpacing = 5.0.feet + 6.0.inches
//
//    enum class CloseNote {
//        AMP, SPEAKER, STAGE;
//
//        val readableName = name.lowercase().replaceFirstChar { it.uppercase() }
//    }
//
//    enum class CenterNote {
//        ONE, TWO, THREE, FOUR, FIVE;
//
//        val readableName = name.lowercase().replaceFirstChar { it.uppercase() }
//    }
//}