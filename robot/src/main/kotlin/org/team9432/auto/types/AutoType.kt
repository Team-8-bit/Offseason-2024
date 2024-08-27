package org.team9432.auto.types

import org.team9432.auto.paths.AutoFieldConstants.CenterNote
import org.team9432.auto.paths.AutoFieldConstants.CloseNote
import org.team9432.lib.dashboard.AutoSelector

sealed interface Auto

object AutoType {
    data class FourNote(
        val ampFirst: Boolean,
        val endAction: EndAction,
        val centerNote: CenterNote,
    ): Auto {
        init {
            assert(centerNote in validCenterNotes)
        }

        companion object {
            val validCenterNotes = setOf(CenterNote.ONE, CenterNote.TWO, CenterNote.THREE, CenterNote.FOUR)

            val options = buildSet {
                for (ampFirst in setOf(false, true))
                    for (centerNote in validCenterNotes)
                        for (endAction in EndAction.entries)
                            add(FourNote(ampFirst, endAction, centerNote))
            }


            fun addOptionToSelector(selector: AutoSelector.AutoSelectorOptionScope<Auto>) = selector.apply {
                var ampFirst: Boolean = false
                var centerNote: CenterNote = CenterNote.ONE
                var endAction: EndAction = EndAction.DO_NOTHING
                val getAuto = { FourNote(ampFirst, endAction, centerNote) }

                addOption("FourNote", getAuto) {
                    addQuestion("Start Note", { ampFirst = it == CloseNote.AMP }) {
                        addOption("Amp", { CloseNote.AMP })
                        addOption("Stage", { CloseNote.STAGE })
                    }

                    addQuestion("End Action", { endAction = it }) {
                        addOption("Nothing", { EndAction.DO_NOTHING })
                        addOption("Drive To Center", { EndAction.DRIVE_TO_CENTER })

                        addOption("Score Centerline", { EndAction.SCORE_CENTERLINE }) {
                            addQuestion("Which Note", { centerNote = it }) {
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
}
