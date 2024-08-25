package org.team9432.auto.types

import org.team9432.auto.AutoSelectorOptions
import org.team9432.auto.paths.AutoFieldConstants.CenterNote
import org.team9432.auto.paths.AutoFieldConstants.CenterNote.*

sealed interface Auto {
    val name: String
}

object AutoType {
    data class FourNote(
        val ampFirst: Boolean,
        val centerNote: CenterNote?,
    ): Auto {
        init {
            assert(centerNote in validCenterNotes)
        }

        override val name: String
            get() {
                val ampFirst = if (ampFirst) "AmpFirst" else ""
                val centerNote = centerNote?.readableName?.let { "AndCenter$it" } ?: ""

                return "${ampFirst}CloseFour$centerNote"
            }

        companion object {
            val validCenterNotes = setOf(null, ONE, TWO, THREE, FOUR)

            val options = buildSet {
                for (ampFirst in setOf(false, true))
                    for (centerNote in validCenterNotes)
                        add(FourNote(ampFirst, centerNote))
            }

            enum class EndAction {
                NOTHING, DRIVE_TO_CENTER, SCORE_CENTER
            }

            var ampFirst = true
            var centerNote: CenterNote? = null
            var endAction: EndAction = EndAction.NOTHING

            fun AutoSelectorOptions.applyFourNoteSelectorOptions() {
                addQuestion("Start Note") {
                    addOption("Amp") { ampFirst = true }
                    addOption("Stage") { ampFirst = false }
                }

                addQuestion("End Action") {
                    addOption("Nothing") { endAction = EndAction.NOTHING }

                    addOption("Drive To Center") {
                        endAction = EndAction.DRIVE_TO_CENTER

                        addQuestion("Center End Position") {
                            //examples
                            addOption("Source") {}
                            addOption("Center") {}
                            addOption("Amp") {}
                        }
                    }

                    addOption("Score Centerline") {
                        endAction = EndAction.SCORE_CENTER

                        addQuestion("Which Note") {
                            validCenterNotes.filterNotNull().forEach {
                                addOption(it.readableName) { centerNote = it }
                            }
                        }
                    }
                }
            }
        }
    }
}
