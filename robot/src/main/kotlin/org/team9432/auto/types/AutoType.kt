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

            fun AutoSelectorOptions.applyFourNoteSelectorOptions() {
                addQuestion("Start Note") {
                    addOption("Amp")
                    addOption("Stage")
                }

                addQuestion("End Action") {
                    addOption("Nothing")

                    addOption("Drive To Center") {
                        addQuestion("Center End Position") {
                            //examples
                            addOption("Source")
                            addOption("Center")
                            addOption("Amp")
                        }
                    }

                    addOption("Score Centerline") {
                        addQuestion("Which Note") {
                            validCenterNotes.filterNotNull().forEach { addOption(it.readableName) }
                        }
                    }
                }
            }
        }
    }
}
