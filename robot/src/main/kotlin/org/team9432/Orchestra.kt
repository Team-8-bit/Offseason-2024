package org.team9432

import com.ctre.phoenix6.Orchestra
import org.team9432.resources.swerve.Swerve

object Orchestra {
    private val orchestra = Orchestra()

    init {
        Swerve.getTalons().forEach { orchestra.addInstrument(it) }
    }

    fun loadAndPlay(track: String) {
        orchestra.loadMusic(track)
        orchestra.play()
    }
}