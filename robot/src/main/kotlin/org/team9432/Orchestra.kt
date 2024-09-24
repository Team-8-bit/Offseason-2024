package org.team9432

import com.ctre.phoenix6.Orchestra

object Orchestra {
    private val orchestra = Orchestra()

    init {
//        Swerve.getTalons().forEach { orchestra.addInstrument(it) }
    }

    fun loadAndPlay(track: String) {
        orchestra.loadMusic(track)
        orchestra.play()
    }
}