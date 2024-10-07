package org.team9432.resources.drive.controllers

abstract class GenericDriveController<Output> {
    abstract fun calculate(): Output
    abstract fun atGoal(): Boolean
}