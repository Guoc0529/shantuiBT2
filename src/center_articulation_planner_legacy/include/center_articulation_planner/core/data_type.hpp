#pragma once

// === Planner state machine ===
enum class PlannerState
{
    Idle,                // Waiting for goal
    PlanningGlobalPath,  // Computing global path
    AdjustingDirection,  // Align forward/backward direction
    AligningInPlace,     // Align heading in place (placeholder)
    Moving,              // Following path
    CheckingGoal,        // Waiting for final goal confirmation
    BackingOutUnload,    // Backing out and re-entering after unload yaw error
    Completed,           // Done
    Error                // Error
};
