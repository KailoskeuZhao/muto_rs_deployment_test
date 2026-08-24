# Agent instructions

Before changing the commander, natural-language router, object search,
registry-confirmation path, command layer, frontier integration, or their
behavioral tests, read `docs/commander_stack_reference.md` completely.

That note describes the current stack but is not guaranteed to be correct or
current. Verify its claims against source code, ROS interfaces, launch wiring,
parameters, installed artifacts, tests, and runtime traces. If your verified
change makes the description stale, update the note in the same change.

The deployment target is ROS 2 Humble on aarch64. Do not alter production
parameters or interfaces merely to make this checkout run under a Jazzy host.
Run connected behavior validation in the matching Humble environment.
