# Pre-competition checklist: vision-bench

Generated from `Robots.java` by the build (`./gradlew competitionChecklist`, part of every build): don't edit it here, change the robot instead.

## Which camera goes where

| Station | Camera (its label) | Its serial ends | Type | Read by |
|---|---|---|---|---|
| bench-left | 1 | `F621` | ThriftyCam | Visor |
| bench-right | 2 | `E7DA` | ThriftyCam | Visor |

No camera is tied to a coprocessor in the code: the robot finds the board each camera on Visor is plugged into by its device, and the camera's card on Pitwall's Pit view names the board its frames come from.

## In the pit

- [ ] bench-left holds camera 1, its serial ending `F621`.
- [ ] bench-right holds camera 2, its serial ending `E7DA`.
- [ ] bench-left reads a tag on its Pitwall card, with no dark, clipping or incomplete-frame warning.
- [ ] bench-right reads a tag on its Pitwall card, with no dark, clipping or incomplete-frame warning.
- [ ] The event's venue is chosen on the venue card. With no survey of the field, make it there: New venue for this event, with the event's name.

## On the field

On the event's field, with the robot disabled and no field system connected (the field's calibration time, not a match), from each camera's card on Pitwall's Pit view:

- [ ] Read bench-left's dark and clipping warnings.
- [ ] Read bench-right's dark and clipping warnings.
- [ ] Adjust the ThriftyCams' gain once, from one of their cards, until none warns dark or clipping, and keep it: Keep in this venue: every ThriftyCam.
- [ ] Download the venue from the venue card, and commit it to `src/main/deploy/venues/`.
