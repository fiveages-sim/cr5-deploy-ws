# Hand2 joint index calibration

The mapping in `include/wujihand2_ros2_control/hands/hand2/wuji_hand2_protocol.hpp`
is an **assumption**: SDK command/state index `i` ↔ `hand2.yaml` joint order.

Official docs guarantee index `0..19` and labels `{finger}_S{1..4}`, not a
frozen table against our URDF names. Calibrate on hardware before production.

## Procedure

1. Launch real HI (scan: omit `device_address`; or use `device_address` / `serial_number` — left `.110:50001` / right `.111` per Wuji docs).
2. Hold all joints; confirm activate left `cmd == state` (no jump).
3. For SDK index `k = 0 .. 19`:
   - Send a small position step on only that SDK slot (temporary debug) **or**
     claim the corresponding ROS joint from `kJointNameSuffixes[k]` and step it.
   - Confirm visually / in `/joint_states` that **only** the expected URDF joint moves.
4. If mismatch: edit `kJointNameSuffixes` order (or add an explicit remapping table) and rebuild.
5. Freeze the table in git; do not flip signs for `hand_base` +Z (`wrist_align`).

## Assumed table (pre-calibration)

| SDK | ROS suffix |
|-----|------------|
| 0–3 | thumb_cmc_flex, thumb_cmc_abd, thumb_mcp, thumb_ip |
| 4–7 | index_finger_* |
| 8–11 | middle_finger_* |
| 12–15 | ring_* |
| 16–19 | pinky_* |
