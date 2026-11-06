# Adapted for FluxVLA on 2026-10-08; see NOTICE.md for source and license.
#!/usr/bin/env python

import logging

logger = logging.getLogger(__name__)


class VREventHandler:
    """
    Map left-controller thumbstick to LeRobot recording events.

    Left thumbstick:
      - right  -> exit current episode early
      - left   -> re-record episode
      - down   -> stop recording
      - up     -> reset request (semantics depend on the robot backend)
    """

    def __init__(self, vr_monitor, threshold: float = 0.7):
        self.vr_monitor = vr_monitor
        self.threshold = threshold
        self.events = {
            "exit_early": False,
            "rerecord_episode": False,
            "stop_recording": False,
            "reset_arm": False,
        }
        self._prev = {"x": 0.0, "y": 0.0}

    def update_events(self) -> dict[str, bool]:
        dual = self.vr_monitor.get_latest_goal_nowait()
        left_goal = dual.get("left") if isinstance(dual, dict) else None
        if left_goal is None or not getattr(left_goal, "metadata", None):
            return self.events.copy()

        thumb = left_goal.metadata.get("thumbstick", {}) or {}
        x = float(thumb.get("x", 0.0))
        y = float(thumb.get("y", 0.0))

        if x > self.threshold and self._prev["x"] <= self.threshold:
            logger.info("VR left thumbstick right -> exit_early")
            self.events["exit_early"] = True
        elif x < -self.threshold and self._prev["x"] >= -self.threshold:
            logger.info("VR left thumbstick left -> rerecord_episode")
            self.events["rerecord_episode"] = True
            self.events["exit_early"] = True
        if y > self.threshold and self._prev["y"] <= self.threshold:
            logger.info("VR left thumbstick down -> stop_recording")
            self.events["stop_recording"] = True
            self.events["exit_early"] = True
        elif y < -self.threshold and self._prev["y"] >= -self.threshold:
            logger.info("VR left thumbstick up -> reset_arm")
            self.events["reset_arm"] = True

        self._prev = {"x": x, "y": y}
        return self.events.copy()

    def reset_events(self) -> None:
        for key in self.events:
            if key != "reset_arm":
                self.events[key] = False

    def take_reset_arm(self) -> bool:
        flag = bool(self.events.get("reset_arm", False))
        self.events["reset_arm"] = False
        return flag

    def print_control_guide(self) -> None:
        logger.info(
            "VR recording controls (left thumbstick): "
            "right=next episode / start recording, left=re-record, down=stop, up=reset request"
        )


class RecordingButtons:
    """Right modifier + left stick; fresh neutral before/after mode changes.

    Releases/reconnects cannot replay commands. Direction edges require a neutral
    sample, diagonal input is ignored. Left gripper stays gated until recentered.
    """
    def __init__(self, modifier_button='a', threshold=.7, neutral=.2, max_age_s=.25):
        if modifier_button not in ('a', 'b', 'thumbstick'):
            raise ValueError('modifier_button must be a, b or thumbstick')
        if not 0 < neutral < threshold <= 1 or not 0 < max_age_s <= 1:
            raise ValueError('Invalid recording button thresholds')
        self.button = modifier_button
        self.threshold, self.neutral, self.max_age_s = threshold, neutral, max_age_s
        self.reset()

    def reset(self):
        self.released = False
        self.mode = False
        self.armed = False
        self.block_left = True
        self.last_sequences = None

    def update(self, raw):
        import math
        try:
            hands = [raw[side] for side in ('left', 'right')]
            if any(not math.isfinite(h['xlevr.sample_age_s']) or
                   not 0 <= h['xlevr.sample_age_s'] <= self.max_age_s for h in hands):
                raise ValueError('stale')
            x = float(hands[0].get('xlevr.thumbstick', {}).get('x', 0))
            y = float(hands[0].get('xlevr.thumbstick', {}).get('y', 0))
            if not all(math.isfinite(v) and abs(v) <= 1 for v in (x, y)):
                raise ValueError('invalid axes')
            seq = tuple(h['xlevr.sample_sequence'] for h in hands)
            pressed = bool(hands[1].get('xlevr.buttons', {}).get(self.button, False))
        except (KeyError, TypeError, ValueError):
            self.reset()
            return dict(command=None, block_left=True, fresh=False, mode=False)
        centered = abs(x) <= self.neutral and abs(y) <= self.neutral
        command = None
        restarted = self.last_sequences is not None and any(a < b for a, b in zip(seq, self.last_sequences))
        if restarted:
            self.reset()  # browser/session sequence restarted
        duplicate = seq == self.last_sequences
        self.last_sequences = seq
        if not self.released:
            if not pressed and centered:
                self.released = True; self.block_left = False
        elif not pressed:
            self.mode = False; self.armed = False
            if centered: self.block_left = False
        else:
            self.block_left = True
            if not self.mode:
                self.mode = True; self.armed = False
            if centered:
                self.armed = True
            elif self.armed and not duplicate:
                self.armed = False  # diagonals also require recentering
                if abs(y) <= self.neutral and abs(x) >= self.threshold:
                    command = 'right' if x > 0 else 'left'
                elif abs(x) <= self.neutral and abs(y) >= self.threshold:
                    command = 'down' if y > 0 else 'up'
        return dict(command=command, block_left=self.block_left, fresh=True, mode=self.mode, restarted=restarted)
