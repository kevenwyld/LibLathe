"""
Refusals that can be waived.

An operation refuses a path it can make but that a check finds wrong (it
would cut into the part, the insert would hit it) by raising a Refusal
naming the check. Told to ignore that check (its name in the operation's
ignored_checks), the operation records a warning instead and carries on
with the path it was about to refuse. Refusals where there is no sensible
path (a zero pitch, no tool) stay plain errors.
"""

# the checks that can be waived, and what the operation does instead
CHECKS = {
    "path-intersects-part": "the path as calculated",
    "part-defeaturing-failed": "the simplified outline as it came out",
    "face-cuts-part": "faces to the plane and diameter set",
    "thread-front-clearance": "the lead-in as set, or three pitches",
    "thread-lead-in-too-long": "the lead-in as set",
    "thread-hits-behind-end": "ends the thread where it is set",
    "groove-narrower-than-blade": "cuts the blade's width",
}


class Refusal(ValueError):
    """A check's refusal: str() is the reason, .check the check's name."""

    def __init__(self, check, message):
        super().__init__(message)
        self.check = check
