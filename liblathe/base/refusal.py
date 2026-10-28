"""
Refusals that can be waived: a check raises Refusal(check, reason); a check named in an op's
ignored_checks becomes a warning and the path is made anyway. Unusable inputs stay plain errors.
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
