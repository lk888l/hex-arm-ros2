"""Launch-local notification emitted by the verified startup process."""
from launch import Event


class StartupVerified(Event):
    def __init__(self, profile, readiness_token):
        super().__init__()
        self.profile = str(profile)
        self.readiness_token = readiness_token
