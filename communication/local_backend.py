"""In-process simulator link. It is NOT Wi-Fi/UDP and does not claim hardware ACKs."""
from core.models import Command, CommandResult, Telemetry
from simulation.engine import SimulationEngine
from simulation.scenarios import ScenarioRunner


class LocalBackend:
    def __init__(self) -> None:
        self.engine = SimulationEngine()
        self.scenario = ScenarioRunner(self.engine)
        self.paused = False

    def send_command(self, command: Command) -> CommandResult:
        return self.engine.command(command)

    def tick(self, dt: float = .05) -> Telemetry | None:
        self.scenario.before_step()
        if not self.paused:
            self.engine.advance(dt)
        self.scenario.after_step()
        sample = self.engine.snapshot()
        # Even when telemetry is dropped the virtual onboard controller keeps running.
        if self.engine.time < self.engine.dropout_until:
            return None
        return sample

    def close(self) -> None:
        from core.models import CommandKind
        self.send_command(Command(CommandKind.DISARM))
        self.paused = True
