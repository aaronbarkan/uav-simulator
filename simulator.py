"""Minimal outbound-only MAVLink 2 telemetry simulator (no flight dynamics)."""

import argparse
import math
import os
import select
import signal
import sys
import time
from dataclasses import dataclass

# Set before importing pymavlink, including during direct Python execution.
os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil


EARTH_RADIUS_M = 6_371_000.0


@dataclass(frozen=True)
class UavState:
    latitude: float
    longitude: float
    alt_msl_m: float
    relative_alt_m: float
    north_mps: float
    east_mps: float
    down_mps: float
    roll: float
    pitch: float
    yaw: float


def state_at(args, elapsed):
    """Pure short-distance approximation; elapsed is active simulation seconds."""
    heading = math.radians(args.heading_deg)
    north = east = 0.0
    lat, lon = args.lat, args.lon
    if args.scenario == "straight":
        north = args.speed_mps * math.cos(heading)
        east = args.speed_mps * math.sin(heading)
        lat += math.degrees(north * elapsed / EARTH_RADIUS_M)
        lon += math.degrees(east * elapsed / (
            EARTH_RADIUS_M * math.cos(math.radians(args.lat))))
        # The local approximation is undefined near/crossing a pole.
        if not -89 <= lat <= 89:
            raise ValueError("flight left the supported latitude range [-89, 89]")
        lon = (lon + 180) % 360 - 180
    yaw = (heading + math.pi) % (2 * math.pi) - math.pi
    return UavState(lat, lon, args.alt_msl_m, args.relative_alt_m,
                    north, east, 0.0, 0.0, 0.0, yaw)


def position_fields(state, uptime):
    """MAVLink integer units; boot milliseconds wrap at the uint32 boundary."""
    return (
        int(uptime * 1000) & 0xFFFFFFFF,
        round(state.latitude * 1e7), round(state.longitude * 1e7),
        round(state.alt_msl_m * 1000), round(state.relative_alt_m * 1000),
        round(state.north_mps * 100), round(state.east_mps * 100),
        round(state.down_mps * 100),
        round(math.degrees(state.yaw) % 360 * 100) % 36000,
    )


class MavlinkSender:
    def __init__(self, host, port):
        self.connection = mavutil.mavlink_connection(
            f"udpout:{host}:{port}", source_system=1,
            source_component=mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            dialect="common",
        )

    def heartbeat(self):
        self.connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
            mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
            0, mavutil.mavlink.MAV_STATE_ACTIVE,
        )

    def telemetry(self, state, uptime):
        fields = position_fields(state, uptime)
        self.connection.mav.global_position_int_send(*fields)
        self.connection.mav.attitude_send(
            fields[0], state.roll, state.pitch, state.yaw, 0.0, 0.0, 0.0,
        )

    def close(self):
        self.connection.close()


@dataclass
class Periodic:
    interval: float
    deadline: float

    def due(self, now):
        if now < self.deadline:
            return False
        # Keep deadlines on the original cadence, skipping missed slots.
        self.deadline += (math.floor((now - self.deadline) / self.interval) + 1) * self.interval
        return True


class Simulation:
    def __init__(self, args, sender, now):
        self.args = args
        self.sender = sender
        self.started = now
        self.paused_at = None
        self.paused_seconds = 0.0
        self.heartbeat = Periodic(1.0, now)
        self.telemetry = Periodic(1.0 / args.rate_hz, now)

    def elapsed(self, now):
        end = now if self.paused_at is None else self.paused_at
        return end - self.started - self.paused_seconds

    def tick(self, now):
        if self.paused_at is not None:
            return
        if self.heartbeat.due(now):
            self.sender.heartbeat()
        if self.telemetry.due(now):
            state = state_at(self.args, self.elapsed(now))
            self.sender.telemetry(state, now - self.started)

    def command(self, command, now):
        command = command.strip().lower()
        if command == "quit":
            return False
        if command == "pause":
            if self.paused_at is None:
                self.paused_at = now
            print("Paused", flush=True)
        elif command == "resume":
            if self.paused_at is not None:
                self.paused_seconds += now - self.paused_at
                self.paused_at = None
                self.heartbeat.deadline = self.telemetry.deadline = now
            print("Running", flush=True)
        elif command == "status":
            state = state_at(self.args, self.elapsed(now))
            mode = "running" if self.paused_at is None else "paused"
            print(f"{mode} {self.args.scenario} t={self.elapsed(now):.2f}s "
                  f"lat={state.latitude:.7f} lon={state.longitude:.7f} "
                  f"alt={state.alt_msl_m:.2f}m heading={self.args.heading_deg:g}deg "
                  f"speed={math.hypot(state.north_mps, state.east_mps):g}m/s", flush=True)
        elif command == "help":
            print("Commands: status, pause, resume, help, quit", flush=True)
        elif command:
            print("Unknown command. Commands: status, pause, resume, help, quit", flush=True)
        return True


def run(args):
    running = True

    def stop(signum, frame):
        nonlocal running
        running = False

    previous_handlers = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    sender = None
    try:
        sender = MavlinkSender(args.host, args.port)
        simulation = Simulation(args, sender, time.monotonic())
        try:
            stdin_fd = sys.stdin.fileno()
        except (AttributeError, OSError, ValueError):
            stdin_fd = None
        pending = b""
        discarding_line = False
        print(f"Sending {args.scenario} to {args.host}:{args.port} "
              f"at {args.rate_hz:g} Hz; heartbeat 1 Hz. Type help for commands.", flush=True)
        while running:
            now = time.monotonic()
            timeout = 0.1 if simulation.paused_at is not None else max(0.0, min(
                0.1, simulation.heartbeat.deadline - now, simulation.telemetry.deadline - now))
            if stdin_fd is None:
                time.sleep(timeout)
            else:
                try:
                    readable, _, _ = select.select([stdin_fd], [], [], timeout)
                    if readable:
                        # Read bytes rather than readline: pipes may contain partial or
                        # multiple lines, and Python's text buffering hides ready input.
                        chunk = os.read(stdin_fd, 4096)
                        eof = not chunk
                        if discarding_line:
                            if b"\n" in chunk:
                                _, chunk = chunk.split(b"\n", 1)
                                discarding_line = False
                            else:
                                # Even a command-looking suffix belongs to the bad line.
                                chunk = b""
                        pending += chunk
                        if eof:
                            stdin_fd = None
                            if pending:
                                pending += b"\n"
                        while b"\n" in pending:
                            line, pending = pending.split(b"\n", 1)
                            if len(line) > 4096:
                                print("Command too long; discarded", flush=True)
                                continue
                            if not simulation.command(line.decode(errors="replace"), time.monotonic()):
                                running = False
                                break
                        if len(pending) > 4096:
                            pending = b""
                            discarding_line = True
                            print("Command too long; discarded", flush=True)
                except (OSError, ValueError):
                    stdin_fd = None
            if running:
                simulation.tick(time.monotonic())
    finally:
        if sender is not None:
            sender.close()
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
    print("Stopped", flush=True)


def bounded_float(low, high):
    def parse(value):
        number = float(value)
        if not math.isfinite(number) or not low <= number <= high:
            raise argparse.ArgumentTypeError(f"must be finite and between {low} and {high}")
        return number
    return parse


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__,
        epilog="Straight flight uses a short-distance approximation and stops outside "
               "latitude [-89, 89]. The pause command suppresses all telemetry, including "
               "heartbeat, for connection-timeout testing.",
    )
    parser.add_argument("--host", default="host.docker.internal")
    parser.add_argument("--port", type=int, default=14550)
    parser.add_argument("--scenario", choices=("hover", "straight"), default="hover")
    parser.add_argument("--rate-hz", type=bounded_float(0.1, 100), default=5.0)
    parser.add_argument("--speed-mps", type=bounded_float(0, 327.67), default=5.0,
                        help="speed in m/s (straight scenario only)")
    parser.add_argument("--heading-deg", type=bounded_float(0, 360), default=90.0)
    parser.add_argument("--lat", type=bounded_float(-89, 89), default=32.0)
    parser.add_argument("--lon", type=bounded_float(-180, 180), default=34.8)
    parser.add_argument("--alt-msl-m", type=bounded_float(-2147483, 2147483), default=100.0)
    parser.add_argument("--relative-alt-m", type=bounded_float(-2147483, 2147483), default=50.0)
    args = parser.parse_args(argv)
    if not args.host.strip() or ":" in args.host or any(c.isspace() for c in args.host):
        parser.error("--host must be an IPv4 address or hostname without a port")
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    args.heading_deg %= 360
    return args


if __name__ == "__main__":
    try:
        run(parse_args())
    except (OSError, ValueError) as error:
        print(f"simulator: {error}", file=sys.stderr)
        sys.exit(1)
