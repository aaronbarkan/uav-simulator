# Minimal UAV telemetry simulator

A single-process Python 3.12 simulator that sends MAVLink 2 (`common` dialect)
telemetry over UDP. It generates a stationary hover or constant-speed straight
flight using a short-distance Earth approximation. It has no autopilot, physics
engine, GUI, command receiver, or inbound MAVLink processing.

## Docker Desktop on macOS

Start Docker Desktop, then run from this directory (Apple Silicon or Intel):

```sh
docker build -t uav-sim:dev .
docker run --rm uav-sim:dev --help
docker run --rm -it --name uav-sim uav-sim:dev \
  --host host.docker.internal --port 14550 \
  --scenario straight --rate-hz 5 --speed-mps 5
```

`host.docker.internal` addresses the Mac from the container. Start a UDP receiver
on the Mac as shown below. **Do not publish a container port** (`-p`): the
simulator only sends. Without `-it`, it continues transmitting with no interactive
stdin. Stop a detached/noninteractive container with `docker stop uav-sim`.

The base image is `python:3.12-slim`; pymavlink is pinned to `2.4.50`.
`MAVLINK20=1` is set both in the image and before the Python import. The only
direct third-party dependency is pymavlink (which installs its own dependencies).

## Direct Python execution

Use Python 3.12 and an explicit host address when running outside Docker:

```sh
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python simulator.py --host 127.0.0.1 --scenario hover
```

## Options

| Option | Default | Accepted values / units |
| --- | --- | --- |
| `--host` | `host.docker.internal` | IPv4 address or hostname, without a port |
| `--port` | `14550` | Integer 1–65535 |
| `--scenario` | `hover` | `hover` or `straight` |
| `--rate-hz` | `5` | 0.1–100 Hz for position and attitude; heartbeat stays at 1 Hz |
| `--speed-mps` | `5` | 0–327.67 m/s; used in straight flight |
| `--heading-deg` | `90` | 0–360°, clockwise from north; 360 normalizes to 0 |
| `--lat` | `32.0` | −89–89° latitude |
| `--lon` | `34.8` | −180–180° longitude |
| `--alt-msl-m` | `100` | −2147483–2147483 m above mean sea level |
| `--relative-alt-m` | `50` | −2147483–2147483 m above the chosen reference |

All numeric values must be finite. The speed and altitude limits ensure the
encoded integers fit MAVLink fields. MSL and relative altitude are independent
fixed inputs; there is no terrain model. Source system/component IDs are fixed
at **1/1**. `--help` lists the options.

Hover fixes position, altitude and orientation, with zero velocities. Straight
flight computes displacement directly from active elapsed time with Earth radius
6,371,000 m and longitude scaling at the starting latitude. North/east velocity
follows the heading; altitude stays fixed, down velocity is zero, and roll/pitch
are zero. Both scenarios use the supplied heading for yaw. Longitude wraps at
the dateline; flight stops with an error if latitude leaves [−89, 89]. This is a
local approximation for short runs, not a realistic long-distance flight model.

## Terminal commands

Enter a command followed by Return:

| Command | Effect |
| --- | --- |
| `status` | Show running/paused state, scenario, active elapsed time, position, heading and actual speed |
| `pause` | Suspend **all** transmissions, including heartbeat, and freeze motion |
| `resume` | Continue from the frozen position and immediately restart telemetry |
| `help` | List commands |
| `quit` | Close the UDP sender and exit |

Ctrl-C and SIGTERM also exit cleanly. Stdin is polled without blocking; EOF
removes it from polling and transmission continues. There is no per-packet log. Oversized command lines (over 4096 bytes) are
discarded through the next newline; the following command is accepted.

Scheduling uses monotonic deadlines, skipping missed slots rather than sending a
catch-up burst. Motion uses monotonic elapsed time excluding pauses. MAVLink
`time_boot_ms` measures actual monotonic process uptime **including pauses** and
wraps modulo 2³². It is never a Unix timestamp. Packets already queued in the
network may arrive just after a pause command; none are generated while paused.
Suppressing heartbeat with `pause` is useful for testing GCS connection timeouts.

## Wire contract

| Message | Rate | Encoding |
| --- | --- | --- |
| `HEARTBEAT` | 1 Hz | QUADROTOR, GENERIC autopilot, armed base mode, ACTIVE state |
| `GLOBAL_POSITION_INT` | `--rate-hz` | degrees × 1e7; MSL/relative altitude in mm; N/E/D velocity in cm/s; heading in centidegrees [0, 35999] |
| `ATTITUDE` | `--rate-hz` | roll/pitch/yaw in radians; angular rates zero in rad/s |

Down velocity is positive down. Yaw is normalized to [−π, π); heading is the same
direction expressed in [0, 360) degrees. Integer fields use nearest-integer
rounding, except boot milliseconds which truncate. The MAVLink 2 packet marker
is `0xFD`; the heartbeat payload's `mavlink_version` field is **3**, which is
normal and does not indicate the packet framing version.

## Separate Mac receiver verification

In another Mac terminal, activate the virtual environment installed above and
run this receiver **before** starting the container. Bind to `0.0.0.0`, since a
loopback-only listener may not receive container traffic. Allow incoming UDP for
Python if the macOS firewall asks, and close any other receiver using port 14550.
This is a temporary verification command, not a shipped service.

```sh
python - <<'PY'
import collections
import time
from pymavlink.dialects.v20 import common
from pymavlink import mavutil

receiver = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
parser = common.MAVLink(None)
counts = collections.Counter()
first = {}
print('Listening for up to 30 seconds; start the container now.', flush=True)
started = None
deadline = time.monotonic() + 30
try:
    while time.monotonic() < deadline:
        data = receiver.recv()
        if not data:
            time.sleep(0.01)
            continue
        assert data[0] == 0xFD, 'Expected MAVLink 2 framing'
        for message in parser.parse_buffer(data) or []:
            now = time.monotonic()
            if started is None:
                started = now
                deadline = now + 10
            assert (message.get_srcSystem(), message.get_srcComponent()) == (1, 1)
            kind = message.get_type()
            counts[kind] += 1
            if kind not in first:
                first[kind] = message
                print(message)
    print('Counts during ~10 seconds of traffic:', dict(counts))
    assert set(counts) == {'HEARTBEAT', 'GLOBAL_POSITION_INT', 'ATTITUDE'}, counts
finally:
    receiver.close()
PY
```

With default settings, expect about 10 heartbeats, 50 position messages and 50
attitude messages in ten seconds. The first position is near latitude 32.0,
longitude 34.8, with MSL/relative altitude 100000/50000 mm and heading 9000.
Straight flight at default speed has `vx=0`, `vy=500`, `vz=0`, yaw ≈1.5708,
and increasing longitude. Hover coordinates stay fixed and velocities are zero.
UDP can drop packets; the sender does not acknowledge delivery.

## Tests

```sh
# In the activated virtual environment:
python -m unittest discover -s tests -v

# Run the same tests in the Python 3.12 image:
docker run --rm --entrypoint python \
  -v "$PWD/tests:/app/tests:ro" uav-sim:dev \
  -m unittest discover -s tests -v
```

Tests use only unittest and the production dependency. They cover deterministic
motion, integer conversions, MAVLink 2 encode/decode, source IDs, CLI validation,
scheduler rates/drift, pause/resume timing, actual localhost UDP delivery,
cardinal headings and dateline crossings over UDP, late/restarted receivers,
oversized command lines, stdin EOF, SIGINT and SIGTERM. Integration tests bind an
ephemeral localhost UDP port and take approximately eight seconds.
