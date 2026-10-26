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