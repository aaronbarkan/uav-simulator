import io
import math
import contextlib
import signal
import select
import socket
import subprocess
import sys
import time
from pathlib import Path
from unittest.mock import Mock, patch
import unittest
from dataclasses import replace

from simulator import (
    EARTH_RADIUS_M, MavlinkSender, Simulation, Periodic, mavutil, parse_args, position_fields, state_at,
)


class StateTests(unittest.TestCase):
    def test_hover_is_fixed(self):
        args = parse_args([])
        self.assertEqual(state_at(args, 0), state_at(args, 3600))
        state = state_at(args, 0)
        self.assertEqual((state.north_mps, state.east_mps, state.down_mps), (0, 0, 0))

    def test_straight_cardinal_headings(self):
        for heading, north, east in [(0, 5, 0), (90, 0, 5), (180, -5, 0), (270, 0, -5)]:
            args = parse_args(['--scenario', 'straight', '--heading-deg', str(heading)])
            state = state_at(args, 10)
            self.assertAlmostEqual(state.north_mps, north)
            self.assertAlmostEqual(state.east_mps, east)
            self.assertAlmostEqual(math.radians(state.latitude - args.lat) * EARTH_RADIUS_M, north * 10)
            self.assertAlmostEqual(math.radians(state.longitude - args.lon) * EARTH_RADIUS_M
                                   * math.cos(math.radians(args.lat)), east * 10)
            self.assertAlmostEqual(math.degrees(state.yaw) % 360, heading)
            self.assertEqual((state.roll, state.pitch), (0, 0))

    def test_dateline_wrap_and_polar_limit(self):
        args = parse_args(['--scenario', 'straight', '--lon', '180'])
        self.assertTrue(-180 <= state_at(args, 10).longitude < -179)
        args = parse_args(['--scenario', 'straight', '--lat', '89', '--heading-deg', '0'])
        with self.assertRaises(ValueError):
            state_at(args, 10)

    def test_integer_units_and_down_sign(self):
        state = replace(state_at(parse_args([]), 0), down_mps=1.25, north_mps=-2.5)
        self.assertEqual(position_fields(state, 1.25),
                         (1250, 320000000, 348000000, 100000, 50000, -250, 0, 125, 9000))
        self.assertEqual(position_fields(state, (2**32 + 42) / 1000)[0], 42)
        state = replace(state, yaw=math.radians(359.9999))
        self.assertEqual(position_fields(state, 0)[-1], 0)


class WireTests(unittest.TestCase):
    def setUp(self):
        self.receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(self.receiver.close)
        self.receiver.bind(('127.0.0.1', 0))
        self.receiver.settimeout(1)
        self.sender = MavlinkSender('127.0.0.1', self.receiver.getsockname()[1])
        self.addCleanup(self.sender.close)

    def receive(self, count):
        # Real sender/socket and strict independent parser (including CRC checks).
        parser = mavutil.mavlink.MAVLink(None)
        messages = []
        for _ in range(count):
            packet = self.receiver.recv(2048)
            self.assertEqual(packet[0], 0xFD)
            decoded = parser.parse_buffer(packet)
            self.assertEqual(len(decoded), 1)
            message = decoded[0]
            self.assertEqual((message.get_srcSystem(), message.get_srcComponent()), (1, 1))
            messages.append(message)
        return messages

    def test_all_messages_round_trip(self):
        self.sender.heartbeat()
        self.sender.telemetry(state_at(parse_args([]), 0), 1.25)
        messages = self.receive(3)
        self.assertEqual([m.get_msgId() for m in messages], [0, 33, 30])
        self.assertEqual([m.get_type() for m in messages],
                         ['HEARTBEAT', 'GLOBAL_POSITION_INT', 'ATTITUDE'])
        for message in messages:
            self.assertEqual(message.get_msgbuf()[0], 0xFD)
            self.assertEqual((message.get_srcSystem(), message.get_srcComponent()), (1, 1))
        heartbeat, position, attitude = messages
        self.assertEqual(heartbeat.type, mavutil.mavlink.MAV_TYPE_QUADROTOR)
        self.assertEqual(heartbeat.autopilot, mavutil.mavlink.MAV_AUTOPILOT_GENERIC)
        self.assertEqual(heartbeat.base_mode, mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        self.assertEqual(heartbeat.system_status, mavutil.mavlink.MAV_STATE_ACTIVE)
        self.assertEqual((position.lat, position.lon, position.alt, position.relative_alt),
                         (320000000, 348000000, 100000, 50000))
        self.assertEqual((position.time_boot_ms, attitude.time_boot_ms), (1250, 1250))
        self.assertEqual((position.vx, position.vy, position.vz), (0, 0, 0))
        self.assertAlmostEqual(math.degrees(attitude.yaw), position.hdg / 100, places=4)
        self.assertEqual((attitude.rollspeed, attitude.pitchspeed, attitude.yawspeed), (0, 0, 0))

    def test_cardinal_headings_and_motion_over_udp(self):
        for heading, north, east in [(0, 5, 0), (90, 0, 5), (180, -5, 0), (270, 0, -5)]:
            with self.subTest(heading=heading):
                args = parse_args(['--scenario', 'straight', '--heading-deg', str(heading)])
                self.sender.telemetry(state_at(args, 10), 10)
                position, attitude = self.receive(2)
                self.assertEqual((position.vx, position.vy, position.vz),
                                 (north * 100, east * 100, 0))
                self.assertEqual(position.hdg, heading * 100)
                difference = (math.degrees(attitude.yaw) - position.hdg / 100 + 180) % 360 - 180
                self.assertAlmostEqual(difference, 0, places=4)
                self.assertAlmostEqual(math.radians(position.lat / 1e7 - args.lat)
                                       * EARTH_RADIUS_M, north * 10, delta=0.01)
                self.assertAlmostEqual(math.radians(position.lon / 1e7 - args.lon)
                                       * EARTH_RADIUS_M * math.cos(math.radians(args.lat)),
                                       east * 10, delta=0.01)

    def test_nonzero_fields_and_uint32_wrap_over_udp(self):
        state = replace(state_at(parse_args([]), 0), latitude=-12.3456789,
                        longitude=-45.1234567, alt_msl_m=-12.345, relative_alt_m=67.891,
                        north_mps=-1.23, east_mps=4.56, down_mps=7.89,
                        roll=0.25, pitch=-0.5, yaw=math.radians(359.9999))
        self.sender.telemetry(state, (2**32 + 42) / 1000)
        position, attitude = self.receive(2)
        self.assertEqual((position.lat, position.lon, position.alt, position.relative_alt),
                         (-123456789, -451234567, -12345, 67891))
        self.assertEqual((position.vx, position.vy, position.vz), (-123, 456, 789))
        self.assertEqual((position.time_boot_ms, attitude.time_boot_ms), (42, 42))
        self.assertEqual(position.hdg, 0)
        self.assertAlmostEqual(attitude.roll, 0.25)
        self.assertAlmostEqual(attitude.pitch, -0.5)
        self.assertAlmostEqual(attitude.yaw, state.yaw, places=5)

    def test_hover_and_both_dateline_crossings_over_udp(self):
        args = parse_args([])
        positions = []
        for elapsed in (0, 100):
            self.sender.telemetry(state_at(args, elapsed), elapsed)
            position, _ = self.receive(2)
            positions.append(position)
            self.assertEqual((position.vx, position.vy, position.vz), (0, 0, 0))
        self.assertEqual((positions[0].lat, positions[0].lon, positions[0].alt),
                         (positions[1].lat, positions[1].lon, positions[1].alt))
        for heading, longitude, expected_sign in [(90, 180, -1), (270, -180, 1)]:
            args = parse_args(['--scenario', 'straight', '--lon', str(longitude),
                               '--heading-deg', str(heading)])
            self.sender.telemetry(state_at(args, 10), 10)
            position, _ = self.receive(2)
            self.assertTrue(1799990000 < position.lon * expected_sign < 1800000000)

    def test_pause_preserves_position_and_uptime_over_udp(self):
        sim = Simulation(parse_args(['--scenario', 'straight']), self.sender, 100)
        sim.tick(102)
        _, before, _ = self.receive(3)
        with contextlib.redirect_stdout(io.StringIO()):
            sim.command('pause', 102)
            sim.tick(112)
            self.receiver.settimeout(0.05)
            with self.assertRaises(socket.timeout):
                self.receiver.recv(2048)
            sim.command('resume', 112)
        sim.tick(112)
        _, after, _ = self.receive(3)
        self.assertEqual((before.lat, before.lon), (after.lat, after.lon))
        self.assertEqual(after.time_boot_ms - before.time_boot_ms, 10000)



class SchedulerTests(unittest.TestCase):
    def test_rates_and_no_catchup_burst(self):
        sender = Mock()
        sim = Simulation(parse_args([]), sender, 100)
        for step in range(10000):
            sim.tick(100 + step / 1000)
        self.assertEqual(sender.heartbeat.call_count, 10)
        self.assertEqual(sender.telemetry.call_count, 50)
        sim.tick(200)
        self.assertEqual(sender.heartbeat.call_count, 11)
        self.assertEqual(sender.telemetry.call_count, 51)
        sim.tick(200)
        self.assertEqual(sender.telemetry.call_count, 51)

    def test_heartbeat_does_not_calculate_state(self):
        sender = Mock()
        sim = Simulation(parse_args(['--rate-hz', '0.1']), sender, 0)
        sim.tick(0)
        with patch('simulator.state_at', side_effect=AssertionError('unnecessary calculation')):
            sim.tick(0.5)
            sim.tick(1)
        self.assertEqual(sender.heartbeat.call_count, 2)
        self.assertEqual(sender.telemetry.call_count, 1)

    def test_fractional_rate_deadlines_do_not_drift(self):
        timer = Periodic(1 / 7, 0)
        for index in range(70):
            self.assertTrue(timer.due(index / 7 + 0.001))
        self.assertAlmostEqual(timer.deadline, 10)

    def test_pause_freezes_motion_but_not_boot_uptime(self):
        sender = Mock()
        sim = Simulation(parse_args(['--scenario', 'straight']), sender, 100)
        with contextlib.redirect_stdout(io.StringIO()):
            sim.tick(100)
            sim.command('pause', 102)
            sim.command('pause', 103)
            for now in (103, 110, 120):
                sim.tick(now)
                self.assertEqual(sim.elapsed(now), 2)
            self.assertEqual(sender.heartbeat.call_count, 1)
            self.assertEqual(sender.telemetry.call_count, 1)
            sim.command('resume', 122)
            sim.tick(122)
            state, uptime = sender.telemetry.call_args.args
            self.assertEqual(state, state_at(sim.args, 2))
            self.assertEqual(uptime, 22)
            sim.command('resume', 123)
            self.assertEqual(sim.elapsed(123), 3)
            sim.command('pause', 124)
            sim.command('resume', 134)
            self.assertEqual(sim.elapsed(135), 5)
            self.assertFalse(sim.command('quit', 135))


class CliTests(unittest.TestCase):
    def test_invalid_bounds(self):
        for option, value in [('--port', '0'), ('--port', '65536'), ('--rate-hz', '0'),
                              ('--rate-hz', 'nan'), ('--speed-mps', 'inf'),
                              ('--speed-mps', '328'), ('--heading-deg', '-1'),
                              ('--heading-deg', '361'), ('--lat', '90'),
                              ('--lon', '181'), ('--alt-msl-m', '2147484'),
                              ('--relative-alt-m', '-2147484'), ('--host', ''),
                              ('--host', 'host:123')]:
            with self.subTest(option=option, value=value), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    parse_args([option, value])
                self.assertEqual(error.exception.code, 2)

    def test_heading_360_normalizes(self):
        self.assertEqual(parse_args(['--heading-deg', '360']).heading_deg, 0)


class UdpIntegrationTests(unittest.TestCase):
    def start_sender(self, receiver, stdin=subprocess.PIPE):
        process = subprocess.Popen(
            [sys.executable, str(Path(__file__).resolve().parents[1] / 'simulator.py'),
             '--host', '127.0.0.1', '--port', str(receiver.getsockname()[1]),
             '--scenario', 'straight', '--rate-hz', '10'],
            stdin=stdin, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        )
        self.addCleanup(self.stop_sender, process)
        return process

    @staticmethod
    def stop_sender(process):
        if process.poll() is None:
            process.kill()
        process.communicate(timeout=3)

    def receive_for(self, receiver, duration):
        messages = []
        end = time.monotonic() + duration
        parser = mavutil.mavlink.MAVLink(None)
        while time.monotonic() < end:
            receiver.settimeout(max(0.001, end - time.monotonic()))
            try:
                data = receiver.recv(2048)
            except socket.timeout:
                break
            self.assertEqual(data[0], 0xFD)
            message = parser.parse_char(data)
            self.assertIsNotNone(message)
            self.assertEqual((message.get_srcSystem(), message.get_srcComponent()), (1, 1))
            self.assertIn(message.get_type(), ('HEARTBEAT', 'GLOBAL_POSITION_INT', 'ATTITUDE'))
            messages.append(message)
        return messages

    def test_udp_rates_pause_resume_and_quit(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(('127.0.0.1', 0))
            process = self.start_sender(receiver)
            messages = self.receive_for(receiver, 2.4)
            positions = [m for m in messages if m.get_type() == 'GLOBAL_POSITION_INT']
            self.assertTrue(18 <= len(positions) <= 25, len(positions))
            self.assertEqual(len(positions), sum(m.get_type() == 'ATTITUDE' for m in messages))
            self.assertTrue(2 <= sum(m.get_type() == 'HEARTBEAT' for m in messages) <= 3)
            self.assertGreater(positions[-1].lon, positions[0].lon)
            self.assertEqual((positions[-1].vx, positions[-1].vy, positions[-1].vz), (0, 500, 0))
            process.stdin.write(b'pause\nstatus\n')
            process.stdin.flush()
            self.receive_for(receiver, 0.2)  # Drain packets sent before the command was read.
            self.assertEqual(self.receive_for(receiver, 0.4), [])
            process.stdin.write(b'resume\n')
            process.stdin.flush()
            resumed = self.receive_for(receiver, 0.25)
            first = next(m for m in resumed if m.get_type() == 'GLOBAL_POSITION_INT')
            # Boot time advances across pause, position advances only while active.
            self.assertGreater(first.time_boot_ms - positions[-1].time_boot_ms, 500)
            self.assertLess(first.lon - positions[-1].lon, 100)
            process.stdin.write(b'help\nquit\n')
            process.stdin.flush()
            stdout, stderr = process.communicate(timeout=3)
            self.assertEqual(process.returncode, 0, stderr)
            self.assertIn(b'paused straight', stdout)
            self.assertIn(b'Commands:', stdout)
            self.assertIn(b'Stopped', stdout)

    def test_late_and_restarted_receiver(self):
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(('127.0.0.1', 0))
        address = receiver.getsockname()
        process = self.start_sender(receiver)
        receiver.close()
        # Synchronize with startup; sender is running with no listener.
        self.assertTrue(select.select([process.stdout], [], [], 3)[0])
        self.assertIn(b'Sending', process.stdout.readline())
        time.sleep(0.15)
        for _ in range(2):
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as listener:
                listener.bind(address)
                messages = self.receive_for(listener, 1.2)
                self.assertEqual({m.get_type() for m in messages},
                                 {'HEARTBEAT', 'GLOBAL_POSITION_INT', 'ATTITUDE'})
            time.sleep(0.15)
        self.assertIsNone(process.poll())

    def test_overlong_line_tail_is_not_a_command(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(('127.0.0.1', 0))
            process = self.start_sender(receiver)
            self.assertTrue(select.select([process.stdout], [], [], 3)[0])
            self.assertIn(b'Sending', process.stdout.readline())
            process.stdin.write(b'x' * 8192)
            process.stdin.flush()
            self.assertTrue(select.select([process.stdout], [], [], 3)[0])
            self.assertIn(b'Command too long', process.stdout.readline())
            # A command-looking suffix in a later read must be discarded too.
            stdout, stderr = process.communicate(b'quit\nstatus\nquit\n', timeout=3)
            self.assertEqual(process.returncode, 0, stderr)
            self.assertIn(b'running straight', stdout)
            self.assertIn(b'heading=90deg speed=5m/s', stdout)
            self.assertNotIn(b'Unknown command', stdout)

    def test_complete_overlong_line_is_not_executed(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(('127.0.0.1', 0))
            process = self.start_sender(receiver)
            # Includes the newline in the read that first exceeds the limit.
            stdout, stderr = process.communicate(b' ' * 4096 + b'quit\nstatus\nquit\n', timeout=3)
            self.assertEqual(process.returncode, 0, stderr)
            self.assertIn(b'Command too long', stdout)
            self.assertIn(b'running straight', stdout)

    def test_eof_and_signals(self):
        for stop_signal in (signal.SIGINT, signal.SIGTERM):
            with self.subTest(signal=stop_signal), socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
                receiver.bind(('127.0.0.1', 0))
                process = self.start_sender(receiver, stdin=subprocess.DEVNULL)
                self.assertGreaterEqual(len(self.receive_for(receiver, 0.4)), 3)
                process.send_signal(stop_signal)
                stdout, stderr = process.communicate(timeout=3)
                self.assertEqual(process.returncode, 0, stderr)
                self.assertIn(b'Stopped', stdout)


if __name__ == '__main__':
    unittest.main()
