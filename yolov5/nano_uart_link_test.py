#!/usr/bin/env python3
"""Jetson Nano <-> STM32 UART5 PING/ECHO/VISION link tester.

Python 3.6 compatible and standard-library only.
"""

from __future__ import print_function

import argparse
import csv
import glob
import os
import select
import signal
import sys
import termios
import time


DEFAULT_PATTERNS = (
    "/dev/serial/by-id/*",
    "/dev/ttyUSB*",
    "/dev/ttyACM*",
    "/dev/ttyTHS*",
)


def monotonic_ms():
    return int(time.monotonic() * 1000.0)


def discover_ports():
    ports = []
    for pattern in DEFAULT_PATTERNS:
        for path in sorted(glob.glob(pattern)):
            real = os.path.realpath(path)
            if not any(os.path.realpath(item) == real for item in ports):
                ports.append(path)
    return ports


def open_uart(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[3] = 0
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def write_all(fd, payload):
    offset = 0
    while offset < len(payload):
        try:
            written = os.write(fd, payload[offset:])
        except BlockingIOError:
            select.select([], [fd], [], 0.2)
            continue
        if written <= 0:
            raise IOError("UART write returned zero bytes")
        offset += written


def append_csv(writer, csv_file, direction, kind, seq, line, rtt_ms, error):
    writer.writerow([
        time.strftime("%Y-%m-%dT%H:%M:%S"),
        monotonic_ms(),
        direction,
        kind,
        seq,
        line,
        "" if rtt_ms is None else "%.3f" % rtt_ms,
        error,
    ])
    csv_file.flush()


def main():
    parser = argparse.ArgumentParser(description="STM32 UART5 link tester")
    parser.add_argument("--port", help="serial device; auto-detect if omitted")
    parser.add_argument("--interval", type=float, default=1.0,
                        help="PING interval in seconds (default: 1.0)")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="stop after N seconds; 0 runs until Ctrl-C")
    parser.add_argument("--log", default="nano_uart_link.csv",
                        help="CSV output path")
    parser.add_argument("--echo-every", type=int, default=5,
                        help="send an ECHO after every N PINGs; 0 disables")
    parser.add_argument("--vision-test", action="store_true",
                        help="also send one synthetic VISION line every 5 PINGs")
    parser.add_argument("--vision-rate", type=float, default=0.0,
                        help="synthetic VISION lines per second; 0 disables")
    parser.add_argument("--verbose-vision", action="store_true",
                        help="print every synthetic VISION TX/ACK line")
    parser.add_argument("--list", action="store_true",
                        help="list candidate ports and exit")
    args = parser.parse_args()

    ports = discover_ports()
    if args.list:
        for port in ports:
            print("%s -> %s" % (port, os.path.realpath(port)))
        return 0

    port = args.port
    if not port:
        external_ports = [item for item in ports
                          if not item.startswith("/dev/ttyTHS")]
        if not external_ports:
            print("No serial port found. Connect USB-TTL or pass --port /dev/ttyTHS*." ,
                  file=sys.stderr)
            return 2
        port = external_ports[0]

    print("Opening %s -> %s at 115200 8N1" % (port, os.path.realpath(port)))
    fd = open_uart(port)
    stop = [False]

    def request_stop(_signum, _frame):
        stop[0] = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    start = time.monotonic()
    next_ping = start
    next_vision = start
    seq = 0
    vision_seq = 0
    pending = {}
    vision_pending = {}
    rx_buffer = b""
    sent = 0
    pong = 0
    echo_ack = 0
    vision_ack = 0
    vision_sent = 0
    malformed = 0
    deferred_actions = []

    log_directory = os.path.dirname(os.path.abspath(args.log))
    if not os.path.isdir(log_directory):
        os.makedirs(log_directory)

    with open(args.log, "a", newline="") as csv_file:
        writer = csv.writer(csv_file)
        if csv_file.tell() == 0:
            writer.writerow([
                "wall_time", "monotonic_ms", "direction", "kind", "seq",
                "line", "rtt_ms", "error"
            ])
            csv_file.flush()

        try:
            while not stop[0]:
                now = time.monotonic()
                if args.duration > 0.0 and (now - start) >= args.duration:
                    break

                if now >= next_ping:
                    seq += 1
                    line = "PING,%d" % seq
                    write_all(fd, (line + "\n").encode("ascii"))
                    pending[str(seq)] = time.monotonic()
                    sent += 1
                    append_csv(writer, csv_file, "TX", "PING", seq, line, None, "")
                    print("TX %s" % line)

                    # Do not burst optional commands immediately after PING.
                    # STM32 replies synchronously and its UART has no RX FIFO;
                    # spacing these transport probes preserves the same payloads
                    # while avoiding an artificial software overrun.
                    if args.echo_every > 0 and seq % args.echo_every == 0:
                        deferred_actions.append(
                            (now + 0.10, "ECHO", seq,
                             "ECHO,nano-%d" % seq))

                    if args.vision_test and seq % 5 == 0:
                        vision_seq += 1
                        deferred_actions.append(
                            (now + 0.20, "VISION", vision_seq,
                             "VISION,%d,0,0.95,10,20,100,120" % vision_seq))

                    next_ping += max(args.interval, 0.05)
                    if next_ping < now:
                        next_ping = now + max(args.interval, 0.05)

                while deferred_actions and now >= deferred_actions[0][0]:
                    _due, action_kind, action_seq, action_line = \
                        deferred_actions.pop(0)
                    write_all(fd, (action_line + "\n").encode("ascii"))
                    if action_kind == "VISION":
                        vision_pending[str(action_seq)] = time.monotonic()
                        vision_sent += 1
                    append_csv(writer, csv_file, "TX", action_kind, action_seq,
                               action_line, None, "")
                    if action_kind != "VISION" or args.verbose_vision:
                        print("TX %s" % action_line)

                if args.vision_rate > 0.0 and now >= next_vision:
                    vision_seq += 1
                    vision_line = "VISION,%d,0,0.95,10,20,100,120" % vision_seq
                    write_all(fd, (vision_line + "\n").encode("ascii"))
                    vision_pending[str(vision_seq)] = time.monotonic()
                    vision_sent += 1
                    append_csv(writer, csv_file, "TX", "VISION", vision_seq,
                               vision_line, None, "")
                    if args.verbose_vision:
                        print("TX %s" % vision_line)
                    vision_interval = 1.0 / max(args.vision_rate, 0.1)
                    next_vision += vision_interval
                    if next_vision < now:
                        next_vision = now + vision_interval

                poll_timeout = 0.005 if args.vision_rate > 0.0 else 0.05
                ready, _, _ = select.select([fd], [], [], poll_timeout)
                if fd not in ready:
                    continue

                try:
                    chunk = os.read(fd, 4096)
                except BlockingIOError:
                    continue
                if not chunk:
                    continue
                rx_buffer += chunk

                while b"\n" in rx_buffer:
                    raw, rx_buffer = rx_buffer.split(b"\n", 1)
                    line = raw.rstrip(b"\r").decode("ascii", "replace")
                    fields = line.split(",")
                    kind = fields[0] if fields else ""
                    line_seq = fields[1] if len(fields) > 1 else ""
                    rtt_ms = None
                    error = ""

                    if kind == "PONG" and line_seq in pending:
                        rtt_ms = (time.monotonic() - pending.pop(line_seq)) * 1000.0
                        pong += 1
                    elif kind == "ECHO_ACK":
                        echo_ack += 1
                    elif kind == "VISION_ACK" and line_seq in vision_pending:
                        rtt_ms = ((time.monotonic() -
                                   vision_pending.pop(line_seq)) * 1000.0)
                        vision_ack += 1
                    elif kind in ("STM32_READY", "HEARTBEAT", "STATUS",
                                  "ERR", "FAULT"):
                        pass
                    else:
                        malformed += 1
                        error = "UNEXPECTED_LINE"

                    append_csv(writer, csv_file, "RX", kind, line_seq,
                               line, rtt_ms, error)
                    if kind == "VISION_ACK" and not args.verbose_vision:
                        if vision_ack % 30 == 0:
                            print("VISION progress sent=%d ack=%d" %
                                  (vision_sent, vision_ack))
                    elif rtt_ms is None:
                        print("RX %s" % line)
                    else:
                        print("RX %s rtt=%.3f ms" % (line, rtt_ms))
        finally:
            os.close(fd)

    success = (100.0 * pong / sent) if sent else 0.0
    vision_success = ((100.0 * vision_ack / vision_sent)
                      if vision_sent else 0.0)
    print("SUMMARY port=%s ping_sent=%d pong=%d ping_success=%.2f%% "
          "echo_ack=%d vision_sent=%d vision_ack=%d "
          "vision_success=%.2f%% unexpected=%d log=%s" %
          (port, sent, pong, success, echo_ack, vision_sent, vision_ack,
           vision_success, malformed, args.log))
    passed = sent > 0 and pong == sent
    if vision_sent > 0:
        passed = passed and vision_ack == vision_sent
    return 0 if passed else 3


if __name__ == "__main__":
    sys.exit(main())
