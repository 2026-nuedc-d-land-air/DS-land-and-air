"""
PC-side V2.2 flight-controller simulator for this directory's main.py.

Usage:
  python fc_simulator.py --self-test
  python fc_simulator.py COM3

The simulator uses the current camera implementation's four-byte ACK and
12-byte Action/Result/ActionId/SourceTimeMs result payload.
"""

import struct
import sys
import threading
import time


ADDR_FC = 0x21
ADDR_CAMERA = 0x50
TYPE_ACK = 0x11
TYPE_CAMERA_MODE = 0x90
TYPE_CAMERA_TARGET = 0x91
TYPE_CAMERA_ACTION = 0x92
TYPE_CAMERA_ACTION_RESULT = 0x93
FLAG_ACK_REQUIRED = 0x01
ACTION_DROP = 0x01


def crc16_ccitt(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (((crc << 1) ^ 0x1021) if crc & 0x8000 else crc << 1) & 0xFFFF
    return crc


def encode_frame(msg_type, src, dst, seq, flags, payload=b''):
    if len(payload) > 64:
        raise ValueError('payload exceeds 64 bytes')
    body = struct.pack('>BBBBBBB', 0x02, msg_type, src, dst, seq, flags,
                       len(payload)) + payload
    return b'\xAA\x55' + body + struct.pack('>H', crc16_ccitt(body))


class V2Parser:
    def __init__(self, local_addr):
        self.local_addr = local_addr
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while True:
            frame = self._one()
            if frame is None:
                return frames
            frames.append(frame)

    def _one(self):
        while len(self.buffer) >= 2 and self.buffer[:2] != b'\xAA\x55':
            del self.buffer[0]
        if len(self.buffer) < 11:
            return None
        if self.buffer[2] != 0x02 or self.buffer[8] > 64:
            del self.buffer[0]
            return None
        frame_len = 11 + self.buffer[8]
        if len(self.buffer) < frame_len:
            return None
        body = bytes(self.buffer[2:frame_len - 2])
        crc_rx = struct.unpack('>H', self.buffer[frame_len - 2:frame_len])[0]
        if crc16_ccitt(body) != crc_rx:
            del self.buffer[0]
            return None
        frame = {
            'type': self.buffer[3],
            'src': self.buffer[4],
            'dst': self.buffer[5],
            'seq': self.buffer[6],
            'flags': self.buffer[7],
            'payload': bytes(self.buffer[9:frame_len - 2]),
        }
        del self.buffer[:frame_len]
        if frame['dst'] not in (self.local_addr, 0x10):
            return None
        return frame


class SerialLink:
    def __init__(self, port):
        self.port = port
        self.serial = None

    def open(self):
        try:
            import serial
        except ImportError as e:
            raise RuntimeError('pyserial is required: pip install pyserial') from e
        self.serial = serial.Serial(self.port, 115200, timeout=0.05)

    def read(self):
        return self.serial.read(256) if self.serial else b''

    def write(self, data):
        return self.serial.write(data)

    def close(self):
        if self.serial:
            self.serial.close()
            self.serial = None


class FCSimulator:
    def __init__(self, link):
        self.link = link
        self.parser = V2Parser(ADDR_FC)
        self.seq = 0
        self.running = False
        self.task_type = 0
        self.mission_id = 0
        self.mode_seq = 0
        self.target_rx_count = 0

    def next_seq(self):
        seq = self.seq
        self.seq = (self.seq + 1) & 0xFF
        return seq

    def start(self):
        self.link.open()
        self.running = True
        threading.Thread(target=self._receive_loop, daemon=True).start()
        print('[FC] connected at 115200 8N1')

    def stop(self):
        self.running = False
        self.link.close()

    def _send(self, frame, label):
        self.link.write(frame)
        print(f'[TX] {label}: {frame.hex(" ")}')

    def send_mode(self, mode, task_type, mission_id):
        seq = self.next_seq()
        self.task_type = task_type
        self.mission_id = mission_id
        self.mode_seq = seq
        payload = struct.pack('>BBHBB', mode, task_type, mission_id, 0, 0)
        self._send(encode_frame(TYPE_CAMERA_MODE, ADDR_FC, ADDR_CAMERA, seq,
                                FLAG_ACK_REQUIRED, payload),
                   f'MODE mode={mode} task={task_type} mission={mission_id} seq={seq}')

    def send_drop(self, action_id=None):
        if not self.mission_id:
            print('[ERR] send mode first')
            return
        if action_id is None:
            action_id = self.mission_id
        seq = self.next_seq()
        payload = struct.pack('>BBHBHB', ACTION_DROP, 0, action_id,
                              self.task_type, self.mission_id, self.mode_seq)
        self._send(encode_frame(TYPE_CAMERA_ACTION, ADDR_FC, ADDR_CAMERA, seq,
                                FLAG_ACK_REQUIRED, payload),
                   f'DROP action={ACTION_DROP} action_id={action_id} seq={seq}')

    def _receive_loop(self):
        while self.running:
            try:
                data = self.link.read()
                for frame in self.parser.feed(data):
                    self._handle_frame(frame)
            except Exception as e:
                if self.running:
                    print(f'[RX] error: {e}')
            time.sleep(0.005)

    def _handle_frame(self, frame):
        payload = frame['payload']
        if frame['type'] == TYPE_ACK and len(payload) >= 4:
            print(f'[RX] ACK request=0x{payload[0]:02X} seq={payload[1]} '
                  f'result={payload[2]} detail={payload[3]}')
        elif frame['type'] == TYPE_CAMERA_TARGET and len(payload) == 18:
            self.target_rx_count += 1
            if self.target_rx_count % 30 != 0:
                return
            flags, quality, err_x, err_y, diameter, count, source_ms, task, mission, mode = \
                struct.unpack('>BBhhHHIBHB', payload)
            if flags & 1:
                valid = 'locked' if flags & 4 else 'observed-unlocked'
            elif flags & 2:
                valid = 'predicted'
            else:
                valid = 'no-target'
            source = ' apriltag' if flags & 8 else ''
            print(f'[RX] TARGET #{self.target_rx_count} {valid}{source} '
                  f'q={quality} err=({err_x:+d},{err_y:+d}) '
                  f'dia={diameter} frame={count} task={task}/{mission}/{mode} t={source_ms}')
        elif frame['type'] == TYPE_CAMERA_ACTION_RESULT and len(payload) == 12:
            action, result, action_id, source_ms, task, mission, mode = \
                struct.unpack('>BBHIBHB', payload)
            print(f'[RX] RESULT action={action} result={result} action_id={action_id} '
                  f'task={task}/{mission}/{mode} t={source_ms}')
        else:
            print(f'[RX] type=0x{frame["type"]:02X} payload={payload.hex(" ")}')


def self_test():
    mode_payload = struct.pack('>BBHBB', 2, 1, 7, 0, 0)
    mode = encode_frame(TYPE_CAMERA_MODE, ADDR_FC, ADDR_CAMERA, 3,
                        FLAG_ACK_REQUIRED, mode_payload)
    camera_parser = V2Parser(ADDR_CAMERA)
    parsed = camera_parser.feed(mode)
    assert len(parsed) == 1 and parsed[0]['payload'] == mode_payload

    result_payload = struct.pack('>BBHIBHB', ACTION_DROP, 0, 7, 1234, 1, 7, 3)
    assert len(result_payload) == 12
    result = encode_frame(TYPE_CAMERA_ACTION_RESULT, ADDR_CAMERA, ADDR_FC,
                          4, 0, result_payload)
    fc_parser = V2Parser(ADDR_FC)
    parsed = fc_parser.feed(result)
    assert len(parsed) == 1 and parsed[0]['payload'] == result_payload
    print('[PASS] V2.2 current-main protocol self-test')


def interactive(simulator):
    print('Commands: mode <0-3> <task> <mission> | drop [action_id] | quit')
    while True:
        try:
            parts = input('fc> ').strip().split()
        except (EOFError, KeyboardInterrupt):
            break
        if not parts:
            continue
        try:
            if parts[0] == 'mode' and len(parts) == 4:
                simulator.send_mode(int(parts[1]), int(parts[2]), int(parts[3]))
            elif parts[0] == 'drop' and len(parts) in (1, 2):
                simulator.send_drop(int(parts[1]) if len(parts) == 2 else None)
            elif parts[0] in ('quit', 'exit', 'q'):
                break
            else:
                print('[ERR] invalid command')
        except ValueError:
            print('[ERR] numeric value required')


def main():
    if len(sys.argv) == 2 and sys.argv[1] == '--self-test':
        self_test()
        return
    if len(sys.argv) != 2:
        print(f'Usage: {sys.argv[0]} COM3 | --self-test')
        return
    simulator = FCSimulator(SerialLink(sys.argv[1]))
    try:
        simulator.start()
        interactive(simulator)
    finally:
        simulator.stop()


if __name__ == '__main__':
    main()
