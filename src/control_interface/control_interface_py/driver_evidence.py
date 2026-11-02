"""Read-only native-driver evidence codecs, no ROS or device access."""
import math
import re
import struct

CAPABILITY = 'driver_native_evidence_v1'
TRACE = ord('W')
HEADER = struct.Struct('<BIIIBBB')


def decode_trace(body):
    if len(body) < HEADER.size:
        raise ValueError('short driver evidence')
    version, sequence, time_us, dropped, axis, phase, size = HEADER.unpack_from(body)
    if version != 1 or axis >= 6 or phase > 3 or not 0 < size <= 24 or len(body) != HEADER.size+size:
        raise ValueError('invalid driver evidence')
    raw = body[HEADER.size:]
    return 'DRIVER_UART:'+('TX', 'REPLY', 'DISCARDED', 'PARTIAL')[phase]+':'+raw.hex(), \
        [float(x) for x in (sequence, time_us, dropped, axis, phase, size)]+[float(x) for x in raw]


def parse_state(text):
    fields = text.split(',')
    if len(fields) != 8 or fields[:2] != ['OK_DRIVER_STATE', 'V1']:
        raise ValueError('invalid native readback response')
    values = dict(field.split('=', 1) for field in fields[2:])
    if set(values) != {'A', 'ENC', 'T0', 'T1', 'DA', 'DS'} or values['A'] != '0':
        raise ValueError('invalid native readback fields')
    result = {'axis': 0, 'encoder_count': int(values['ENC']),
              'start_device_us': int(values['T0']), 'end_device_us': int(values['T1'])}
    for key, label, name in (('DA', 'Angle', 'driver_angle_deg'), ('DS', '(?:speed|vitesse)', 'driver_reported_rpm')):
        raw = bytes.fromhex(values[key]).decode('ascii')
        match = re.fullmatch(label+r':\s*([+-]?\d+(?:\.\d+)?)\r?', raw, re.IGNORECASE)
        if not match or not math.isfinite(float(match[1])):
            raise ValueError('unexpected '+key+' response: '+repr(raw))
        result[name] = float(match[1]); result[key+'_raw'] = raw
    return result
