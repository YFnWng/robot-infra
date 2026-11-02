"""Explicit C ABI binding for simulation; never opens a device or compiles code."""
import ctypes as c


class Config(c.Structure):
    _fields_ = [("lower", c.c_int32*6), ("upper", c.c_int32*6),
                ("step", c.c_uint32*6), ("tolerance", c.c_uint32*6),
                ("minimum_rpm", c.c_uint16*6), ("maximum_rpm", c.c_uint16*6),
                ("native_position", c.c_uint8)]


class Update(c.Structure):
    _fields_ = [("command_id", c.c_uint32), ("rpm", c.c_int16*6),
                ("residual", c.c_int64*6), ("status", c.c_uint8),
                ("stop_mask", c.c_uint8), ("reversal_mask", c.c_uint8),
                ("overshoot_mask", c.c_uint8), ("native_start_mask", c.c_uint8),
                ("native_deci_degrees", c.c_uint32*6)]


class NativePositionSession:
    def __init__(self, library, config):
        self.lib = c.CDLL(str(library))
        try:
            if self.lib.position_abi_version() != 2:
                raise RuntimeError('position native ABI mismatch; rebuild control_interface')
        except AttributeError as error:
            raise RuntimeError('old position native library; rebuild control_interface') from error
        signatures = {
            "position_create": ([c.POINTER(Config)], c.c_void_p),
            "position_destroy": ([c.c_void_p], None),
            "position_open": ([c.c_void_p, c.c_bool, c.c_uint32], c.c_uint32),
            "position_cancel": ([c.c_void_p], None),
            "position_native_started": ([c.c_void_p, c.c_uint8], None),
            "position_submit": ([c.c_void_p, c.c_void_p, c.c_size_t, c.c_uint32,
                                  c.POINTER(c.c_int32), c.c_bool, c.c_uint8], c.c_int),
            "position_update": ([c.c_void_p, c.c_uint32, c.POINTER(c.c_int32),
                                  c.POINTER(c.c_int16), c.c_bool, c.c_uint8, c.POINTER(Update)], None),
        }
        for name, (args, result) in signatures.items():
            function = getattr(self.lib, name)
            function.argtypes, function.restype = args, result
        self.handle = self.lib.position_create(c.byref(config))
        if not self.handle:
            raise RuntimeError("position native session allocation failed")

    def close(self):
        if self.handle:
            self.lib.position_destroy(self.handle)
            self.handle = None

    def open(self, stopped, nonce=0):
        return self.lib.position_open(self.handle, stopped, nonce)

    def cancel(self):
        self.lib.position_cancel(self.handle)

    def native_started(self, mask):
        self.lib.position_native_started(self.handle, mask)

    def submit(self, envelope, time_us, counts, fresh=True, allowed=0x3f):
        payload = envelope.encode_payload()
        return self.lib.position_submit(self.handle, payload, len(payload), time_us,
                                         (c.c_int32*6)(*counts), fresh, allowed)

    def update(self, time_us, counts, applied, fresh=True, allowed=0x3f):
        result = Update()
        self.lib.position_update(self.handle, time_us, (c.c_int32*6)(*counts),
                                  (c.c_int16*6)(*applied), fresh, allowed, c.byref(result))
        return result
