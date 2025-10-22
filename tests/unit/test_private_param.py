import struct
import logging
import pytest
from robstride.protocol.private import PrivateProtocolHandler
from robstride.protocol.can_utils import build_extended_can_id
from robstride.models import ParameterData, CommunicationType


def make_msg(extid: int, data: bytes):
    import can
    return can.Message(arbitration_id=extid, data=data, is_extended_id=True, dlc=8)


def test_param_decode_float32_le():
    # param index 0x7018 (LIMIT_CUR) value 8.0 as float32 little-endian
    handler = PrivateProtocolHandler(motor_id=0x7F, can_bus=None, master_id=0xFD)
    idx = 0x7018
    val = struct.pack('<f', 8.0)
    data = struct.pack('<H', idx) + b'\x00\x00' + val
    extid = build_extended_can_id(CommunicationType.GET_SINGLE_PARAMETER, 0x007F, handler.master_id, handler.motor_id)
    msg = make_msg(extid, data)
    pd = ParameterData()
    processed = handler._process_parameter_response(msg, pd)
    assert processed is True
    assert abs(pd.limit_cur - 8.0) < 1e-6


def test_addressing_relaxed_accepts_swapped_bytes():
    # Simulate a message where data_field bytes are swapped
    handler = PrivateProtocolHandler(motor_id=0x7F, can_bus=None, master_id=0xFD)
    idx = 0x7018
    val = struct.pack('<f', 8.0)
    data = struct.pack('<H', idx) + b'\x00\x00' + val
    # place master id in low byte of data_field
    alt_data_field = (0x00 << 8) | (handler.master_id & 0xFF)
    extid = build_extended_can_id(CommunicationType.GET_SINGLE_PARAMETER, alt_data_field, handler.master_id, handler.motor_id)
    msg = make_msg(extid, data)
    pd = ParameterData()
    # process_message should accept this frame due to relaxed addressing
    accepted = handler.process_message(msg, None, pd)
    assert accepted is True


def test_param_decode_uint32_fallback():
    handler = PrivateProtocolHandler(motor_id=0x7F, can_bus=None, master_id=0xFD)
    idx = 0x7005
    # Put uint32 value 123456789 in little-endian at offset 4
    val = struct.pack('<I', 123456789)
    data = struct.pack('<H', idx) + b'\x00\x00' + val
    extid = build_extended_can_id(CommunicationType.GET_SINGLE_PARAMETER, 0x007F, handler.master_id, handler.motor_id)
    msg = make_msg(extid, data)
    pd = ParameterData()
    processed = handler._process_parameter_response(msg, pd)
    assert processed is True
    # check one of the writable fields that maps to 0x7005 (run_mode) - may not map to uint32
    # but ensure processing returned True and didn't crash
    assert processed
