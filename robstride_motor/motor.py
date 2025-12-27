"""RobStrideモーター制御の実装."""

import struct
import time
from typing import Optional

import can
from can.typechecking import CanFilter

from robstride_motor.bus import create_can_bus
from robstride_motor.types import (
    ACTUATOR_OPERATION_MAPPING,
    ActuatorType,
    CommunicationType,
    ControlMode,
    FirmwareInfo,
    MotorFeedback,
    ParameterIndex,
)

# CAN拡張フレームフラグ
CAN_EFF_FLAG = 0x80000000


class RobStrideMotor:
    """CANインターフェース経由のRobStride BLDCモーターコントローラー."""

    def __init__(
        self,
        can_interface: str,
        master_id: int,
        motor_id: int,
        actuator_type: ActuatorType,
        bus: Optional[can.BusABC] = None,
        bitrate: int = 1000000,
    ) -> None:
        """モーターコントローラーを初期化する.

        Args:
            can_interface: CANインターフェースバックエンド。以下のいずれかを指定:
                          - 'gs_usb': USB CANアダプタ（例: candleLight）
                          - 'socketcan': SocketCANインターフェース（Linuxのみ）
                          - 特定のチャンネル名（例: 'can0'）: レガシーSocketCANモード
            master_id: マスターデバイスID（通常は0xFF）
            motor_id: モーターデバイスID
            actuator_type: パラメータマッピング用のアクチュエータタイプ
            bus: 事前に初期化されたCANバスオブジェクト。指定された場合、
                 他のバス初期化パラメータは無視されます。
            bitrate: CANビットレート（bps）（デフォルト: 1000000）。
        """
        self.can_interface = can_interface
        self.master_id = master_id
        self.motor_id = motor_id
        self.actuator_type = actuator_type
        self.bitrate = bitrate

        # モーター状態
        self.position: float = 0.0
        self.velocity: float = 0.0
        self.torque: float = 0.0
        self.temperature: float = 0.0
        self.error_code: int = 0
        self.pattern: int = 0
        self.current_mode: int = 0

        # CANバスを初期化または受け入れる
        if bus is not None:
            self.bus = bus
            self._owns_bus = False
        else:
            self._init_bus()
            self._owns_bus = True

    def _init_bus(self) -> None:
        """CANバスインターフェースを初期化する.

        can_interface値に基づく3つのモードをサポート:
        - 'gs_usb': gs_usbバックエンド経由のUSB CANアダプタ
        - 'socketcan': SocketCANインターフェース（Linux）
        - その他の値: レガシーSocketCANモード用のチャンネル名として扱われる
        """
        if self.can_interface in ("gs_usb", "socketcan"):
            # 明示的なインターフェースでcreate_can_busを使用
            self.bus = create_can_bus(
                interface=self.can_interface,  # type: ignore[arg-type]
                bitrate=self.bitrate,
            )
        else:
            # レガシーモード: can_interfaceをSocketCANのチャンネル名として扱う
            self.bus = can.interface.Bus(
                channel=self.can_interface,
                interface="socketcan",
                receive_own_messages=False,
            )

        # このモーターからのメッセージのみを受信するフィルタを設定
        # CAN_EFF_FLAG = 0x80000000 は拡張フレーム用
        filters: list[CanFilter] = [
            {
                "can_id": (self.motor_id << 8) | CAN_EFF_FLAG,
                "can_mask": 0xFF00 | CAN_EFF_FLAG,
                "extended": True,
            }
        ]
        self.bus.set_filters(filters)

    def __del__(self) -> None:
        """このインスタンスが所有するCANバスをクリーンアップする."""
        if hasattr(self, "bus") and hasattr(self, "_owns_bus") and self._owns_bus:
            self.bus.shutdown()

    def _float_to_uint(self, x: float, x_min: float, x_max: float, bits: int) -> int:
        """浮動小数点数を符号なし整数に変換する.

        Args:
            x: 変換する値
            x_min: 最小値
            x_max: 最大値
            bits: エンコード用のビット数

        Returns:
            エンコードされた符号なし整数値
        """
        x = max(x_min, min(x_max, x))
        span = x_max - x_min
        offset = x - x_min
        return int((offset * ((1 << bits) - 1)) / span)

    def _uint_to_float(self, x_int: int, x_min: float, x_max: float, bits: int) -> float:
        """符号なし整数を浮動小数点数に変換する.

        Args:
            x_int: 変換する整数値
            x_min: 最小値
            x_max: 最大値
            bits: デコード用のビット数

        Returns:
            デコードされた浮動小数点値
        """
        span = x_max - x_min
        return float(x_int) * span / ((1 << bits) - 1) + x_min

    def _bytes_to_float(self, data: bytes) -> float:
        """4バイトを浮動小数点数に変換する（バイト4-7からリトルエンディアン）.

        C++実装と一致:
            uint32_t data = bytedata[7]<<24|bytedata[6]<<16|bytedata[5]<<8|bytedata[4];

        Args:
            data: バイトデータ（最低8バイト）

        Returns:
            浮動小数点値
        """
        if len(data) < 8:
            return 0.0
        # バイト4-7にリトルエンディアン形式でfloatが格納（C++ Byte_to_floatと一致）
        result: float = struct.unpack("<f", data[4:8])[0]
        return result

    def _send_frame(self, communication_type: int, extra_data: int, data: bytes) -> None:
        """モーターにCANフレームを送信する.

        Args:
            communication_type: 通信タイプ（5ビット）
            extra_data: 追加データフィールド（16ビット）
            data: ペイロードデータ（8バイト）
        """
        # 29ビット拡張CAN IDを構築
        can_id = (
            (communication_type & 0x1F) << 24 | (extra_data & 0xFFFF) << 8 | (self.motor_id & 0xFF)
        )

        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=True,
        )
        self.bus.send(msg)

    def _receive_frame(self, timeout: float = 0.0) -> tuple[int, int, int, bytes] | None:
        """モーターからCANフレームを受信する.

        Args:
            timeout: 受信タイムアウト（秒）（0 = ノンブロッキング）

        Returns:
            (communication_type, extra_data, host_id, data)のタプル、またはNone
        """
        msg = self.bus.recv(timeout=timeout if timeout > 0 else None)

        if msg is None:
            return None

        if not msg.is_extended_id:
            raise RuntimeError("Frame is not extended ID")

        can_id = msg.arbitration_id

        communication_type = (can_id >> 24) & 0x1F
        extra_data = (can_id >> 8) & 0xFFFF
        host_id = can_id & 0xFF

        self.error_code = (can_id >> 16) & 0x3F
        self.pattern = (can_id >> 22) & 0x03

        return communication_type, extra_data, host_id, bytes(msg.data)

    def _receive_status_frame(self, timeout: float = 1.0) -> tuple[int, float] | None:
        """モーター状態フレームを受信して解析する.

        Args:
            timeout: 受信タイムアウト（秒）

        Returns:
            パラメータ応答の場合: (index, value)のタプル
            その他のフレームの場合: None
        """
        result = self._receive_frame(timeout=timeout)
        if result is None:
            raise RuntimeError("No frame received")

        communication_type, extra_data, _, data = result

        if communication_type == CommunicationType.MOTOR_REQUEST:
            # フィードバックデータを解析（ビッグエンディアン）
            if len(data) < 8:
                raise RuntimeError("Data size too small")

            position_u16 = (data[0] << 8) | data[1]
            velocity_u16 = (data[2] << 8) | data[3]
            torque_u16 = (data[4] << 8) | data[5]
            temperature_u16 = (data[6] << 8) | data[7]

            op_params = ACTUATOR_OPERATION_MAPPING[self.actuator_type]

            self.position = (position_u16 / 32767.0 - 1.0) * op_params.position
            self.velocity = (velocity_u16 / 32767.0 - 1.0) * op_params.velocity
            self.torque = (torque_u16 / 32767.0 - 1.0) * op_params.torque
            self.temperature = temperature_u16 * 0.1
            return None

        elif communication_type == CommunicationType.GET_SINGLE_PARAMETER:
            # パラメータ応答を解析
            index = (data[1] << 8) | data[0]
            if index == ParameterIndex.RUN_MODE:
                value = float(data[4])
                self.current_mode = data[4]
            else:
                value = self._bytes_to_float(data)
            return (index, value)

        return None

    def enable_motor(self) -> MotorFeedback:
        """モーターを有効化する.

        Returns:
            モーターフィードバックデータ
        """
        data = bytes([0] * 8)
        self._send_frame(CommunicationType.MOTOR_ENABLE, self.master_id, data)
        time.sleep(0.001)
        self._receive_status_frame()
        
        # 現在のrun_modeを取得してcurrent_modeを更新
        self.get_parameter(ParameterIndex.RUN_MODE)
        time.sleep(0.001)
        
        return MotorFeedback(self.position, self.velocity, self.torque, self.temperature)

    def disable_motor(self, clear_error: bool = False) -> None:
        """モーターを停止する.

        Args:
            clear_error: エラーフラグをクリアするかどうか
        """
        data = bytes([1 if clear_error else 0] + [0] * 7)
        self._send_frame(CommunicationType.MOTOR_STOP, self.master_id, data)
        time.sleep(0.001)
        self._receive_status_frame()

    def set_parameter(self, index: int, value: float, is_mode: bool = False) -> None:
        """単一パラメータを設定する.

        Args:
            index: パラメータインデックス
            value: パラメータ値
            is_mode: Trueの場合、モード設定として扱う（uint8）
        """
        data_bytes = bytearray(8)
        data_bytes[0] = index & 0xFF
        data_bytes[1] = (index >> 8) & 0xFF
        data_bytes[2] = 0
        data_bytes[3] = 0

        if is_mode:
            data_bytes[4] = int(value) & 0xFF
        else:
            data_bytes[4:8] = struct.pack("<f", value)

        self._send_frame(CommunicationType.SET_SINGLE_PARAMETER, self.master_id, bytes(data_bytes))
        time.sleep(0.001)
        self._receive_status_frame()

    def get_parameter(self, index: int) -> float | None:
        """単一パラメータを取得する.

        Args:
            index: パラメータインデックス

        Returns:
            パラメータ値、または受信できなかった場合はNone
        """
        data_bytes = bytearray(8)
        data_bytes[0] = index & 0xFF
        data_bytes[1] = (index >> 8) & 0xFF

        self._send_frame(CommunicationType.GET_SINGLE_PARAMETER, self.master_id, bytes(data_bytes))
        time.sleep(0.001)
        result = self._receive_status_frame()
        if result is not None:
            _, value = result
            return value
        return None

    def _switch_mode(self, target_mode: ControlMode, auto_enable: bool = True) -> None:
        """必要に応じて制御モードを切り替える.

        Args:
            target_mode: ターゲット制御モード
            auto_enable: Trueの場合、モード切り替え後にモーターを有効化する
        """
        # 必要に応じてモードをチェックして切り替える
        if self.current_mode != target_mode:
            self.disable_motor(clear_error=False)
            time.sleep(0.001)
            self.set_parameter(ParameterIndex.RUN_MODE, float(target_mode), is_mode=True)
            time.sleep(0.001)
            self.get_parameter(ParameterIndex.RUN_MODE)
            time.sleep(0.001)
            if auto_enable:
                self.enable_motor()
                time.sleep(0.001)

    def send_motion_command(
        self,
        torque: float,
        position: float,
        velocity: float,
        kp: float = 0.5,
        kd: float = 0.1,
    ) -> MotorFeedback:
        """モーション制御コマンドを送信する（モード0）.

        Args:
            torque: 目標トルク (Nm)
            position: 目標位置 (rad)
            velocity: 目標速度 (rad/s)
            kp: 位置比例ゲイン
            kd: 位置微分ゲイン

        Returns:
            モーターフィードバックデータ
        """
        # _switch_modeは既にpattern == 2をチェック済み（C++実装と一致）
        self._switch_mode(ControlMode.MOTION_CONTROL, auto_enable=True)

        op_params = ACTUATOR_OPERATION_MAPPING[self.actuator_type]

        torque_uint = self._float_to_uint(torque, -op_params.torque, op_params.torque, 16)
        pos_uint = self._float_to_uint(position, -op_params.position, op_params.position, 16)
        vel_uint = self._float_to_uint(velocity, -op_params.velocity, op_params.velocity, 16)
        kp_uint = self._float_to_uint(kp, 0.0, op_params.kp, 16)
        kd_uint = self._float_to_uint(kd, 0.0, op_params.kd, 16)

        data = bytes(
            [
                (pos_uint >> 8) & 0xFF,
                pos_uint & 0xFF,
                (vel_uint >> 8) & 0xFF,
                vel_uint & 0xFF,
                (kp_uint >> 8) & 0xFF,
                kp_uint & 0xFF,
                (kd_uint >> 8) & 0xFF,
                kd_uint & 0xFF,
            ]
        )

        self._send_frame(CommunicationType.MOTION_CONTROL, torque_uint, data)
        self._receive_status_frame()
        return MotorFeedback(self.position, self.velocity, self.torque, self.temperature)

    def send_velocity_command(
        self, velocity: float, limit_cur: float = 23.0, acceleration: float = 20.0
    ) -> MotorFeedback:
        """速度制御コマンドを送信する（モード2）.

        公式仕様によれば、速度モードには以下が必要:
        - Setting run_mode to 2 (velocity mode)
        - Setting limit_cur (0x7018) for current limit
        - Setting spd_ref (0x700A) for velocity command

        Args:
            velocity: Target velocity (rad/s)
            limit_cur: Current limit (A), default 23.0
            acceleration: Acceleration limit (rad/s²), default 20.0

        Returns:
            Motor feedback data
        """
        # Switch to velocity mode if needed
        if self.current_mode != ControlMode.VELOCITY and self.pattern == 2:
            self._switch_mode(ControlMode.VELOCITY, auto_enable=True)

        # Always set limit_cur and acceleration to ensure they're applied
        # even when already in velocity mode
        self.set_parameter(ParameterIndex.LIMIT_CUR, limit_cur)
        time.sleep(0.001)
        self.set_parameter(ParameterIndex.ACC_RAD, acceleration)
        time.sleep(0.001)
        self.set_parameter(ParameterIndex.SPD_REF, velocity)
        return MotorFeedback(self.position, self.velocity, self.torque, self.temperature)

    def send_position_pp_command(
        self, angle: float, speed: float, acceleration: float
    ) -> MotorFeedback:
        """Send position control command in PP mode (Mode 1).

        Per official spec, PP mode requires:
        - Setting run_mode to 1 (position PP mode)
        - Setting vel_max (0x7024) for max speed
        - Setting acc_set (0x7025) for acceleration
        - Setting loc_ref (0x7016) for target position

        Note: Per spec, speed and acceleration changes during motion are not supported.
        To stop during motion, set vel_max to 0.

        Args:
            angle: Target angle (rad)
            speed: Speed limit (rad/s)
            acceleration: Acceleration limit (rad/s²)

        Returns:
            Motor feedback data
        """
        self._switch_mode(ControlMode.POSITION_PP, auto_enable=True)

        self.set_parameter(ParameterIndex.VEL_MAX, speed)
        time.sleep(0.001)
        self.set_parameter(ParameterIndex.ACC_SET, acceleration)
        time.sleep(0.001)
        self.set_parameter(ParameterIndex.LOC_REF, angle)
        time.sleep(0.001)

        return MotorFeedback(self.position, self.velocity, self.torque, self.temperature)

    def send_position_csp_command(self, angle: float, speed: float) -> MotorFeedback:
        """Send position control command in CSP mode (Mode 5).

        Args:
            angle: Target angle (rad)
            speed: Speed limit (rad/s)

        Returns:
            Motor feedback data
        """
        self._switch_mode(ControlMode.POSITION_CSP, auto_enable=True)

        # Send speed limit as float (rad/s) - this is the correct protocol behavior
        # Note: The original C++ code had a bug where it encoded speed with float_to_uint
        # then sent the encoded integer as a float, which is semantically incorrect.
        self.set_parameter(ParameterIndex.LIMIT_SPD_CSP, speed)
        time.sleep(0.001)
        self.set_parameter(ParameterIndex.LOC_REF, angle)
        time.sleep(0.001)

        return MotorFeedback(self.position, self.velocity, self.torque, self.temperature)

    def send_current_command(self, iq: float, id_val: float = 0.0) -> MotorFeedback:
        """Send current control command (Mode 3).

        Args:
            iq: Q-axis current (A)
            id_val: D-axis current (A)

        Returns:
            Motor feedback data
        """
        self._switch_mode(ControlMode.CURRENT, auto_enable=True)

        # Send Iq and Id as float values (A) - this is the correct protocol behavior
        # Note: The original C++ code had a bug where it encoded iq with float_to_uint
        # then sent the encoded integer as a float, which is semantically incorrect.
        self.set_parameter(ParameterIndex.IQ_REF, iq)
        time.sleep(0.001)
        self.set_parameter(ParameterIndex.ID_REF, id_val)
        time.sleep(0.001)

        return MotorFeedback(self.position, self.velocity, self.torque, self.temperature)

    def set_zero_position(self) -> None:
        """Set current position as zero."""
        self.disable_motor(clear_error=False)

        if self.current_mode != ControlMode.SET_ZERO:
            self.set_parameter(ParameterIndex.RUN_MODE, float(ControlMode.VELOCITY), is_mode=True)
            time.sleep(0.001)
            self.get_parameter(ParameterIndex.RUN_MODE)
            time.sleep(0.001)

        data = bytes([1] + [0] * 7)
        self._send_frame(CommunicationType.SET_POS_ZERO, self.master_id, data)
        time.sleep(0.001)

        self.enable_motor()

    def set_can_id(self, new_id: int, save: bool = False) -> bool:
        """Change motor CAN ID.

        After changing the ID, this instance will automatically update to
        communicate with the motor at the new ID.

        Args:
            new_id: New CAN ID for motor (1-127)
            save: If True, save the new ID to flash memory (persistent across power cycles)

        Returns:
            True if ID change was successful, False otherwise

        Raises:
            ValueError: If new_id is out of valid range (1-127)
        """
        if not 1 <= new_id <= 127:
            raise ValueError(f"CAN ID must be between 1 and 127, got {new_id}")

        old_id = self.motor_id
        self.disable_motor(clear_error=False)
        time.sleep(0.001)

        # Send CAN ID change command
        # Communication type 7: bit16~23 = new CAN ID, bit8~15 = master ID
        data = bytes([0] * 8)
        extra_data = (new_id << 8) | self.master_id
        self._send_frame(CommunicationType.CAN_ID, extra_data, data)
        time.sleep(0.01)

        # Update internal motor ID and reconfigure CAN filter for new ID
        self.motor_id = new_id
        filters: list[CanFilter] = [
            {
                "can_id": (self.motor_id << 8) | CAN_EFF_FLAG,
                "can_mask": 0xFF00 | CAN_EFF_FLAG,
                "extended": True,
            }
        ]
        self.bus.set_filters(filters)

        # Verify by trying to communicate with new ID
        time.sleep(0.01)
        try:
            # Try to enable motor at new ID to verify
            self.enable_motor()
            time.sleep(0.001)
            self.disable_motor(clear_error=False)

            # Save settings if requested
            if save:
                time.sleep(0.001)
                self.save_settings()

            return True
        except RuntimeError:
            # Revert to old ID if communication failed
            self.motor_id = old_id
            filters = [
                {
                    "can_id": (self.motor_id << 8) | CAN_EFF_FLAG,
                    "can_mask": 0xFF00 | CAN_EFF_FLAG,
                    "extended": True,
                }
            ]
            self.bus.set_filters(filters)
            return False

    def save_settings(self) -> None:
        """Save current settings to flash memory.

        This persists settings (like CAN ID) across power cycles.
        Requires firmware version 0.2.3.0 or later.

        Note: After saving, settings will be retained even after power off.
        """
        # Communication type 22: Save settings
        # Data must be: 01 02 03 04 05 06 07 08
        data = bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08])
        self._send_frame(CommunicationType.SAVE_SETTINGS, self.master_id, data)
        time.sleep(0.1)  # Give time for flash write
        # Response is motor feedback frame (type 2), handled by _receive_status_frame
        try:
            self._receive_status_frame(timeout=1.0)
        except RuntimeError:
            pass  # Some firmware versions may not respond

    def get_zero_sta(self) -> int | None:
        """Get the current zero position flag (zero_sta).

        The zero_sta flag determines the position range at power-on:
        - 0: Position range is 0 to 2π (default)
        - 1: Position range is -π to π

        Returns:
            Current zero_sta value (0 or 1), or None if read failed
        """
        # Read zero_sta (0x7029) - uint8 type
        data_bytes = bytearray(8)
        data_bytes[0] = ParameterIndex.ZERO_STA & 0xFF
        data_bytes[1] = (ParameterIndex.ZERO_STA >> 8) & 0xFF

        self._send_frame(CommunicationType.GET_SINGLE_PARAMETER, self.master_id, bytes(data_bytes))
        time.sleep(0.001)

        result = self._receive_frame(timeout=1.0)
        if result is None:
            return None

        communication_type, extra_data, _, data = result

        if communication_type != CommunicationType.GET_SINGLE_PARAMETER:
            return None

        # Check for read success (bit23~16 == 0x00)
        read_status = (extra_data >> 8) & 0xFF
        if read_status != 0x00:
            return None

        # Extract uint8 value from byte 4
        return int(data[4])

    def set_zero_sta(self, flag: int, save: bool = True) -> bool:
        """Set the zero position flag (zero_sta).

        The zero_sta flag determines the position range at power-on:
        - 0: Position range is 0 to 2π (default)
        - 1: Position range is -π to π

        This is useful when you want the motor to report positions in
        a symmetric range around zero (-π to π) instead of (0 to 2π).

        Args:
            flag: Zero position flag (0 or 1)
            save: If True, save the setting to flash memory (persistent across power cycles)

        Returns:
            True if setting was successful, False otherwise

        Raises:
            ValueError: If flag is not 0 or 1
        """
        if flag not in (0, 1):
            raise ValueError(f"zero_sta must be 0 or 1, got {flag}")

        # Set zero_sta (0x7029) - uint8 type
        self.set_parameter(ParameterIndex.ZERO_STA, float(flag), is_mode=True)
        time.sleep(0.001)

        # Verify the setting
        current_value = self.get_zero_sta()

        if current_value == flag:
            if save:
                time.sleep(0.001)
                self.save_settings()
            return True

        return False

    def read_initial_position(self, timeout: float = 10.0) -> float:
        """Read initial position from motor feedback.

        Waits for the first motor feedback frame and returns the initial position.
        This is useful for calibration and initialization.

        Args:
            timeout: Timeout in seconds to wait for feedback

        Returns:
            Initial position in radians

        Raises:
            RuntimeError: If no valid feedback is received within timeout
        """
        import time

        start_time = time.time()

        while True:
            if time.time() - start_time > timeout:
                raise RuntimeError("Timeout waiting for motor feedback")

            result = self._receive_frame(timeout=0.1)
            if result is None:
                continue

            communication_type, extra_data, host_id, data = result

            # Extract mid (master_id) and eid (motor_id) from extra_data
            # In C++: mid = (canid >> 8) & 0xFF, eid = canid & 0xFF
            # extra_data contains the middle 16 bits, but we need to check the layout
            mid = extra_data & 0xFF

            # Check for specific motor feedback frame (matching C++ implementation)
            # type == 0x02 && mid == 0x01 && eid == motor_id
            if (
                communication_type == CommunicationType.MOTOR_REQUEST
                and mid == self.master_id
                and host_id == self.motor_id
            ):
                if len(data) < 8:
                    continue

                # Parse position (big endian)
                position_u16 = (data[0] << 8) | data[1]
                op_params = ACTUATOR_OPERATION_MAPPING[self.actuator_type]
                position = (position_u16 / 32767.0 - 1.0) * op_params.position

                # Update internal state
                self.position = position
                velocity_u16 = (data[2] << 8) | data[3]
                torque_u16 = (data[4] << 8) | data[5]
                temperature_u16 = (data[6] << 8) | data[7]
                self.velocity = (velocity_u16 / 32767.0 - 1.0) * op_params.velocity
                self.torque = (torque_u16 / 32767.0 - 1.0) * op_params.torque
                self.temperature = temperature_u16 * 0.1

                return position

    def get_feedback(self) -> MotorFeedback:
        """Get current motor feedback.

        Returns:
            Motor feedback data
        """
        return MotorFeedback(self.position, self.velocity, self.torque, self.temperature)

    def get_string_parameter(self, index: int) -> str | None:
        """Get string parameter from motor.

        This method reads string-type parameters such as version info.
        String parameters are returned as null-terminated ASCII strings in bytes 4-7.

        Args:
            index: Parameter index (e.g., ParameterIndex.APP_CODE_VERSION)

        Returns:
            String value, or None if not received
        """
        data_bytes = bytearray(8)
        data_bytes[0] = index & 0xFF
        data_bytes[1] = (index >> 8) & 0xFF

        self._send_frame(CommunicationType.GET_SINGLE_PARAMETER, self.master_id, bytes(data_bytes))
        time.sleep(0.001)

        result = self._receive_frame(timeout=1.0)
        if result is None:
            return None

        communication_type, extra_data, _, data = result

        if communication_type != CommunicationType.GET_SINGLE_PARAMETER:
            return None

        # Check for read success (bit23~16 == 0x00)
        read_status = (extra_data >> 8) & 0xFF
        if read_status != 0x00:
            return None

        # Extract string from bytes 4-7 (null-terminated)
        if len(data) < 8:
            return None

        # String data is in bytes 4-7, null-terminated
        string_bytes = data[4:8]
        # Find null terminator and decode
        try:
            null_pos = string_bytes.find(b'\x00')
            if null_pos >= 0:
                string_bytes = string_bytes[:null_pos]
            return string_bytes.decode('ascii').strip()
        except (UnicodeDecodeError, ValueError):
            return None

    def get_string_parameter_full(self, index: int, max_chunks: int = 4) -> str | None:
        """Get full string parameter from motor (supports multi-chunk reading).

        Some string parameters (like version strings) may span multiple reads.
        This method attempts to read the full string by making multiple requests
        if needed.

        Args:
            index: Parameter index (e.g., ParameterIndex.APP_CODE_VERSION)
            max_chunks: Maximum number of chunks to read (each chunk is 4 bytes)

        Returns:
            Full string value, or None if not received
        """
        result_parts: list[str] = []

        for chunk_offset in range(max_chunks):
            data_bytes = bytearray(8)
            # Index with chunk offset for sequential reads
            chunk_index = index + chunk_offset
            data_bytes[0] = chunk_index & 0xFF
            data_bytes[1] = (chunk_index >> 8) & 0xFF

            self._send_frame(
                CommunicationType.GET_SINGLE_PARAMETER, self.master_id, bytes(data_bytes)
            )
            time.sleep(0.002)

            result = self._receive_frame(timeout=1.0)
            if result is None:
                break

            communication_type, extra_data, _, data = result

            if communication_type != CommunicationType.GET_SINGLE_PARAMETER:
                break

            # Check for read success
            read_status = (extra_data >> 8) & 0xFF
            if read_status != 0x00:
                break

            if len(data) < 8:
                break

            # Extract string chunk from bytes 4-7
            string_bytes = data[4:8]
            try:
                # Check for null terminator
                null_pos = string_bytes.find(b'\x00')
                if null_pos >= 0:
                    if null_pos > 0:
                        result_parts.append(string_bytes[:null_pos].decode('ascii'))
                    break
                result_parts.append(string_bytes.decode('ascii'))
            except (UnicodeDecodeError, ValueError):
                break

        if not result_parts:
            return None
        return ''.join(result_parts).strip()

    def get_firmware_info(self) -> FirmwareInfo:
        """Get firmware version information from motor.

        Reads various version-related parameters from the motor including
        bootloader version, application version, git commit, and build info.

        Note: Version information parameters (0x1000-0x1007) may not be supported
        by all firmware versions. If not supported, the corresponding fields will
        contain "unsupported" instead of actual values.

        Returns:
            FirmwareInfo object containing all version information
        """
        boot_version = self.get_string_parameter(ParameterIndex.BOOT_CODE_VERSION) or "unsupported"
        time.sleep(0.001)
        app_version = self.get_string_parameter(ParameterIndex.APP_CODE_VERSION) or "unsupported"
        time.sleep(0.001)
        git_version = self.get_string_parameter(ParameterIndex.APP_GIT_VERSION) or "unsupported"
        time.sleep(0.001)
        build_date = self.get_string_parameter(ParameterIndex.APP_BUILD_DATE) or "unsupported"
        time.sleep(0.001)
        build_time = self.get_string_parameter(ParameterIndex.APP_BUILD_TIME) or "unsupported"
        time.sleep(0.001)
        app_name = self.get_string_parameter(ParameterIndex.APP_CODE_NAME) or "unsupported"

        return FirmwareInfo(
            boot_version=boot_version,
            app_version=app_version,
            git_version=git_version,
            build_date=build_date,
            build_time=build_time,
            app_name=app_name,
        )
