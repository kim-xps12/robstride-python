"""
macOS / Linux 対応 CAN サンプル

- macOS: gs_usb + monkey patch
- Linux(Ubuntu含む): SocketCAN (can0 など)

使用方法:
    source venv/bin/activate

macOS:
    sudo python main.py

Linux:
    # 例: can0 を 1Mbps で上げる
    sudo ip link set can0 up type can bitrate 1000000
    python main.py
"""

import platform
import sys

import can

# macOS(gs_usb)でのみ使うので、importエラーを避けるため遅延importする
# import usb.core


def apply_macos_patch():
    """
    macOS用のモンキーパッチを適用する。

    gs_usbライブラリはLinux向けにカーネルドライバーの操作を行うが、
    macOSにはこの機能がないため、該当処理をスキップする。
    """
    import gs_usb.gs_usb as gs_usb_module
    from gs_usb.gs_usb import (
        DeviceMode,
        _GS_USB_BREQ_MODE,
        GS_CAN_MODE_START,
        GS_CAN_MODE_NORMAL,
        GS_CAN_MODE_HW_TIMESTAMP,
        GS_CAN_MODE_LISTEN_ONLY,
        GS_CAN_MODE_LOOP_BACK,
        GS_CAN_MODE_ONE_SHOT,
    )

    def patched_start(self, flags=(GS_CAN_MODE_NORMAL | GS_CAN_MODE_HW_TIMESTAMP)):
        """macOS用: カーネルドライバーのチェックをスキップしたstart()"""
        self.gs_usb.reset()

        # macOSではis_kernel_driver_active()が"Entity not found"等になるため、
        # カーネルドライバー関連の処理をスキップする

        flags &= self.device_capability.feature
        flags &= (
            GS_CAN_MODE_LISTEN_ONLY
            | GS_CAN_MODE_LOOP_BACK
            | GS_CAN_MODE_ONE_SHOT
            | GS_CAN_MODE_HW_TIMESTAMP
        )
        self.device_flags = flags

        mode = DeviceMode(GS_CAN_MODE_START, flags)
        self.gs_usb.ctrl_transfer(0x41, _GS_USB_BREQ_MODE, 0, 0, mode.pack())

    gs_usb_module.GsUsb.start = patched_start


def open_can_bus(bitrate: int = 1000000) -> can.BusABC:
    """
    OSに応じてCANバスを開く。

    macOS: gs_usb (USBデバイスをpyusbで見つけて接続)
    Linux: socketcan (can0を前提。必要なら環境変数等にしても良い)
    """
    os_name = platform.system()

    if os_name == "Darwin":
        # macOS: gs_usb + patch
        apply_macos_patch()

        import usb.core  # macOSのみで必要
        # candleLight互換などでよくある VID/PID（必要に応じて変更）
        dev = usb.core.find(idVendor=0x1D50, idProduct=0x606F)
        if dev is None:
            raise RuntimeError("CANデバイスが見つかりません (VID=0x1D50, PID=0x606F)")

        # python-can の gs_usb backend を開く
        return can.Bus(
            interface="gs_usb",
            channel=dev.product,
            bus=dev.bus,
            address=dev.address,
            bitrate=bitrate,
        )

    if os_name == "Linux":
        # Linux: SocketCAN
        # 事前に `ip link set can0 up type can bitrate 1000000` 等で起動しておくこと
        return can.Bus(
            interface="socketcan",
            channel="can0",
            bitrate=bitrate,  # socketcanでは無視される実装もあるが、害はない
        )

    raise RuntimeError(f"未対応OSです: {os_name}")


def main():
    bus = None
    try:
        bus = open_can_bus(bitrate=1000000)
        print("CANバスに接続しました")

        # メッセージ送信
        msg = can.Message(
            arbitration_id=0x123,
            data=[0x11, 0x22, 0x33, 0x44],
            is_extended_id=False,
        )
        bus.send(msg)
        print(f"送信: ID={msg.arbitration_id:03X}, Data={msg.data.hex()}")

        # メッセージ受信
        received = bus.recv(timeout=1.0)
        if received is not None:
            print(f"受信: ID={received.arbitration_id:03X}, Data={received.data.hex()}")
        else:
            print("受信: タイムアウト")

    except Exception as e:
        # 権限エラーの見分けは環境依存なので、まずは原因をそのまま出す
        print(f"エラー: {e}")
        print("")
        if platform.system() == "Darwin":
            print("macOSではUSBデバイスへのアクセスにroot権限が必要な場合があります。")
            print("  sudo python main.py")
        elif platform.system() == "Linux":
            print("LinuxではSocketCANインタフェース(can0等)がUPになっているか確認してください。")
            print("  sudo ip link set can0 up type can bitrate 1000000")
        sys.exit(1)
    finally:
        if bus is not None:
            bus.shutdown()


if __name__ == "__main__":
    main()
