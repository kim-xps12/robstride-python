"""RobStrideモーター用CANバス初期化モジュール.

このモジュールはCANバス初期化機能を提供します:
- gs_usb: USBCANアダプタ（macOS互換性のためのモンキーパッチ付き）
- socketcan: SocketCAN（Linux標準のCANインターフェース）

ユーザーは明示的にインターフェース（'gs_usb' または 'socketcan'）を指定する必要があります。
"""

from typing import Literal

import can


def _apply_gs_usb_patch() -> None:
    """gs_usbをmacOSで動作させるためのモンキーパッチを適用する.

    gs_usbライブラリはLinuxカーネルドライバーの操作を前提に設計されているが、
    macOSにはこれらの機能がない。このパッチはカーネルドライバーチェックを
    スキップすることでgs_usbをmacOSで動作可能にする。
    """
    import gs_usb.gs_usb as gs_usb_module  # type: ignore
    from gs_usb.gs_usb import (  # type: ignore
        GS_CAN_MODE_HW_TIMESTAMP,
        GS_CAN_MODE_LISTEN_ONLY,
        GS_CAN_MODE_LOOP_BACK,
        GS_CAN_MODE_NORMAL,
        GS_CAN_MODE_ONE_SHOT,
        GS_CAN_MODE_START,
        _GS_USB_BREQ_MODE,
        DeviceMode,
    )

    def patched_start(
        self: "gs_usb_module.GsUsb",  # type: ignore
        flags: int = (GS_CAN_MODE_NORMAL | GS_CAN_MODE_HW_TIMESTAMP),
    ) -> None:
        """macOS互換のstart()：カーネルドライバーチェックをスキップする."""
        self.gs_usb.reset()

        # macOSではis_kernel_driver_active()が"Entity not found"エラーになるため、
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


def create_can_bus(
    interface: Literal["gs_usb", "socketcan"],
    channel: str = "can0",
    bitrate: int = 1000000,
    usb_vendor_id: int = 0x1D50,
    usb_product_id: int = 0x606F,
) -> can.BusABC:
    """指定されたインターフェースでCANバスを作成する.

    ユーザーは明示的にCANインターフェースバックエンドを指定する必要があります。

    Args:
        interface: CANインターフェースバックエンド。以下のいずれかを指定:
                   - 'gs_usb': USB CANアダプタ（例: candleLight）。macOS/Linuxで動作。
                   - 'socketcan': SocketCANインターフェース（Linuxのみ）。
        channel: CANチャンネル名（例: SocketCAN用の 'can0'）。デフォルト: 'can0'。
        bitrate: CANビットレート（bps）（デフォルト: 1000000）。SocketCANの場合、
                 インターフェースは 'ip link' コマンドで別途設定する必要があります。
        usb_vendor_id: gs_usbデバイスのUSBベンダーID（デフォルト: 0x1D50）
        usb_product_id: gs_usbデバイスのUSBプロダクトID（デフォルト: 0x606F）

    Returns:
        初期化されたCANバスオブジェクト

    Raises:
        RuntimeError: CANデバイスが見つからない、または初期化に失敗した場合
        ValueError: 無効なインターフェースが指定された場合

    Example:
        >>> # gs_usb (USB CANアダプタ)
        >>> bus = create_can_bus(interface="gs_usb", bitrate=1000000)
        >>>
        >>> # SocketCAN (Linux)
        >>> bus = create_can_bus(interface="socketcan", channel="can0")
    """
    if interface == "gs_usb":
        _apply_gs_usb_patch()

        import usb.core  # type: ignore[import-untyped]

        dev = usb.core.find(idVendor=usb_vendor_id, idProduct=usb_product_id)  # type: ignore[no-untyped-call]
        if dev is None:
            raise RuntimeError(
                f"CANデバイスが見つかりません (VID=0x{usb_vendor_id:04X}, PID=0x{usb_product_id:04X})。\n"
                "以下を確認してください:\n"
                "  1. USB CANアダプタが接続されている\n"
                "  2. ベンダーID/プロダクトIDが正しい\n"
                "  3. USB権限（macOSではsudoが必要な場合があります）"
            )

        return can.Bus(
            interface="gs_usb",
            channel=dev.product,
            bus=dev.bus,
            address=dev.address,
            bitrate=bitrate,
        )

    elif interface == "socketcan":
        return can.Bus(
            interface="socketcan",
            channel=channel,
            bitrate=bitrate,
            receive_own_messages=False,
        )

    else:
        raise ValueError(
            f"無効なインターフェース: '{interface}'。"
            "'gs_usb' または 'socketcan' を指定してください。"
        )


__all__ = ["create_can_bus"]
