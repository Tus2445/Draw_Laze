"""Quản lý lưu/khôi phục cấu hình ứng dụng bằng QSettings (file .ini cục bộ cạnh main.py hoặc file .exe)."""

import os
import sys
from typing import Optional

from PySide6.QtCore import QByteArray, QSettings

_SETTINGS_FILE_NAME = "uart_gui_settings.ini"


def _settings_file_path() -> str:
    if getattr(sys, "frozen", False):
        base_dir = os.path.dirname(os.path.abspath(sys.executable))
    else:
        base_dir = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(base_dir, _SETTINGS_FILE_NAME)


class AppSettings:
    """Lớp bọc mỏng quanh QSettings, cung cấp getter/setter an toàn kiểu dữ liệu."""

    def __init__(self, settings_path: Optional[str] = None):
        # Cho phép truyền đường dẫn file .ini tùy chỉnh (dùng trong unit test) để
        # không bao giờ ghi đè lên file cấu hình thật của ứng dụng.
        path = settings_path or _settings_file_path()
        self._settings = QSettings(path, QSettings.IniFormat)

    # --- Serial ---
    def last_port(self) -> str:
        return str(self._settings.value("serial/port", "", str))

    def set_last_port(self, port: str) -> None:
        self._settings.setValue("serial/port", port or "")

    def last_baudrate(self, default: int) -> int:
        try:
            return int(self._settings.value("serial/baudrate", default))
        except (TypeError, ValueError):
            return default

    def set_last_baudrate(self, baudrate: int) -> None:
        self._settings.setValue("serial/baudrate", int(baudrate))

    def last_timeout(self, default: float) -> float:
        try:
            return float(self._settings.value("serial/timeout", default))
        except (TypeError, ValueError):
            return default

    def set_last_timeout(self, timeout: float) -> None:
        self._settings.setValue("serial/timeout", float(timeout))

    # --- TAP Sender ---
    def last_tap_dir(self) -> str:
        return str(self._settings.value("tap/last_dir", "", str))

    def set_last_tap_dir(self, directory: str) -> None:
        self._settings.setValue("tap/last_dir", directory or "")

    def ack_value(self, default: str = "ok") -> str:
        value = self._settings.value("tap/ack_value", default, str)
        return str(value).strip() or default

    def set_ack_value(self, value: str) -> None:
        self._settings.setValue("tap/ack_value", value or "ok")

    # --- UART Terminal ---
    def terminal_hex_mode(self, default: bool = False) -> bool:
        return self._to_bool(self._settings.value("terminal/hex_mode", default), default)

    def set_terminal_hex_mode(self, enabled: bool) -> None:
        self._settings.setValue("terminal/hex_mode", bool(enabled))

    def terminal_auto_scroll(self, default: bool = True) -> bool:
        return self._to_bool(self._settings.value("terminal/auto_scroll", default), default)

    def set_terminal_auto_scroll(self, enabled: bool) -> None:
        self._settings.setValue("terminal/auto_scroll", bool(enabled))

    def terminal_append_lf(self, default: bool = True) -> bool:
        return self._to_bool(self._settings.value("terminal/append_lf", default), default)

    def set_terminal_append_lf(self, enabled: bool) -> None:
        self._settings.setValue("terminal/append_lf", bool(enabled))

    def terminal_line_ending_index(self, default: int = 0) -> int:
        try:
            idx = int(self._settings.value("terminal/line_ending_index", default))
            return idx if 0 <= idx <= 3 else default
        except (TypeError, ValueError):
            return default

    def set_terminal_line_ending_index(self, index: int) -> None:
        self._settings.setValue("terminal/line_ending_index", int(index))

    # --- Cửa sổ chính ---
    def window_geometry(self) -> Optional[QByteArray]:
        value = self._settings.value("window/geometry")
        return value if isinstance(value, QByteArray) else None

    def set_window_geometry(self, geometry: QByteArray) -> None:
        self._settings.setValue("window/geometry", geometry)

    def sync(self) -> None:
        self._settings.sync()

    @staticmethod
    def _to_bool(value, default: bool) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        try:
            return bool(int(value))
        except (TypeError, ValueError):
            return default