import os
import queue
import re
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from typing import TYPE_CHECKING, Deque, List, Optional, Tuple

from PySide6.QtCore import Qt, QThread, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QIntValidator, QTextCharFormat, QTextCursor
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QFileDialog,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from log_export import LogExportWorker
from serial_manager import SerialManager

if TYPE_CHECKING:
    from app_settings import AppSettings

MAX_EXPORT_RECORDS = 50000


@dataclass
class ParsedTapCommand:
    line_number: int
    raw_line: str
    cleaned_command: str


def clean_gcode_line(line: str) -> str:
    clean_line = line.strip()
    clean_line = re.sub(r"\([^)]*\)", " ", clean_line)
    if ";" in clean_line:
        clean_line = clean_line.split(";")[0]
    clean_line = re.sub(r"\s+", " ", clean_line).strip()
    return clean_line


def parse_tap_file(file_path: str) -> tuple[int, List[ParsedTapCommand]]:
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Không tìm thấy file: '{file_path}'")

    with open(file_path, "r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()

    total_lines = len(lines)
    commands: List[ParsedTapCommand] = []
    for idx, raw in enumerate(lines, 1):
        cleaned = clean_gcode_line(raw)
        if cleaned:
            commands.append(
                ParsedTapCommand(
                    line_number=idx,
                    raw_line=raw.rstrip("\r\n"),
                    cleaned_command=cleaned,
                )
            )
    return total_lines, commands


def classify_response(
    response: str, ack_value: str = "ok", strict_ack: bool = True
) -> str:
    low = response.strip().lower()
    if not low:
        return "empty"

    ack_low = (ack_value or "ok").strip().lower()

    if strict_ack:
        if low == ack_low or low.startswith(ack_low + ":") or low.startswith(ack_low + " "):
            return "ok"
        if low == "error" or low.startswith("error:") or low.startswith("error "):
            return "error"
        if low == "alarm" or low.startswith("alarm:") or low.startswith("alarm "):
            return "alarm"
    else:
        if ack_low in low:
            return "ok"
        if "error" in low or "alarm" in low:
            return "error" if "error" in low else "alarm"

    return "info"


class TapSenderWorker(QThread):
    status_changed = Signal(str)
    progress_updated = Signal(int, int, int, str)
    log_event = Signal(str, str)
    transmission_finished = Signal(bool, str)
    # (line_number, command, sent_ok, ack_response, status) dùng cho Export Log
    command_result = Signal(int, str, bool, str, str)

    def __init__(
        self,
        serial_mgr: SerialManager,
        commands: List[ParsedTapCommand],
        total_file_lines: int,
        command_timeout: float = 5.0,
        boot_delay: float = 2.0,
        stop_on_timeout: bool = True,
        stop_on_error: bool = False,
        strict_ack: bool = True,
        ack_value: str = "ok",
        wait_on_timeout: bool = False,
        infinite_timeout: bool = False,
        queue_size: int = 1,
        inter_cmd_delay_ms: int = 0,
    ):
        super().__init__()
        self.serial_mgr = serial_mgr
        self.commands = commands
        self.total_file_lines = total_file_lines
        self.command_timeout = max(0.1, float(command_timeout))
        self.boot_delay = max(0.0, float(boot_delay))
        self.stop_on_timeout = stop_on_timeout
        self.stop_on_error = stop_on_error
        self.strict_ack = strict_ack
        self.ack_value = (ack_value or "ok").strip() or "ok"
        self.wait_on_timeout = wait_on_timeout
        self.infinite_timeout = bool(infinite_timeout)
        self.queue_size = max(1, int(queue_size))
        self.inter_cmd_delay_ms = max(0, int(inter_cmd_delay_ms))

        self._rx_queue: "queue.Queue[str]" = queue.Queue()
        self._stop_event = threading.Event()
        self._pause_event = threading.Event()
        self._pause_event.set()
        self._waiting_for_ack = False

    def on_line_received(self, line: str) -> None:
        # Nếu đang tạm dừng và không có lệnh .tap nào đang chờ ACK, bỏ qua
        # phản hồi của các lệnh gửi thủ công từ Tab 2 để không làm lệch ACK.
        if not self._pause_event.is_set() and not self._waiting_for_ack:
            return
        self._rx_queue.put(line)

    def request_pause(self, paused: bool) -> None:
        if paused:
            self._pause_event.clear()
            self.status_changed.emit("Paused")
            self.log_event.emit(
                "WARN",
                "Đã tạm dừng truyền file (Có thể gửi lệnh thủ công bên Tab 2 Terminal).",
            )
        else:
            # Nếu không có lệnh .tap nào đang dở dang chờ ACK, xóa sạch phản hồi
            # phát sinh từ việc gửi lệnh ở Tab 2 trong lúc tạm dừng trước khi chạy tiếp.
            if not self._waiting_for_ack:
                while not self._rx_queue.empty():
                    try:
                        self._rx_queue.get_nowait()
                    except queue.Empty:
                        break
            self._pause_event.set()
            self.status_changed.emit(
                "Waiting for Response" if self._waiting_for_ack else "Sending"
            )
            self.log_event.emit("INFO", "Tiếp tục truyền file.")

    def request_stop(self) -> None:
        self._stop_event.set()
        self._pause_event.set()

    def _sleep_interruptible(self, duration: float) -> bool:
        if duration <= 0:
            return not self._stop_event.is_set()
        end_time = time.monotonic() + duration
        while True:
            if self._stop_event.is_set():
                return False
            remaining = end_time - time.monotonic()
            if remaining <= 0:
                return True
            time.sleep(min(0.002, remaining))

    def run(self) -> None:
        total_cmds = len(self.commands)
        ack_count = 0
        error_count = 0

        if total_cmds == 0:
            self.status_changed.emit("Error")
            self.transmission_finished.emit(False, "File không chứa lệnh hợp lệ nào.")
            return

        if self.boot_delay > 0:
            self.status_changed.emit("Connecting")
            self.log_event.emit(
                "INFO",
                f"Chờ {self.boot_delay:.1f}s để vi điều khiển ổn định và xóa buffer...",
            )
            if not self._sleep_interruptible(self.boot_delay):
                self.status_changed.emit("Stopped")
                self.transmission_finished.emit(
                    False, "Đã hủy trong lúc chờ khởi động."
                )
                return

        self.serial_mgr.reset_buffers()
        while not self._rx_queue.empty():
            try:
                startup_line = self._rx_queue.get_nowait()
                self.log_event.emit("RX", f"Khởi động: {startup_line}")
            except queue.Empty:
                break

        if self.queue_size > 1:
            self.log_event.emit(
                "INFO",
                f"Chế độ gửi theo hàng đợi: Gửi liên tục tối đa {self.queue_size} lệnh "
                f"(độ trễ giữa các lệnh: {self.inter_cmd_delay_ms} ms) mới chờ 1 phản hồi '{self.ack_value}'.",
            )

        inter_delay_sec = self.inter_cmd_delay_ms / 1000.0
        batch_start = 0

        while batch_start < total_cmds:
            if self._stop_event.is_set():
                self.status_changed.emit("Stopped")
                self.transmission_finished.emit(
                    False,
                    f"Đã dừng bởi người dùng ({ack_count}/{total_cmds} lệnh hoàn tất).",
                )
                return

            while not self._pause_event.is_set():
                if self._stop_event.is_set():
                    self.status_changed.emit("Stopped")
                    self.transmission_finished.emit(
                        False,
                        f"Đã dừng bởi người dùng ({ack_count}/{total_cmds} lệnh hoàn tất).",
                    )
                    return
                time.sleep(0.05)

            if not self.serial_mgr.is_connected:
                self.status_changed.emit("Error")
                self.transmission_finished.emit(
                    False, "Mất kết nối UART trong khi đang truyền file."
                )
                return

            batch = self.commands[batch_start : batch_start + self.queue_size]
            batch_len = len(batch)
            first_idx = batch_start + 1
            last_idx = batch_start + batch_len

            self.status_changed.emit("Sending")
            self._waiting_for_ack = True

            # Gửi liên tục toàn bộ các lệnh trong lô (hàng đợi) hiện tại
            for offset, item in enumerate(batch):
                curr_idx = batch_start + offset + 1
                if self._stop_event.is_set():
                    self._waiting_for_ack = False
                    self.status_changed.emit("Stopped")
                    self.transmission_finished.emit(
                        False,
                        f"Đã dừng bởi người dùng ({ack_count}/{total_cmds} lệnh hoàn tất).",
                    )
                    return

                self.progress_updated.emit(
                    ack_count, curr_idx, total_cmds, item.cleaned_command
                )

                payload = (item.cleaned_command + "\n").encode("ascii", errors="replace")
                if not self.serial_mgr.write_bytes(payload, source="TAP"):
                    self._waiting_for_ack = False
                    self.status_changed.emit("Error")
                    self.command_result.emit(
                        item.line_number, item.cleaned_command, False, "", "SEND_FAILED"
                    )
                    self.transmission_finished.emit(
                        False,
                        f"Lỗi gửi lệnh tại dòng {item.line_number}: {item.cleaned_command}",
                    )
                    return

                if self.queue_size > 1:
                    self.log_event.emit(
                        "TX",
                        f"[Lô {first_idx}..{last_idx} ({offset + 1}/{batch_len}) | "
                        f"Lệnh {curr_idx}/{total_cmds} | Dòng {item.line_number}/{self.total_file_lines}] "
                        f"Gửi: {item.cleaned_command}",
                    )
                else:
                    self.log_event.emit(
                        "TX",
                        f"[Lệnh {curr_idx}/{total_cmds} | Dòng {item.line_number}/{self.total_file_lines}] "
                        f"Gửi: {item.cleaned_command}",
                    )

                # Tạo độ trễ nhỏ (ms) giữa các lệnh trong cùng một lô để tránh tràn RX buffer trên STM32
                if offset < batch_len - 1 and inter_delay_sec > 0:
                    if not self._sleep_interruptible(inter_delay_sec):
                        self._waiting_for_ack = False
                        self.status_changed.emit("Stopped")
                        self.transmission_finished.emit(
                            False,
                            f"Đã dừng bởi người dùng ({ack_count}/{total_cmds} lệnh hoàn tất).",
                        )
                        return

            # Sau khi đã gửi đủ N lệnh (hoặc các lệnh cuối cùng của file), chờ 1 ACK duy nhất
            if self._pause_event.is_set():
                self.status_changed.emit("Waiting for Response")

            start_wait = time.monotonic()
            deadline = start_wait + self.command_timeout
            max_deadline = start_wait + (self.command_timeout * 1.5)
            batch_acknowledged = False

            last_item = batch[-1]
            batch_desc = (
                f"lệnh #{first_idx} ({last_item.cleaned_command})"
                if batch_len == 1
                else f"lô lệnh #{first_idx}..#{last_idx} ({batch_len} lệnh, cuối: {last_item.cleaned_command})"
            )

            while True:
                if self._stop_event.is_set():
                    self._waiting_for_ack = False
                    self.status_changed.emit("Stopped")
                    for it in batch:
                        self.command_result.emit(
                            it.line_number, it.cleaned_command, True, "", "STOPPED"
                        )
                    self.transmission_finished.emit(
                        False, "Đã dừng gửi lệnh theo yêu cầu người dùng."
                    )
                    return

                if not self.serial_mgr.is_connected:
                    self._waiting_for_ack = False
                    self.status_changed.emit("Error")
                    for it in batch:
                        self.command_result.emit(
                            it.line_number, it.cleaned_command, True, "", "DISCONNECTED"
                        )
                    self.transmission_finished.emit(
                        False, "Thiết bị ngắt kết nối khi đang chờ phản hồi."
                    )
                    return

                # Nếu người dùng đang bấm Tạm dừng ngay khi lô lệnh hiện tại chưa nhận xong ACK,
                # tạm dừng tính thời gian Timeout cho đến khi bấm Tiếp tục hoặc nhận được ACK.
                if not self._pause_event.is_set() and self._rx_queue.empty():
                    pause_tick = time.monotonic()
                    time.sleep(0.05)
                    paused_delta = time.monotonic() - pause_tick
                    start_wait += paused_delta
                    deadline += paused_delta
                    max_deadline += paused_delta
                    continue

                now = time.monotonic()
                no_timeout_active = self.infinite_timeout or bool(
                    getattr(self.serial_mgr, "no_timeout", False)
                )
                if not no_timeout_active and now >= deadline:
                    if self.wait_on_timeout:
                        elapsed_wait = now - start_wait
                        self.log_event.emit(
                            "WARN",
                            f"Đã chờ {elapsed_wait:.1f}s cho {batch_desc} "
                            f"— Vẫn đang tiếp tục chờ phản hồi '{self.ack_value}'...",
                        )
                        deadline = now + self.command_timeout
                        max_deadline = deadline + (self.command_timeout * 0.5)
                    else:
                        break

                try:
                    response = self._rx_queue.get(timeout=0.05)
                except queue.Empty:
                    continue

                category = classify_response(
                    response, ack_value=self.ack_value, strict_ack=self.strict_ack
                )
                if category == "ok":
                    ack_count += batch_len
                    batch_acknowledged = True
                    self._waiting_for_ack = False
                    if batch_len > 1:
                        self.log_event.emit(
                            "OK",
                            f"Phản hồi cho lô #{first_idx}..#{last_idx} ({batch_len} lệnh): {response}",
                        )
                    else:
                        self.log_event.emit("OK", f"Phản hồi: {response}")

                    for it in batch:
                        self.command_result.emit(
                            it.line_number, it.cleaned_command, True, response, "OK"
                        )
                    self.progress_updated.emit(
                        ack_count, last_idx, total_cmds, last_item.cleaned_command
                    )
                    break
                elif category in ("error", "alarm"):
                    error_count += 1
                    self._waiting_for_ack = False
                    self.log_event.emit(
                        "ERR", f"Cảnh báo/Lỗi từ thiết bị: {response}"
                    )
                    for it in batch:
                        self.command_result.emit(
                            it.line_number,
                            it.cleaned_command,
                            True,
                            response,
                            category.upper(),
                        )
                    if self.stop_on_error:
                        self.status_changed.emit("Error")
                        self.transmission_finished.emit(
                            False,
                            f"Dừng truyền do thiết bị báo lỗi tại {batch_desc}: {response}",
                        )
                        return
                    else:
                        batch_acknowledged = True
                        break
                else:
                    self.log_event.emit("RX", f"Thiết bị: {response}")
                    if self.wait_on_timeout:
                        deadline = max(deadline, time.monotonic() + 0.5)
                    else:
                        deadline = min(
                            max_deadline, max(deadline, time.monotonic() + 0.5)
                        )

            self._waiting_for_ack = False
            if not batch_acknowledged:
                error_count += 1
                timeout_msg = (
                    f"Quá thời gian chờ ({self.command_timeout:.1f}s) cho {batch_desc}"
                )
                self.log_event.emit("ERR", timeout_msg)
                for it in batch:
                    self.command_result.emit(
                        it.line_number, it.cleaned_command, True, "", "TIMEOUT"
                    )
                if self.stop_on_timeout:
                    self.status_changed.emit("Error")
                    self.transmission_finished.emit(False, timeout_msg)
                    return
                else:
                    self.log_event.emit(
                        "WARN",
                        "Tiếp tục gửi lô lệnh kế tiếp theo cấu hình bỏ qua Timeout.",
                    )

            batch_start += batch_len

        if error_count == 0 and ack_count == total_cmds:
            self.status_changed.emit("Completed")
            self.transmission_finished.emit(
                True,
                f"Hoàn thành truyền toàn bộ file G-code ({ack_count}/{total_cmds} lệnh thành công).",
            )
        else:
            self.status_changed.emit("Error")
            self.transmission_finished.emit(
                False,
                f"Kết thúc truyền file với {error_count} lỗi/cảnh báo "
                f"({ack_count}/{total_cmds} lệnh nhận được '{self.ack_value}').",
            )


class TapPreviewDialog(QDialog):
    def __init__(
        self,
        file_path: str,
        total_lines: int,
        commands: List[ParsedTapCommand],
        parent: Optional[QWidget] = None,
    ):
        super().__init__(parent)
        self.setWindowTitle(f"Xem trước lệnh đã xử lý — {os.path.basename(file_path)}")
        self.resize(760, 500)

        layout = QVBoxLayout(self)
        summary = QLabel(
            f"<b>File:</b> {file_path}<br>"
            f"<b>Tổng số dòng trong file:</b> {total_lines} | "
            f"<b>Số lệnh hợp lệ sẽ gửi:</b> {len(commands)} | "
            f"<b>Số dòng trống/chú thích đã lọc:</b> {total_lines - len(commands)}"
        )
        layout.addWidget(summary)

        table = QTableWidget(len(commands), 3, self)
        table.setHorizontalHeaderLabels(
            ["STT Lệnh", "Dòng gốc", "Lệnh G-code sau khi lọc (; và (...))"]
        )
        table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        table.setEditTriggers(QTableWidget.NoEditTriggers)
        table.setAlternatingRowColors(True)

        for row, cmd in enumerate(commands):
            table.setItem(row, 0, QTableWidgetItem(str(row + 1)))
            table.setItem(row, 1, QTableWidgetItem(str(cmd.line_number)))
            table.setItem(row, 2, QTableWidgetItem(cmd.cleaned_command))

        layout.addWidget(table)

        close_btn = QPushButton("Đóng")
        close_btn.setMinimumWidth(100)
        close_btn.clicked.connect(self.accept)
        layout.addWidget(close_btn, alignment=Qt.AlignRight)


class TapFileSenderTab(QWidget):
    """Giao diện Tab 1: Chọn file .tap, xem trước, chọn xung laze, điều khiển gửi và theo dõi tiến độ."""

    request_auto_connect = Signal()
    transmission_state_changed = Signal(bool)  # True khi đang truyền tích cực, False khi dừng/tạm dừng

    def __init__(
        self,
        serial_mgr: SerialManager,
        settings: Optional["AppSettings"] = None,
        parent: Optional[QWidget] = None,
    ):
        super().__init__(parent)
        self.serial_mgr = serial_mgr
        self.settings = settings
        self.current_file_path: str = ""
        self.total_file_lines: int = 0
        self.parsed_commands: List[ParsedTapCommand] = []
        self.worker: Optional[TapSenderWorker] = None
        self._start_timestamp: Optional[float] = None
        self._rx_signal_count: int = 0
        self._last_browse_dir: str = settings.last_tap_dir() if settings else ""
        self._records: Deque[Tuple[str, str, str, str, str, str]] = deque(
            maxlen=MAX_EXPORT_RECORDS
        )
        self._export_worker: Optional[LogExportWorker] = None

        # Trạng thái chọn và xác nhận (ACK) loại xung laze
        self._waiting_laser_ack: bool = False
        self._laser_ack_received: bool = False
        self._pending_laser_cmd: str = ""
        self._confirmed_laser_cmd: str = ""

        self._no_timeout_mode: bool = False
        self._saved_timeout_policy_idx: int = 0

        self._elapsed_timer = QTimer(self)
        self._elapsed_timer.setInterval(200)
        self._elapsed_timer.timeout.connect(self._update_elapsed_time)

        self._build_ui()

        if settings:
            self.ack_value_edit.setText(settings.ack_value())
            if hasattr(settings, "queue_size"):
                self.queue_size_edit.setText(str(settings.queue_size(1)))
            if hasattr(settings, "inter_cmd_delay_ms"):
                self.inter_cmd_delay_spin.setValue(settings.inter_cmd_delay_ms(5))
        self.ack_value_edit.textChanged.connect(self._on_ack_value_changed)
        self.queue_size_edit.textChanged.connect(self._on_queue_size_changed)
        self.queue_size_edit.editingFinished.connect(self._normalize_queue_size_input)
        self.inter_cmd_delay_spin.valueChanged.connect(self._on_inter_cmd_delay_changed)

        # Luôn lắng nghe tín hiệu UART ngoài gửi tới và trạng thái kết nối
        self.serial_mgr.line_received.connect(self._on_external_signal_received)
        self.serial_mgr.connection_changed.connect(self._on_serial_connection_changed)

    @property
    def is_transmitting(self) -> bool:
        return self.worker is not None and self.worker.isRunning()

    def _build_ui(self) -> None:
        outer_layout = QVBoxLayout(self)
        outer_layout.setContentsMargins(0, 0, 0, 0)

        scroll_area = QScrollArea(self)
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QFrame.NoFrame)

        content_widget = QWidget()
        main_layout = QVBoxLayout(content_widget)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(8)

        # 1. Khu vực chọn file .tap
        file_group = QGroupBox("1. Chọn File G-code / .TAP")
        file_group.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        file_layout = QGridLayout(file_group)
        file_layout.setContentsMargins(10, 10, 10, 8)
        file_layout.setHorizontalSpacing(10)
        file_layout.setVerticalSpacing(6)

        self.file_path_edit = QLineEdit()
        self.file_path_edit.setFixedHeight(28)
        self.file_path_edit.setPlaceholderText("Chọn file .tap, .gcode hoặc .nc...")
        self.file_path_edit.setReadOnly(True)

        self.browse_btn = QPushButton("Chọn File...")
        self.browse_btn.setFixedHeight(28)
        self.browse_btn.setMinimumWidth(110)
        self.browse_btn.clicked.connect(self._on_browse_file)

        self.preview_btn = QPushButton("Xem trước lệnh")
        self.preview_btn.setFixedHeight(28)
        self.preview_btn.setMinimumWidth(130)
        self.preview_btn.setEnabled(False)
        self.preview_btn.clicked.connect(self._on_preview_commands)

        self.file_info_label = QLabel(
            "Chưa chọn file. (Hỗ trợ lọc comment ';' và '(...)' theo chuẩn test_UART.py)"
        )
        self.file_info_label.setStyleSheet("color: #555555;")

        file_layout.addWidget(QLabel("Đường dẫn file:"), 0, 0)
        file_layout.addWidget(self.file_path_edit, 0, 1)
        file_layout.addWidget(self.browse_btn, 0, 2)
        file_layout.addWidget(self.preview_btn, 0, 3)
        file_layout.addWidget(self.file_info_label, 1, 0, 1, 4)
        file_layout.setColumnStretch(1, 1)
        main_layout.addWidget(file_group)

        # Khu vực Chọn Loại Xung Laze (Nằm ngay trên mục 2. Điều khiển Truyền)
        laser_group = QGroupBox("Chọn Loại Xung Laze")
        laser_group.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        laser_layout = QHBoxLayout(laser_group)
        laser_layout.setContentsMargins(10, 10, 10, 8)
        laser_layout.setSpacing(10)

        lbl_laser = QLabel("Chọn loại xung laze:")
        self.laser_type_combo = QComboBox()
        self.laser_type_combo.setFixedHeight(28)
        self.laser_type_combo.setMinimumWidth(160)
        self.laser_type_combo.addItem("Loại 1", "Laze1")
        self.laser_type_combo.addItem("Loại 2", "Laze2")
        self.laser_type_combo.currentIndexChanged.connect(self._on_laser_type_changed)

        self.send_laser_btn = QPushButton("Gửi yêu cầu")
        self.send_laser_btn.setFixedHeight(28)
        self.send_laser_btn.setMinimumWidth(130)
        self.send_laser_btn.setStyleSheet(
            "QPushButton { background-color: #0d6efd; color: white; font-weight: bold; padding: 4px 12px; border-radius: 4px; }"
            "QPushButton:disabled { background-color: #9ec5fe; color: #ffffff; }"
        )
        self.send_laser_btn.clicked.connect(self._on_send_laser_request)

        self.laser_status_lbl = QLabel("● Chưa chọn / gửi yêu cầu xung laze")
        self.laser_status_lbl.setStyleSheet("color: #6c757d; font-weight: bold;")

        laser_layout.addWidget(lbl_laser)
        laser_layout.addWidget(self.laser_type_combo)
        laser_layout.addWidget(self.send_laser_btn)
        laser_layout.addSpacing(10)
        laser_layout.addWidget(self.laser_status_lbl)
        laser_layout.addStretch()
        main_layout.addWidget(laser_group)

        # 2. Khu vực điều khiển truyền và cấu hình giao thức
        ctrl_group = QGroupBox("2. Điều khiển Truyền && Tùy chọn Phản hồi")
        ctrl_group.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        ctrl_layout = QVBoxLayout(ctrl_group)
        ctrl_layout.setContentsMargins(10, 10, 10, 10)
        ctrl_layout.setSpacing(8)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(10)

        self.start_btn = QPushButton("▶ Bắt đầu Gửi (Start)")
        self.start_btn.setFixedHeight(32)
        self.start_btn.setMinimumWidth(170)
        self.start_btn.setStyleSheet(
            "QPushButton { background-color: #198754; color: white; font-weight: bold; padding: 4px 14px; border-radius: 4px; }"
            "QPushButton:disabled { background-color: #a3cfbb; color: #ffffff; }"
        )
        self.start_btn.clicked.connect(self._on_start_sending)

        self.pause_btn = QPushButton("⏸ Tạm dừng (Pause)")
        self.pause_btn.setFixedHeight(32)
        self.pause_btn.setMinimumWidth(160)
        self.pause_btn.setEnabled(False)
        self.pause_btn.clicked.connect(self._on_toggle_pause)

        self.stop_btn = QPushButton("⏹ Dừng (Stop)")
        self.stop_btn.setFixedHeight(32)
        self.stop_btn.setMinimumWidth(140)
        self.stop_btn.setStyleSheet(
            "QPushButton { background-color: #dc3545; color: white; font-weight: bold; padding: 4px 14px; border-radius: 4px; }"
            "QPushButton:disabled { background-color: #f1aeb5; color: #ffffff; }"
        )
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self._on_stop_sending)

        self.clear_log_btn = QPushButton("Xóa Log")
        self.clear_log_btn.setFixedHeight(32)
        self.clear_log_btn.setMinimumWidth(100)
        self.clear_log_btn.clicked.connect(self._on_clear_log)

        self.export_log_btn = QPushButton("Xuất Log (Export)")
        self.export_log_btn.setFixedHeight(32)
        self.export_log_btn.setMinimumWidth(150)
        self.export_log_btn.clicked.connect(self._on_export_log)

        btn_row.addWidget(self.start_btn)
        btn_row.addWidget(self.pause_btn)
        btn_row.addWidget(self.stop_btn)
        btn_row.addStretch()
        btn_row.addWidget(self.export_log_btn)
        btn_row.addWidget(self.clear_log_btn)
        ctrl_layout.addLayout(btn_row)

        opts_grid = QGridLayout()
        opts_grid.setHorizontalSpacing(14)
        opts_grid.setVerticalSpacing(8)

        self.auto_connect_chk = QCheckBox("Tự động kết nối trước khi gửi")
        self.auto_connect_chk.setMinimumHeight(24)
        self.auto_connect_chk.setChecked(True)

        self.boot_delay_chk = QCheckBox(
            "Chờ VĐK khởi động 2 giây (giống test_UART.py)"
        )
        self.boot_delay_chk.setMinimumHeight(24)
        self.boot_delay_chk.setChecked(True)

        lbl_inter_delay = QLabel("Độ trễ giữa các lệnh trong lô (ms):")
        self.inter_cmd_delay_spin = QSpinBox()
        self.inter_cmd_delay_spin.setRange(0, 5000)
        self.inter_cmd_delay_spin.setSingleStep(1)
        self.inter_cmd_delay_spin.setValue(5)
        self.inter_cmd_delay_spin.setFixedHeight(28)
        self.inter_cmd_delay_spin.setSuffix(" ms")
        self.inter_cmd_delay_spin.setToolTip(
            "Khoảng nghỉ nhỏ (mili-giây) giữa các lệnh gửi liên tiếp trong cùng một hàng đợi "
            "để tránh tràn bộ đệm UART RX trên vi điều khiển (STM32)."
        )

        opts_grid.addWidget(self.auto_connect_chk, 0, 0)
        opts_grid.addWidget(self.boot_delay_chk, 0, 1)
        opts_grid.addWidget(lbl_inter_delay, 0, 2)
        opts_grid.addWidget(self.inter_cmd_delay_spin, 0, 3)

        lbl_timeout_pol = QLabel("Khi hết Timeout:")
        self.timeout_policy_combo = QComboBox()
        self.timeout_policy_combo.setFixedHeight(28)
        self._normal_timeout_policies = [
            "Tiếp tục chờ phản hồi",
            "Dừng truyền ngay (An toàn)",
            "Bỏ qua & gửi lệnh tiếp",
        ]
        self.timeout_policy_combo.addItems(self._normal_timeout_policies)
        self.timeout_policy_combo.currentIndexChanged.connect(
            self._on_timeout_policy_combo_changed
        )

        lbl_err_pol = QLabel("Khi nhận 'error'/'alarm':")
        self.error_policy_combo = QComboBox()
        self.error_policy_combo.setFixedHeight(28)
        self.error_policy_combo.addItems(
            ["Dừng truyền ngay (An toàn)", "Ghi cảnh báo & gửi tiếp (Giống code gốc)"]
        )

        opts_grid.addWidget(lbl_timeout_pol, 1, 0)
        opts_grid.addWidget(self.timeout_policy_combo, 1, 1)
        opts_grid.addWidget(lbl_err_pol, 1, 2)
        opts_grid.addWidget(self.error_policy_combo, 1, 3)
        opts_grid.setColumnStretch(1, 1)
        opts_grid.setColumnStretch(3, 1)

        lbl_ack_value = QLabel("Giá trị xác nhận (ACK) mong đợi:")
        self.ack_value_edit = QLineEdit("ok")
        self.ack_value_edit.setFixedHeight(28)
        self.ack_value_edit.setPlaceholderText("VD: ok, done, ready...")
        self.ack_value_edit.setToolTip(
            "Chuỗi tín hiệu UART ngoài gửi về được xem là xác nhận hoàn tất lệnh "
            "(mặc định là 'ok', có thể đổi thành bất kỳ giá trị nào)"
        )

        lbl_queue_size = QLabel("Số lượng hàng đợi UART (lệnh/1 ACK):")
        self.queue_size_edit = QLineEdit("1")
        self.queue_size_edit.setFixedHeight(28)
        self.queue_size_edit.setValidator(QIntValidator(1, 1000, self.queue_size_edit))
        self.queue_size_edit.setPlaceholderText("1..1000")
        self.queue_size_edit.setToolTip(
            "Số lệnh gửi liên tiếp qua UART trước khi dừng lại chờ 1 phản hồi ACK "
            "(1 = gửi từng lệnh một; N > 1 = gửi liên tục N lệnh rồi chờ 1 ACK)."
        )
        lbl_queue_unit = QLabel("lệnh")

        queue_size_box_row = QHBoxLayout()
        queue_size_box_row.setContentsMargins(0, 0, 0, 0)
        queue_size_box_row.setSpacing(6)
        queue_size_box_row.addWidget(self.queue_size_edit, stretch=1)
        queue_size_box_row.addWidget(lbl_queue_unit)

        opts_grid.addWidget(lbl_ack_value, 2, 0)
        opts_grid.addWidget(self.ack_value_edit, 2, 1)
        opts_grid.addWidget(lbl_queue_size, 2, 2)
        opts_grid.addLayout(queue_size_box_row, 2, 3)

        ctrl_layout.addLayout(opts_grid)
        main_layout.addWidget(ctrl_group)

        # 3. Khu vực Tiến độ (Progress Panel)
        prog_group = QGroupBox("3. Tiến độ Truyền File")
        prog_group.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        prog_layout = QGridLayout(prog_group)
        prog_layout.setContentsMargins(10, 10, 10, 8)
        prog_layout.setHorizontalSpacing(12)
        prog_layout.setVerticalSpacing(6)

        self.progress_bar = QProgressBar()
        self.progress_bar.setFixedHeight(22)
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("%p% (0/0 lệnh)")

        self.status_value_lbl = QLabel("Idle")
        self.status_value_lbl.setStyleSheet("font-weight: bold; color: #0d6efd;")

        self.ack_count_lbl = QLabel("0 / 0")
        self.current_cmd_num_lbl = QLabel("0")
        self.elapsed_time_lbl = QLabel("00:00.0")
        self.current_cmd_text_lbl = QLabel("—")
        self.current_cmd_text_lbl.setFixedHeight(26)
        self.current_cmd_text_lbl.setSizePolicy(
            QSizePolicy.Expanding, QSizePolicy.Fixed
        )
        self.current_cmd_text_lbl.setStyleSheet(
            "font-family: Consolas, monospace; background: #f8f9fa; padding: 3px 8px; border: 1px solid #dee2e6; border-radius: 3px;"
        )

        prog_layout.addWidget(self.progress_bar, 0, 0, 1, 6)
        prog_layout.addWidget(QLabel("Trạng thái:"), 1, 0)
        prog_layout.addWidget(self.status_value_lbl, 1, 1)
        prog_layout.addWidget(QLabel("Đã xác nhận (ACK):"), 1, 2)
        prog_layout.addWidget(self.ack_count_lbl, 1, 3)
        prog_layout.addWidget(QLabel("Thời gian:"), 1, 4)
        prog_layout.addWidget(self.elapsed_time_lbl, 1, 5)

        prog_layout.addWidget(QLabel("Lệnh hiện tại (#):"), 2, 0)
        prog_layout.addWidget(self.current_cmd_num_lbl, 2, 1)
        prog_layout.addWidget(QLabel("Nội dung lệnh:"), 2, 2)
        prog_layout.addWidget(self.current_cmd_text_lbl, 2, 3, 1, 3)
        prog_layout.setColumnStretch(3, 1)
        prog_layout.setColumnStretch(5, 1)

        main_layout.addWidget(prog_group)

        # Tạo QSplitter dọc cho Mục 4 và Mục 5
        logs_splitter = QSplitter(Qt.Vertical)
        logs_splitter.setChildrenCollapsible(False)

        # 4. Khu vực hiển thị tín hiệu UART ngoài gửi tới
        rx_group = QGroupBox("4. Tín hiệu nhận từ UART ngoài")
        rx_layout = QGridLayout(rx_group)
        rx_layout.setContentsMargins(10, 10, 10, 8)
        rx_layout.setHorizontalSpacing(10)
        rx_layout.setVerticalSpacing(6)

        self.last_rx_value_edit = QLineEdit()
        self.last_rx_value_edit.setReadOnly(True)
        self.last_rx_value_edit.setFixedHeight(26)
        self.last_rx_value_edit.setPlaceholderText("Chưa nhận tín hiệu nào...")
        self.last_rx_value_edit.setStyleSheet(
            "font-family: Consolas, monospace; font-weight: bold; background: #f8f9fa;"
        )

        self.rx_signal_count_lbl = QLabel("0")
        self.rx_signal_count_lbl.setStyleSheet("font-weight: bold;")

        rx_layout.addWidget(QLabel("Tín hiệu mới nhất:"), 0, 0)
        rx_layout.addWidget(self.last_rx_value_edit, 0, 1)
        rx_layout.addWidget(QLabel("Tổng số tín hiệu đã nhận:"), 0, 2)
        rx_layout.addWidget(self.rx_signal_count_lbl, 0, 3)
        rx_layout.setColumnStretch(1, 1)

        self.rx_signal_log = QPlainTextEdit()
        self.rx_signal_log.setReadOnly(True)
        self.rx_signal_log.setMaximumBlockCount(5000)
        self.rx_signal_log.setMinimumHeight(60)
        rx_font = QFont("Consolas", 10)
        rx_font.setStyleHint(QFont.Monospace)
        self.rx_signal_log.setFont(rx_font)
        rx_layout.addWidget(self.rx_signal_log, 1, 0, 1, 4)

        logs_splitter.addWidget(rx_group)

        # 5. Khu vực Nhật ký Truyền (Transmission Log)
        log_group = QGroupBox("5. Nhật ký Truyền (Transmission Log)")
        log_layout = QVBoxLayout(log_group)
        log_layout.setContentsMargins(10, 10, 10, 8)

        self.log_edit = QPlainTextEdit()
        self.log_edit.setReadOnly(True)
        self.log_edit.setMaximumBlockCount(10000)
        self.log_edit.setMinimumHeight(110)
        font = QFont("Consolas", 10)
        font.setStyleHint(QFont.Monospace)
        self.log_edit.setFont(font)

        log_layout.addWidget(self.log_edit)
        logs_splitter.addWidget(log_group)
        logs_splitter.setStretchFactor(0, 1)
        logs_splitter.setStretchFactor(1, 2)

        main_layout.addWidget(logs_splitter, stretch=1)

        scroll_area.setWidget(content_widget)
        outer_layout.addWidget(scroll_area)

    def _on_timeout_policy_combo_changed(self, index: int) -> None:
        if not self._no_timeout_mode and 0 <= index < len(self._normal_timeout_policies):
            self._saved_timeout_policy_idx = index

    def set_no_timeout_mode(self, no_timeout: bool) -> None:
        """Khóa ô 'Khi hết Timeout' và hiển thị trạng thái đang ở chế độ Không Timeout."""
        no_timeout = bool(no_timeout)
        if no_timeout == self._no_timeout_mode:
            self.timeout_policy_combo.setEnabled(
                (not self.is_transmitting) and (not self._no_timeout_mode)
            )
            return

        self._no_timeout_mode = no_timeout
        self.timeout_policy_combo.blockSignals(True)
        if no_timeout:
            curr = self.timeout_policy_combo.currentIndex()
            if 0 <= curr < len(self._normal_timeout_policies):
                self._saved_timeout_policy_idx = curr
            self.timeout_policy_combo.clear()
            self.timeout_policy_combo.addItem("Đang ở chế độ Không Timeout (Chờ vô hạn)")
            self.timeout_policy_combo.setCurrentIndex(0)
            self.timeout_policy_combo.setEnabled(False)
        else:
            self.timeout_policy_combo.clear()
            self.timeout_policy_combo.addItems(self._normal_timeout_policies)
            restore_idx = (
                self._saved_timeout_policy_idx
                if 0 <= self._saved_timeout_policy_idx < len(self._normal_timeout_policies)
                else 0
            )
            self.timeout_policy_combo.setCurrentIndex(restore_idx)
            self.timeout_policy_combo.setEnabled(not self.is_transmitting)
        self.timeout_policy_combo.blockSignals(False)

    def _set_options_enabled(self, enabled: bool) -> None:
        """Khóa/mở các tùy chọn cấu hình giao thức trong khi đang truyền file."""
        self.auto_connect_chk.setEnabled(enabled)
        self.boot_delay_chk.setEnabled(enabled)
        self.timeout_policy_combo.setEnabled(enabled and (not self._no_timeout_mode))
        self.error_policy_combo.setEnabled(enabled)
        self.ack_value_edit.setEnabled(enabled)
        self.queue_size_edit.setEnabled(enabled)
        self.inter_cmd_delay_spin.setEnabled(enabled)
        self.laser_type_combo.setEnabled(enabled)
        self.send_laser_btn.setEnabled(enabled)

    def _on_laser_type_changed(self, _index: int) -> None:
        current_cmd = self.laser_type_combo.currentData()
        if self._confirmed_laser_cmd == current_cmd and current_cmd:
            self._laser_ack_received = True
            self._waiting_laser_ack = False
            ack_val = self.ack_value_edit.text().strip() or "ok"
            self.laser_status_lbl.setText(
                f"● Đã xác nhận {current_cmd} (ACK: '{ack_val}')"
            )
            self.laser_status_lbl.setStyleSheet("color: #198754; font-weight: bold;")
        else:
            self._laser_ack_received = False
            self._waiting_laser_ack = False
            self.laser_status_lbl.setText(
                f"● Đã đổi sang {self.laser_type_combo.currentText()} ({current_cmd}) — Chưa gửi yêu cầu"
            )
            self.laser_status_lbl.setStyleSheet("color: #6c757d; font-weight: bold;")

    def sync_laser_from_config(
        self, mode_str: str, duty_str: str, freq_str: str
    ) -> None:
        """Tự động đồng bộ loại xung Laze và thông số từ Tab 1 (Config) sau khi đã Đọc ($0)."""
        target_cmd = "Laze2" if str(mode_str).strip() == "2" else "Laze1"
        idx = self.laser_type_combo.findData(target_cmd)

        self.laser_type_combo.blockSignals(True)
        if idx >= 0:
            self.laser_type_combo.setCurrentIndex(idx)
        self.laser_type_combo.blockSignals(False)

        self._waiting_laser_ack = False
        self._laser_ack_received = True
        self._pending_laser_cmd = target_cmd
        self._confirmed_laser_cmd = target_cmd

        mode_label = "Loại 2 (PWM+PFM)" if target_cmd == "Laze2" else "Loại 1 (PFM)"
        self.laser_status_lbl.setText(
            f"● Đã đồng bộ từ Tab Config: {mode_label} ({target_cmd} — Duty: {duty_str}%, Freq: {freq_str} Hz)"
        )
        self.laser_status_lbl.setStyleSheet("color: #198754; font-weight: bold;")
        self._append_log(
            "INFO",
            f"Đã tự động lấy cấu hình Laze từ Tab 1 ($0): {target_cmd} (Duty={duty_str}%, Freq={freq_str}Hz).",
        )

    def _on_send_laser_request(self) -> None:
        if self.is_transmitting:
            QMessageBox.warning(
                self,
                "Đang truyền file",
                "Không thể gửi yêu cầu chọn xung laze khi đang truyền file .tap.",
            )
            return

        if not self.serial_mgr.is_connected:
            if self.auto_connect_chk.isChecked():
                self.request_auto_connect.emit()
            if not self.serial_mgr.is_connected:
                QMessageBox.warning(
                    self,
                    "Chưa kết nối UART",
                    "Vui lòng kết nối cổng COM trước khi gửi yêu cầu xung laze.",
                )
                return

        laser_cmd = self.laser_type_combo.currentData() or "Laze1"
        payload = (laser_cmd + "\n").encode("ascii", errors="replace")

        self._laser_ack_received = False
        self._confirmed_laser_cmd = ""
        self._pending_laser_cmd = laser_cmd
        self._waiting_laser_ack = True

        if not self.serial_mgr.write_bytes(payload, source="TAP"):
            self._waiting_laser_ack = False
            self.laser_status_lbl.setText(f"● Lỗi gửi yêu cầu {laser_cmd}")
            self.laser_status_lbl.setStyleSheet("color: #dc3545; font-weight: bold;")
            self._append_log("ERR", f"Không thể gửi yêu cầu xung laze: {laser_cmd}")
            return

        ack_val = self.ack_value_edit.text().strip() or "ok"
        self.laser_status_lbl.setText(
            f"● Đã gửi '{laser_cmd}', đang chờ phản hồi ACK ('{ack_val}')..."
        )
        self.laser_status_lbl.setStyleSheet("color: #fd7e14; font-weight: bold;")
        self._append_log(
            "TX",
            f"Gửi yêu cầu chọn xung laze: {laser_cmd} (Chờ ACK: '{ack_val}')",
        )

    def _on_serial_connection_changed(self, is_connected: bool, _msg: str) -> None:
        if not is_connected:
            self._waiting_laser_ack = False
            self._laser_ack_received = False
            self._pending_laser_cmd = ""
            self._confirmed_laser_cmd = ""
            self.laser_status_lbl.setText("● Chưa chọn / gửi yêu cầu xung laze")
            self.laser_status_lbl.setStyleSheet("color: #6c757d; font-weight: bold;")

    def _on_browse_file(self) -> None:
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Chọn file G-code / TAP",
            self._last_browse_dir,
            "TAP / G-code Files (*.tap *.gcode *.nc *.ngc *.txt);;All Files (*)",
        )
        if not file_path:
            return
        self._last_browse_dir = os.path.dirname(file_path)
        if self.settings:
            self.settings.set_last_tap_dir(self._last_browse_dir)
        self.load_tap_file(file_path)

    def load_tap_file(self, file_path: str) -> bool:
        try:
            total_lines, commands = parse_tap_file(file_path)
        except Exception as exc:
            QMessageBox.critical(
                self, "Lỗi đọc file", f"Không thể mở file '{file_path}':\n{exc}"
            )
            self.file_info_label.setText(f"Lỗi đọc file: {exc}")
            self.file_info_label.setStyleSheet("color: #dc3545; font-weight: bold;")
            self.preview_btn.setEnabled(False)
            return False

        self.current_file_path = file_path
        self.total_file_lines = total_lines
        self.parsed_commands = commands

        self.file_path_edit.setText(file_path)
        self.file_info_label.setText(
            f"Tên file: {os.path.basename(file_path)} | "
            f"Tổng số dòng: {total_lines} | "
            f"Số lệnh hợp lệ sau khi lọc comment: {len(commands)}"
        )
        self.file_info_label.setStyleSheet("color: #198754; font-weight: bold;")
        self.preview_btn.setEnabled(True)

        self.progress_bar.setMaximum(max(1, len(commands)))
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat(f"0% (0/{len(commands)} lệnh)")
        self.ack_count_lbl.setText(f"0 / {len(commands)}")
        self._append_log(
            "INFO",
            f"Đã nạp file '{os.path.basename(file_path)}': {total_lines} dòng, {len(commands)} lệnh hợp lệ.",
        )
        return True

    def _on_preview_commands(self) -> None:
        if not self.current_file_path:
            return
        dlg = TapPreviewDialog(
            self.current_file_path,
            self.total_file_lines,
            self.parsed_commands,
            self,
        )
        dlg.exec()

    def _on_start_sending(self) -> None:
        if self.is_transmitting:
            return

        if not self.current_file_path or not self.parsed_commands:
            QMessageBox.warning(
                self,
                "Chưa có lệnh để gửi",
                "Vui lòng chọn một file .tap hợp lệ trước khi bắt đầu truyền.",
            )
            return

        if not self.load_tap_file(self.current_file_path):
            return

        if not self.serial_mgr.is_connected:
            if self.auto_connect_chk.isChecked():
                self._set_status("Connecting")
                self.request_auto_connect.emit()
            if not self.serial_mgr.is_connected:
                QMessageBox.warning(
                    self,
                    "Chưa kết nối UART",
                    "Vui lòng kết nối cổng COM trên thanh công cụ trước khi truyền file.",
                )
                self._set_status("Idle")
                return

        # Kiểm tra xem đã chọn và nhận được phản hồi ACK cho loại xung laze hay chưa
        if not self._laser_ack_received:
            ack_val = self.ack_value_edit.text().strip() or "ok"
            if self._waiting_laser_ack:
                warn_text = (
                    f"Đã gửi yêu cầu xung laze ({self._pending_laser_cmd}) nhưng CHƯA nhận được "
                    f"phản hồi xác nhận (ACK: '{ack_val}').\n\n"
                    "Bạn có muốn tiếp tục gửi file khi chưa chọn / xác nhận xung laze hay không?"
                )
            else:
                warn_text = (
                    "Bạn chưa gửi yêu cầu chọn loại xung laze (hoặc chưa nhận phản hồi xác nhận ACK).\n\n"
                    "Bạn có muốn gửi file khi chưa chọn xung laze hay không?"
                )

            ans = QMessageBox.question(
                self,
                "Chưa xác nhận xung laze",
                warn_text,
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if ans != QMessageBox.Yes:
                self._set_status("Idle")
                return

            # Hủy trạng thái chờ ACK xung laze để không bắt nhầm ACK của lệnh G-code đầu tiên
            self._waiting_laser_ack = False
            self._append_log(
                "WARN",
                "Người dùng chọn tiếp tục truyền file .tap khi chưa xác nhận xung laze.",
            )

        timeout_policy_idx = (
            self._saved_timeout_policy_idx
            if self._no_timeout_mode
            else self.timeout_policy_combo.currentIndex()
        )
        wait_on_timeout = timeout_policy_idx == 0
        stop_on_timeout = timeout_policy_idx == 1
        stop_on_error = self.error_policy_combo.currentIndex() == 0
        boot_delay = 2.0 if self.boot_delay_chk.isChecked() else 0.0
        ack_value = self.ack_value_edit.text().strip() or "ok"
        queue_size = self._get_queue_size_value()
        inter_cmd_delay_ms = self.inter_cmd_delay_spin.value()

        self.worker = TapSenderWorker(
            serial_mgr=self.serial_mgr,
            commands=self.parsed_commands,
            total_file_lines=self.total_file_lines,
            command_timeout=self.serial_mgr.current_timeout,
            boot_delay=boot_delay,
            stop_on_timeout=stop_on_timeout,
            stop_on_error=stop_on_error,
            strict_ack=True,
            ack_value=ack_value,
            wait_on_timeout=wait_on_timeout,
            infinite_timeout=bool(getattr(self.serial_mgr, "no_timeout", False)),
            queue_size=queue_size,
            inter_cmd_delay_ms=inter_cmd_delay_ms,
        )

        self.serial_mgr.line_received.connect(self.worker.on_line_received)
        self.worker.status_changed.connect(self._set_status)
        self.worker.progress_updated.connect(self._on_progress_updated)
        self.worker.log_event.connect(self._append_log)
        self.worker.command_result.connect(self._on_command_result)
        self.worker.transmission_finished.connect(self._on_transmission_finished)

        self._records.clear()
        self._start_timestamp = time.monotonic()
        self._elapsed_timer.start()

        self.start_btn.setEnabled(False)
        self.browse_btn.setEnabled(False)
        self.pause_btn.setEnabled(True)
        self.pause_btn.setText("⏸ Tạm dừng (Pause)")
        self.stop_btn.setEnabled(True)
        self._set_options_enabled(False)
        self.transmission_state_changed.emit(True)

        self.worker.start()

    def _on_toggle_pause(self) -> None:
        if not self.worker or not self.worker.isRunning():
            return
        is_currently_paused = self.status_value_lbl.text() == "Paused"
        will_pause = not is_currently_paused
        self.worker.request_pause(will_pause)
        if will_pause:
            self.pause_btn.setText("▶ Tiếp tục (Resume)")
            # Mở khóa Tab 2 (UART Terminal) để cho phép gửi lệnh thủ công khi đang tạm dừng
            self.transmission_state_changed.emit(False)
        else:
            self.pause_btn.setText("⏸ Tạm dừng (Pause)")
            # Khóa lại Tab 2 khi tiếp tục truyền file .tap
            self.transmission_state_changed.emit(True)

    def _on_stop_sending(self) -> None:
        if not self.is_transmitting or not self.worker:
            return
        ans = QMessageBox.question(
            self,
            "Xác nhận dừng truyền",
            "Bạn có chắc chắn muốn dừng quá trình truyền file .tap hiện tại không?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if ans == QMessageBox.Yes:
            self.worker.request_stop()

    def force_stop_worker(self) -> None:
        if self.worker and self.worker.isRunning():
            self.worker.request_stop()
            self.worker.wait(1500)

    def _on_clear_log(self) -> None:
        if (
            self.log_edit.document().isEmpty()
            and self.rx_signal_log.document().isEmpty()
            and not self._records
        ):
            return
        ans = QMessageBox.question(
            self,
            "Xóa nhật ký",
            "Bạn có muốn xóa toàn bộ nhật ký truyền file, bộ nhớ xuất log và tín hiệu nhận không?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if ans == QMessageBox.Yes:
            self.clear_all_logs()

    def clear_all_logs(self) -> None:
        """Xóa sạch cả khung hiển thị lẫn danh sách bản ghi phục vụ xuất file."""
        self.log_edit.clear()
        self.rx_signal_log.clear()
        self.last_rx_value_edit.clear()
        self._rx_signal_count = 0
        self.rx_signal_count_lbl.setText("0")
        self._records.clear()

    def _set_status(self, status: str) -> None:
        self.status_value_lbl.setText(status)
        color_map = {
            "Idle": "#6c757d",
            "Connecting": "#0dcaf0",
            "Sending": "#0d6efd",
            "Waiting for Response": "#fd7e14",
            "Paused": "#d97706",
            "Completed": "#198754",
            "Stopped": "#dc3545",
            "Error": "#dc3545",
        }
        color = color_map.get(status, "#212529")
        self.status_value_lbl.setStyleSheet(f"font-weight: bold; color: {color};")

    def _on_progress_updated(
        self, ack_count: int, current_idx: int, total_cmds: int, cmd_text: str
    ) -> None:
        self.progress_bar.setMaximum(max(1, total_cmds))
        self.progress_bar.setValue(ack_count)
        pct = int((ack_count / total_cmds) * 100) if total_cmds else 0
        self.progress_bar.setFormat(f"{pct}% ({ack_count}/{total_cmds} lệnh)")
        self.ack_count_lbl.setText(f"{ack_count} / {total_cmds}")
        self.current_cmd_num_lbl.setText(f"{current_idx} / {total_cmds}")
        self.current_cmd_text_lbl.setText(cmd_text)

    def _on_ack_value_changed(self, text: str) -> None:
        clean_ack = text.strip() or "ok"
        if self.settings:
            self.settings.set_ack_value(clean_ack)

    def _get_queue_size_value(self) -> int:
        text = self.queue_size_edit.text().strip()
        try:
            val = int(text)
        except ValueError:
            val = 1
        return max(1, min(1000, val))

    def _normalize_queue_size_input(self) -> None:
        val = self._get_queue_size_value()
        if self.queue_size_edit.text().strip() != str(val):
            self.queue_size_edit.setText(str(val))

    def _on_queue_size_changed(self, text: str) -> None:
        clean = text.strip()
        if not clean:
            return
        try:
            val = max(1, min(1000, int(clean)))
        except ValueError:
            return
        if self.settings and hasattr(self.settings, "set_queue_size"):
            self.settings.set_queue_size(val)

    def _on_inter_cmd_delay_changed(self, value: int) -> None:
        if self.settings and hasattr(self.settings, "set_inter_cmd_delay_ms"):
            self.settings.set_inter_cmd_delay_ms(value)

    def _on_command_result(
        self,
        line_number: int,
        command: str,
        sent_ok: bool,
        ack_response: str,
        status: str,
    ) -> None:
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._records.append(
            (ts, str(line_number), command, "Yes" if sent_ok else "No", ack_response, status)
        )

    def _on_export_log(self) -> None:
        if not self._records:
            QMessageBox.information(
                self,
                "Không có dữ liệu",
                "Chưa có dữ liệu truyền lệnh nào để xuất. Hãy chạy truyền file trước.",
            )
            return

        file_path, selected_filter = QFileDialog.getSaveFileName(
            self,
            "Xuất nhật ký truyền TAP",
            "tap_transmission_log.csv",
            "CSV Files (*.csv);;Text Files (*.txt)",
        )
        if not file_path:
            return

        is_csv = file_path.lower().endswith(".csv") or "CSV" in selected_filter
        columns = [
            "Timestamp",
            "Line Number",
            "Command",
            "Sent",
            "Ack Received",
            "Status",
        ]
        rows = [list(r) for r in self._records]

        self.export_log_btn.setEnabled(False)
        self._export_worker = LogExportWorker(
            file_path=file_path,
            fmt="csv" if is_csv else "txt",
            columns=columns,
            rows=rows,
        )
        self._export_worker.export_finished.connect(self._on_export_finished)
        self._export_worker.start()

    def _on_export_finished(self, success: bool, message: str) -> None:
        self.export_log_btn.setEnabled(True)
        if success:
            QMessageBox.information(self, "Xuất log thành công", message)
        else:
            QMessageBox.warning(self, "Xuất log thất bại", message)

    def _on_transmission_finished(self, success: bool, summary: str) -> None:
        self._elapsed_timer.stop()
        self._update_elapsed_time()

        if self.worker:
            try:
                self.serial_mgr.line_received.disconnect(self.worker.on_line_received)
            except Exception:
                pass

        self.start_btn.setEnabled(True)
        self.browse_btn.setEnabled(True)
        self.pause_btn.setEnabled(False)
        self.pause_btn.setText("⏸ Tạm dừng (Pause)")
        self.stop_btn.setEnabled(False)
        self._set_options_enabled(True)
        self.transmission_state_changed.emit(False)

        level = "OK" if success else "ERR"
        self._append_log(level, summary)

    def _update_elapsed_time(self) -> None:
        if self._start_timestamp is None:
            return
        elapsed = max(0.0, time.monotonic() - self._start_timestamp)
        mins = int(elapsed // 60)
        secs = elapsed % 60
        self.elapsed_time_lbl.setText(f"{mins:02d}:{secs:04.1f}")

    def _append_log(self, level: str, message: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        color_map = {
            "TX": "#0d6efd",
            "RX": "#495057",
            "OK": "#198754",
            "WARN": "#d97706",
            "ERR": "#dc3545",
            "INFO": "#6f42c1",
        }
        color = color_map.get(level, "#212529")

        cursor = self.log_edit.textCursor()
        cursor.movePosition(QTextCursor.End)

        fmt = QTextCharFormat()
        fmt.setForeground(QColor(color))
        cursor.insertText(f"[{ts}] [{level:4s}] {message}\n", fmt)

        self.log_edit.setTextCursor(cursor)
        self.log_edit.ensureCursorVisible()

    def _on_external_signal_received(self, line: str) -> None:
        """Hiển thị mọi dòng tín hiệu mà thiết bị UART ngoài gửi tới, độc lập với trạng thái truyền file."""
        self._rx_signal_count += 1
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]

        self.last_rx_value_edit.setText(line)
        self.rx_signal_count_lbl.setText(str(self._rx_signal_count))

        cursor = self.rx_signal_log.textCursor()
        cursor.movePosition(QTextCursor.End)
        cursor.insertText(f"[{ts}] {line}\n")
        self.rx_signal_log.setTextCursor(cursor)
        self.rx_signal_log.ensureCursorVisible()

        # Kiểm tra phản hồi ACK nếu đang chờ xác nhận lệnh chọn loại xung laze
        if self._waiting_laser_ack and not self.is_transmitting:
            ack_val = self.ack_value_edit.text().strip() or "ok"
            category = classify_response(line, ack_value=ack_val, strict_ack=True)
            if category == "ok":
                self._waiting_laser_ack = False
                self._laser_ack_received = True
                self._confirmed_laser_cmd = self._pending_laser_cmd
                self.laser_status_lbl.setText(
                    f"● Đã xác nhận {self._confirmed_laser_cmd} (ACK: '{line}')"
                )
                self.laser_status_lbl.setStyleSheet(
                    "color: #198754; font-weight: bold;"
                )
                self._append_log(
                    "OK",
                    f"Thiết bị đã xác nhận xung laze '{self._confirmed_laser_cmd}' (Phản hồi: {line}). Sẵn sàng gửi file.",
                )
            elif category in ("error", "alarm"):
                self._waiting_laser_ack = False
                self._laser_ack_received = False
                self.laser_status_lbl.setText(
                    f"● Thiết bị báo lỗi khi chọn {self._pending_laser_cmd}: {line}"
                )
                self.laser_status_lbl.setStyleSheet(
                    "color: #dc3545; font-weight: bold;"
                )
                self._append_log(
                    "ERR",
                    f"Thiết bị từ chối/báo lỗi khi chọn xung laze '{self._pending_laser_cmd}': {line}",
                )