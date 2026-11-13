import time
from collections import deque
from datetime import datetime
from typing import Deque, List, Optional, Tuple

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor, QFont, QKeyEvent, QTextCharFormat, QTextCursor
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from log_export import LogExportWorker
from serial_manager import SerialManager

# Thời gian (giây) không có dữ liệu mới trước khi buffer dở dang được xả ra màn hình
RX_IDLE_FLUSH_SEC = 0.07
RX_FLUSH_TIMER_INTERVAL_MS = 40
MAX_TERMINAL_RECORDS = 50000


def format_terminal_payload(
    user_input: str,
    line_ending_mode: str = "LF",
    auto_lf_enabled: bool = True,
    is_hex_mode: bool = False,
) -> bytes:
    if user_input == "":
        raise ValueError("Nội dung lệnh trống.")

    effective_mode = line_ending_mode.upper()
    if auto_lf_enabled and effective_mode == "NONE":
        effective_mode = "LF"
    elif not auto_lf_enabled and effective_mode == "LF":
        effective_mode = "NONE"

    ending_bytes = {
        "NONE": b"",
        "LF": b"\n",
        "CR": b"\r",
        "CRLF": b"\r\n",
    }.get(effective_mode, b"")

    if is_hex_mode:
        cleaned_hex = (
            user_input.replace("0x", "")
            .replace("0X", "")
            .replace(",", " ")
            .replace("-", " ")
        )
        raw_bytes = bytes.fromhex(cleaned_hex)
    else:
        raw_bytes = user_input.encode("utf-8")

    if ending_bytes and not raw_bytes.endswith(ending_bytes):
        if effective_mode == "LF" and raw_bytes.endswith(b"\r\n"):
            pass
        else:
            raw_bytes += ending_bytes

    return raw_bytes


def format_bytes_as_hex(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


class HistoryLineEdit(QLineEdit):
    """Ô nhập lệnh hỗ trợ phím mũi tên Lên/Xuống để gọi lại các lệnh đã gửi."""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._history: List[str] = []
        self._history_index: int = 0

    def add_history(self, cmd: str) -> None:
        if not cmd:
            return
        if not self._history or self._history[-1] != cmd:
            self._history.append(cmd)
        self._history_index = len(self._history)

    def keyPressEvent(self, event: QKeyEvent) -> None:
        if event.key() == Qt.Key_Up:
            if self._history and self._history_index > 0:
                self._history_index -= 1
                self.setText(self._history[self._history_index])
            event.accept()
            return
        if event.key() == Qt.Key_Down:
            if self._history:
                if self._history_index < len(self._history) - 1:
                    self._history_index += 1
                    self.setText(self._history[self._history_index])
                else:
                    self._history_index = len(self._history)
                    self.clear()
            event.accept()
            return
        super().keyPressEvent(event)


class UartTerminalTab(QWidget):
    """Tab 2: Giao diện UART Terminal tương tự Hercules."""

    def __init__(
        self,
        serial_mgr: SerialManager,
        settings=None,
        parent: Optional[QWidget] = None,
    ):
        super().__init__(parent)
        self.serial_mgr = serial_mgr
        self.settings = settings
        self._rx_partial_buffer = ""
        self._rx_last_data_time: Optional[float] = None
        self._tap_busy: bool = False
        self._records: Deque[Tuple[str, str, str, str]] = deque(
            maxlen=MAX_TERMINAL_RECORDS
        )
        self._export_worker: Optional[LogExportWorker] = None
        self._build_ui()

        if settings:
            self.hex_view_chk.setChecked(settings.terminal_hex_mode())
            self.auto_scroll_chk.setChecked(settings.terminal_auto_scroll())
            saved_append_lf = settings.terminal_append_lf()
            self.append_lf_chk.setChecked(saved_append_lf)
            if not saved_append_lf:
                self.line_ending_combo.setCurrentIndex(
                    settings.terminal_line_ending_index()
                )
        self.hex_view_chk.toggled.connect(self._save_hex_mode_setting)
        self.auto_scroll_chk.toggled.connect(self._save_auto_scroll_setting)
        self.append_lf_chk.toggled.connect(self._save_append_lf_setting)
        self.line_ending_combo.currentIndexChanged.connect(
            self._save_line_ending_setting
        )

        # Timer xả buffer dở dang khi không có dữ liệu mới trong ~40-70ms
        self._rx_flush_timer = QTimer(self)
        self._rx_flush_timer.setInterval(RX_FLUSH_TIMER_INTERVAL_MS)
        self._rx_flush_timer.timeout.connect(self._flush_idle_partial_buffer)
        self._rx_flush_timer.start()

        self.serial_mgr.raw_data_received.connect(self._on_raw_data_received)
        self.serial_mgr.data_sent.connect(self._on_data_sent)
        self.serial_mgr.connection_changed.connect(self._on_connection_changed)

    def shutdown(self) -> None:
        """Dừng timer và xả nốt dữ liệu dở dang khi ứng dụng đóng."""
        if self._rx_flush_timer.isActive():
            self._rx_flush_timer.stop()
        self._flush_idle_partial_buffer(force=True)

    def set_tap_transmission_active(self, active: bool) -> None:
        """Khóa gửi lệnh thủ công khi Tab 1 đang truyền file .TAP để tránh tranh chấp ACK."""
        self._tap_busy = active
        self.input_edit.setEnabled(not active)
        self.send_btn.setEnabled(not active)
        if active:
            self.input_edit.setPlaceholderText(
                "Đang truyền file .TAP ở Tab 1 — Tạm khóa gửi lệnh thủ công để tránh xung đột phản hồi ACK..."
            )
        else:
            self.input_edit.setPlaceholderText(
                "Nhập lệnh (ví dụ: $$, $X, G0 X10 Y10) rồi nhấn Enter hoặc bấm Gửi... (Dùng phím ↑/↓ để xem lịch sử)"
            )

    def _save_hex_mode_setting(self, checked: bool) -> None:
        if self.settings:
            self.settings.set_terminal_hex_mode(checked)

    def _save_auto_scroll_setting(self, checked: bool) -> None:
        if self.settings:
            self.settings.set_terminal_auto_scroll(checked)

    def _save_append_lf_setting(self, checked: bool) -> None:
        if self.settings:
            self.settings.set_terminal_append_lf(checked)

    def _save_line_ending_setting(self, index: int) -> None:
        if self.settings:
            self.settings.set_terminal_line_ending_index(index)

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        # 1. Thanh tùy chọn hiển thị của Terminal (Chia lưới 2 hàng x 3 cột để không bao giờ bị cắt chữ)
        view_group = QGroupBox("Tùy chọn Hiển thị Terminal (Hercules Style)")
        view_grid = QGridLayout(view_group)
        view_grid.setContentsMargins(12, 12, 12, 10)
        view_grid.setHorizontalSpacing(24)
        view_grid.setVerticalSpacing(8)

        self.auto_scroll_chk = QCheckBox("Tự động cuộn (Auto-scroll)")
        self.auto_scroll_chk.setMinimumHeight(24)
        self.auto_scroll_chk.setChecked(True)

        self.timestamp_chk = QCheckBox("Hiển thị Timestamp")
        self.timestamp_chk.setMinimumHeight(24)
        self.timestamp_chk.setChecked(True)

        self.hex_view_chk = QCheckBox("Hiển thị mã HEX (Hex View)")
        self.hex_view_chk.setMinimumHeight(24)
        self.hex_view_chk.setChecked(False)

        self.show_tap_traffic_chk = QCheckBox("Hiển thị cả dữ liệu khi gửi file .TAP")
        self.show_tap_traffic_chk.setMinimumHeight(24)
        self.show_tap_traffic_chk.setChecked(False)

        self.export_log_btn = QPushButton("Xuất Log (Export)")
        self.export_log_btn.setMinimumHeight(28)
        self.export_log_btn.setMinimumWidth(160)
        self.export_log_btn.clicked.connect(self._on_export_log)

        self.clear_btn = QPushButton("Xóa màn hình (Clear)")
        self.clear_btn.setMinimumHeight(28)
        self.clear_btn.setMinimumWidth(160)
        self.clear_btn.clicked.connect(self.clear_terminal)

        # Hàng 0: 2 Checkbox đầu + Nút Xuất Log bên phải
        view_grid.addWidget(self.auto_scroll_chk, 0, 0)
        view_grid.addWidget(self.timestamp_chk, 0, 1)
        view_grid.addWidget(self.export_log_btn, 0, 2, alignment=Qt.AlignRight)

        # Hàng 1: 2 Checkbox sau + Nút Xóa màn hình bên phải
        view_grid.addWidget(self.hex_view_chk, 1, 0)
        view_grid.addWidget(self.show_tap_traffic_chk, 1, 1)
        view_grid.addWidget(self.clear_btn, 1, 2, alignment=Qt.AlignRight)

        view_grid.setColumnStretch(0, 1)
        view_grid.setColumnStretch(1, 1)
        layout.addWidget(view_group)

        # 2. Khung hiển thị dữ liệu nhận/gửi
        self.terminal_output = QPlainTextEdit()
        self.terminal_output.setReadOnly(True)
        self.terminal_output.setMaximumBlockCount(15000)
        font = QFont("Consolas", 10)
        font.setStyleHint(QFont.Monospace)
        self.terminal_output.setFont(font)
        self.terminal_output.setStyleSheet(
            "QPlainTextEdit { background-color: #1e1e1e; color: #d4d4d4; padding: 6px; border-radius: 4px; }"
        )
        layout.addWidget(self.terminal_output, stretch=1)

        # 3. Khu vực nhập lệnh và cấu hình hậu tố <LF> (Chia lưới 2 hàng rộng rãi)
        input_group = QGroupBox("Gửi Lệnh / Dữ liệu UART")
        input_layout = QVBoxLayout(input_group)
        input_layout.setContentsMargins(12, 12, 12, 10)
        input_layout.setSpacing(10)

        mode_grid = QGridLayout()
        mode_grid.setHorizontalSpacing(14)
        mode_grid.setVerticalSpacing(8)

        # Hàng 0: Chế độ gửi & Kiểu kết thúc dòng
        mode_grid.addWidget(QLabel("Chế độ gửi:"), 0, 0)
        self.input_mode_combo = QComboBox()
        self.input_mode_combo.setMinimumHeight(28)
        self.input_mode_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.input_mode_combo.addItems(
            ["ASCII / UTF-8 Text", "HEX Bytes (VD: 47 30 30 0A)"]
        )
        self.input_mode_combo.currentIndexChanged.connect(
            self._update_effective_mode_label
        )
        mode_grid.addWidget(self.input_mode_combo, 0, 1)

        mode_grid.addWidget(QLabel("Kiểu kết thúc dòng:"), 0, 2)
        self.line_ending_combo = QComboBox()
        self.line_ending_combo.setMinimumHeight(28)
        self.line_ending_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.line_ending_combo.addItems(
            [
                "LF (\\n — 0x0A)",
                "None (Không thêm)",
                "CR (\\r — 0x0D)",
                "CRLF (\\r\\n — 0x0D 0x0A)",
            ]
        )
        self.line_ending_combo.currentIndexChanged.connect(
            self._on_line_ending_combo_changed
        )
        mode_grid.addWidget(self.line_ending_combo, 0, 3)

        # Hàng 1: Hộp kiểm Append <LF> automatically & Nhãn Hậu tố hiệu lực
        self.append_lf_chk = QCheckBox("Append <LF> automatically (0x0A)")
        self.append_lf_chk.setMinimumHeight(28)
        self.append_lf_chk.setChecked(True)
        self.append_lf_chk.setToolTip(
            "Tự động nối ký tự Line Feed (\\n, 0x0A) vào cuối chuỗi gửi đi nếu chưa có."
        )
        self.append_lf_chk.toggled.connect(self._on_auto_lf_toggled)
        mode_grid.addWidget(self.append_lf_chk, 1, 0, 1, 2)

        self.effective_ending_lbl = QLabel("Hậu tố hiệu lực: LF (\\n, 0x0A)")
        self.effective_ending_lbl.setAlignment(Qt.AlignCenter)
        self.effective_ending_lbl.setMinimumHeight(28)
        self.effective_ending_lbl.setMinimumWidth(260)
        self.effective_ending_lbl.setStyleSheet(
            "font-weight: bold; color: #0d6efd; background: #e7f1ff; "
            "padding: 3px 12px; border: 1px solid #b6d4fe; border-radius: 4px;"
        )
        mode_grid.addWidget(self.effective_ending_lbl, 1, 2, 1, 2)

        mode_grid.setColumnStretch(1, 1)
        mode_grid.setColumnStretch(3, 1)
        input_layout.addLayout(mode_grid)

        # Dòng nhập lệnh và nút Gửi
        send_row = QHBoxLayout()
        send_row.setSpacing(10)

        self.input_edit = HistoryLineEdit()
        self.input_edit.setMinimumHeight(32)
        self.input_edit.setPlaceholderText(
            "Nhập lệnh (ví dụ: $$, $X, G0 X10 Y10) rồi nhấn Enter hoặc bấm Gửi... (Dùng phím ↑/↓ để xem lịch sử)"
        )
        self.input_edit.returnPressed.connect(self.send_current_input)

        self.send_btn = QPushButton("Gửi (Send)")
        self.send_btn.setMinimumHeight(32)
        self.send_btn.setMinimumWidth(140)
        self.send_btn.setStyleSheet(
            "QPushButton { background-color: #0d6efd; color: white; font-weight: bold; "
            "padding: 4px 18px; border-radius: 4px; }"
            "QPushButton:disabled { background-color: #9ec5fe; color: #ffffff; }"
        )
        self.send_btn.clicked.connect(self.send_current_input)

        send_row.addWidget(self.input_edit, stretch=1)
        send_row.addWidget(self.send_btn)
        input_layout.addLayout(send_row)

        layout.addWidget(input_group)

    def _get_selected_ending_code(self) -> str:
        idx = self.line_ending_combo.currentIndex()
        return {0: "LF", 1: "NONE", 2: "CR", 3: "CRLF"}.get(idx, "LF")

    def _on_auto_lf_toggled(self, checked: bool) -> None:
        self.line_ending_combo.blockSignals(True)
        if checked:
            self.line_ending_combo.setCurrentIndex(0)
        else:
            if self.line_ending_combo.currentIndex() == 0:
                self.line_ending_combo.setCurrentIndex(1)
        self.line_ending_combo.blockSignals(False)
        self._update_effective_mode_label()

    def _on_line_ending_combo_changed(self, index: int) -> None:
        self.append_lf_chk.blockSignals(True)
        self.append_lf_chk.setChecked(index == 0)
        self.append_lf_chk.blockSignals(False)
        self._update_effective_mode_label()

    def _update_effective_mode_label(self) -> None:
        code = self._get_selected_ending_code()
        desc_map = {
            "NONE": "None (Gửi nguyên bản)",
            "LF": "LF (\\n, 0x0A)",
            "CR": "CR (\\r, 0x0D)",
            "CRLF": "CRLF (\\r\\n, 0x0D 0x0A)",
        }
        self.effective_ending_lbl.setText(f"Hậu tố hiệu lực: {desc_map[code]}")

    def send_current_input(self) -> None:
        if self._tap_busy:
            QMessageBox.information(
                self,
                "Đang truyền file .TAP",
                "Vui lòng đợi quá trình truyền file .TAP hoàn tất hoặc nhấn Dừng trước khi gửi lệnh từ Terminal.",
            )
            return

        text = self.input_edit.text()
        if text == "":
            return

        if not self.serial_mgr.is_connected:
            QMessageBox.warning(
                self,
                "Chưa kết nối UART",
                "Vui lòng kết nối cổng COM trước khi gửi lệnh từ Terminal.",
            )
            return

        is_hex = self.input_mode_combo.currentIndex() == 1
        ending_code = self._get_selected_ending_code()
        auto_lf = self.append_lf_chk.isChecked()

        try:
            payload = format_terminal_payload(
                user_input=text,
                line_ending_mode=ending_code,
                auto_lf_enabled=auto_lf,
                is_hex_mode=is_hex,
            )
        except ValueError as exc:
            QMessageBox.warning(
                self,
                "Dữ liệu đầu vào không hợp lệ",
                f"Không thể mã hóa dữ liệu: {exc}",
            )
            return

        if self.serial_mgr.write_bytes(payload, source="TERM"):
            self.input_edit.add_history(text)
            self.input_edit.clear()

    def _on_data_sent(self, data: bytes, source: str) -> None:
        if source == "TAP" and not self.show_tap_traffic_chk.isChecked():
            return
        display_text = data.decode("latin-1", errors="replace")
        is_hex = self.hex_view_chk.isChecked()
        readable = display_text.replace("\r", "<CR>").replace("\n", "<LF>\n")
        if is_hex:
            readable = f"{readable.strip()}  [HEX: {format_bytes_as_hex(data)}]\n"
        elif not readable.endswith("\n"):
            readable += "\n"

        self._append_terminal_text(f"TX >> {readable}", "#4dabf7")
        self._add_record("TX", display_text, "HEX" if is_hex else "ASCII")

    def _on_raw_data_received(self, chunk: bytes) -> None:
        self._rx_last_data_time = time.monotonic()

        if self.hex_view_chk.isChecked():
            text_repr = (
                chunk.decode("latin-1", errors="replace")
                .replace("\r", "<CR>")
                .replace("\n", "<LF>")
            )
            hex_repr = format_bytes_as_hex(chunk)
            self._append_terminal_text(
                f"RX << {text_repr}  [HEX: {hex_repr}]\n", "#69db7c"
            )
            self._add_record("RX", text_repr, "HEX")
            return

        decoded = chunk.decode("latin-1", errors="replace")
        self._rx_partial_buffer += decoded
        self._flush_complete_lines()

    def _flush_complete_lines(self) -> None:
        """Tách và hiển thị ngay các dòng đã hoàn chỉnh (kết thúc bằng CR, LF hoặc CRLF)."""
        buf = self._rx_partial_buffer
        while True:
            idx_n = buf.find("\n")
            idx_r = buf.find("\r")
            if idx_n == -1 and idx_r == -1:
                break

            if idx_r != -1 and (idx_n == -1 or idx_r < idx_n):
                if idx_n == idx_r + 1:
                    line = buf[:idx_r]
                    buf = buf[idx_n + 1 :]
                else:
                    line = buf[:idx_r]
                    buf = buf[idx_r + 1 :]
            else:
                line = buf[:idx_n]
                buf = buf[idx_n + 1 :]

            self._append_terminal_text(f"RX << {line}\n", "#69db7c")
            self._add_record("RX", line, "ASCII")

        self._rx_partial_buffer = buf

    def _flush_idle_partial_buffer(self, force: bool = False) -> None:
        """Xả phần dữ liệu dở dang (không có CR/LF) nếu không có byte mới trong ~70ms hoặc khi force=True."""
        if not self._rx_partial_buffer:
            return
        if not force:
            if self._rx_last_data_time is None:
                return
            if (time.monotonic() - self._rx_last_data_time) < RX_IDLE_FLUSH_SEC:
                return

        pending = self._rx_partial_buffer
        self._rx_partial_buffer = ""
        self._append_terminal_text(f"RX << {pending}\n", "#69db7c")
        self._add_record("RX", pending, "ASCII")

    def _on_connection_changed(self, is_connected: bool, _message: str) -> None:
        if not is_connected:
            self._flush_idle_partial_buffer(force=True)
            self._rx_partial_buffer = ""
            self._rx_last_data_time = None

    def _add_record(self, direction: str, data_text: str, mode: str) -> None:
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._records.append((ts, direction, data_text.strip("\r\n"), mode))

    def _on_export_log(self) -> None:
        if not self._records:
            QMessageBox.information(
                self,
                "Không có dữ liệu",
                "Chưa có dữ liệu Terminal nào để xuất.",
            )
            return

        file_path, selected_filter = QFileDialog.getSaveFileName(
            self,
            "Xuất nhật ký UART Terminal",
            "terminal_log.csv",
            "CSV Files (*.csv);;Text Files (*.txt)",
        )
        if not file_path:
            return

        is_csv = file_path.lower().endswith(".csv") or "CSV" in selected_filter
        columns = ["Timestamp", "Direction", "Data", "Mode"]
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

    def _append_terminal_text(self, text: str, hex_color: str) -> None:
        cursor = self.terminal_output.textCursor()
        cursor.movePosition(QTextCursor.End)

        prefix = ""
        if self.timestamp_chk.isChecked():
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            prefix = f"[{ts}] "

        fmt = QTextCharFormat()
        fmt.setForeground(QColor(hex_color))
        cursor.insertText(f"{prefix}{text}", fmt)

        if self.auto_scroll_chk.isChecked():
            self.terminal_output.setTextCursor(cursor)
            self.terminal_output.ensureCursorVisible()

    def clear_terminal(self) -> None:
        self._rx_partial_buffer = ""
        self._records.clear()
        self.terminal_output.clear()