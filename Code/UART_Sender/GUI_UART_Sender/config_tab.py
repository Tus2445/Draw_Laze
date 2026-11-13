import re
from typing import Dict, List, Optional, Tuple

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QDoubleValidator, QIntValidator
from PySide6.QtWidgets import (
    QComboBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from serial_manager import SerialManager

_DOLLAR_LINE_RE = re.compile(r"^\$(\d+)\s*=\s*([-+]?\d+(?:\.\d+)?)")
_COMMAND_TIMEOUT_MS = 2500


class ConfigTab(QWidget):
    """Tab 1: Config — Đọc ($0), Ghi ($<id>=<val>) và Khôi phục mặc định ($RST) cấu hình STM32 qua UART."""

    def __init__(
        self,
        serial_mgr: SerialManager,
        settings=None,
        parent: Optional[QWidget] = None,
    ):
        super().__init__(parent)
        self.serial_mgr = serial_mgr
        self.settings = settings

        self._tap_busy: bool = False
        self._mode: str = "IDLE"  # IDLE | READING | WRITING | RESETTING
        self._write_queue: List[Tuple[int, str]] = []
        self._current_write: Optional[Tuple[int, str]] = None
        self._write_applied_count: int = 0
        self._auto_refresh_after_op: bool = False
        self._device_values: Dict[int, str] = {}

        # Lưu widget điều khiển cho từng mã $id
        self._inputs: Dict[int, QWidget] = {}
        self._speed_hint_labels: Dict[int, QLabel] = {}

        self._cmd_timeout_timer = QTimer(self)
        self._cmd_timeout_timer.setSingleShot(True)
        self._cmd_timeout_timer.setInterval(_COMMAND_TIMEOUT_MS)
        self._cmd_timeout_timer.timeout.connect(self._on_command_timeout)

        self._build_ui()
        if settings and hasattr(settings, "config_ack_value"):
            self.ack_value_edit.setText(settings.config_ack_value("OK"))
        self.ack_value_edit.textChanged.connect(self._on_ack_value_changed)
        self._populate_default_placeholders()
        self._update_controls_enabled()

        self.serial_mgr.line_received.connect(self._on_line_received)
        self.serial_mgr.connection_changed.connect(self._on_connection_changed)

    def shutdown(self) -> None:
        if self._cmd_timeout_timer.isActive():
            self._cmd_timeout_timer.stop()
        self._mode = "IDLE"
        self._write_queue.clear()

    def set_tap_transmission_active(self, active: bool) -> None:
        """Khóa thao tác đọc/ghi cấu hình khi đang truyền file .TAP ở Tab 2."""
        self._tap_busy = active
        self._update_controls_enabled()
        if active:
            self._set_status(
                "Đang truyền file .TAP ở Tab 2 — Tạm khóa đọc/ghi cấu hình để tránh xung đột UART.",
                "#d97706",
            )
        elif self._mode == "IDLE":
            if self.serial_mgr.is_connected:
                self._set_status("Sẵn sàng đọc/ghi cấu hình STM32.", "#0f5132")
            else:
                self._set_status("Chưa kết nối cổng UART.", "#6c757d")

    def _build_ui(self) -> None:
        outer_layout = QVBoxLayout(self)
        outer_layout.setContentsMargins(0, 0, 0, 0)

        scroll_area = QScrollArea(self)
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QFrame.NoFrame)

        content = QWidget()
        main_layout = QVBoxLayout(content)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)

        # --- Thanh điều khiển thao tác cấu hình ---
        action_group = QGroupBox("Điều khiển Cấu hình Phần cứng STM32 (UART: $0 | $<id>=<val> | $RST)")
        action_vlayout = QVBoxLayout(action_group)
        action_vlayout.setContentsMargins(12, 10, 12, 10)
        action_vlayout.setSpacing(8)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(10)

        self.read_btn = QPushButton("📥 Đọc cấu hình ($0)")
        self.read_btn.setMinimumHeight(32)
        self.read_btn.setMinimumWidth(175)
        self.read_btn.setStyleSheet(
            "QPushButton { background-color: #0d6efd; color: white; font-weight: bold; "
            "padding: 4px 14px; border-radius: 4px; }"
            "QPushButton:disabled { background-color: #9ec5fe; color: #ffffff; }"
        )
        self.read_btn.setToolTip("Gửi lệnh $0 xuống STM32 để đọc toàn bộ thông số $1..$25 hiện tại")
        self.read_btn.clicked.connect(self.read_config_from_device)

        self.apply_changed_btn = QPushButton("💾 Ghi mục đã đổi")
        self.apply_changed_btn.setMinimumHeight(32)
        self.apply_changed_btn.setMinimumWidth(165)
        self.apply_changed_btn.setStyleSheet(
            "QPushButton { background-color: #198754; color: white; font-weight: bold; "
            "padding: 4px 14px; border-radius: 4px; }"
            "QPushButton:disabled { background-color: #a3cfbb; color: #ffffff; }"
        )
        self.apply_changed_btn.setToolTip(
            "Chỉ gửi các thông số có giá trị khác với lần đọc gần nhất từ STM32 (tuần tự chờ ACK)"
        )
        self.apply_changed_btn.clicked.connect(lambda: self.write_config_to_device(only_changed=True))

        self.apply_all_btn = QPushButton("📤 Ghi tất cả")
        self.apply_all_btn.setMinimumHeight(32)
        self.apply_all_btn.setMinimumWidth(140)
        self.apply_all_btn.setStyleSheet(
            "QPushButton { font-weight: bold; padding: 4px 14px; }"
        )
        self.apply_all_btn.setToolTip(
            "Gửi tuần tự toàn bộ các thông số cho phép chỉnh sửa xuống STM32, chờ ACK từng dòng"
        )
        self.apply_all_btn.clicked.connect(lambda: self.write_config_to_device(only_changed=False))

        self.reset_default_btn = QPushButton("🔄 Khôi phục mặc định ($RST)")
        self.reset_default_btn.setMinimumHeight(32)
        self.reset_default_btn.setMinimumWidth(200)
        self.reset_default_btn.setStyleSheet(
            "QPushButton { background-color: #dc3545; color: white; font-weight: bold; "
            "padding: 4px 14px; border-radius: 4px; }"
            "QPushButton:disabled { background-color: #f1aeb5; color: #ffffff; }"
        )
        self.reset_default_btn.setToolTip("Gửi lệnh $RST để STM32 khôi phục toàn bộ thông số gốc và tự đọc lại $0")
        self.reset_default_btn.clicked.connect(self.reset_device_defaults)

        self.cancel_wait_btn = QPushButton("✖ Hủy chờ")
        self.cancel_wait_btn.setMinimumHeight(32)
        self.cancel_wait_btn.setMinimumWidth(115)
        self.cancel_wait_btn.setStyleSheet(
            "QPushButton { font-weight: bold; color: #dc3545; padding: 4px 12px; }"
        )
        self.cancel_wait_btn.setToolTip("Hủy phiên chờ phản hồi cấu hình hiện tại (hữu ích khi bật chờ vô hạn)")
        self.cancel_wait_btn.setVisible(False)
        self.cancel_wait_btn.clicked.connect(self.cancel_current_operation)

        btn_row.addWidget(self.read_btn)
        btn_row.addWidget(self.apply_changed_btn)
        btn_row.addWidget(self.apply_all_btn)
        btn_row.addWidget(self.reset_default_btn)
        btn_row.addWidget(self.cancel_wait_btn)
        btn_row.addStretch()
        action_vlayout.addLayout(btn_row)

        ack_row = QHBoxLayout()
        ack_row.setSpacing(10)
        lbl_ack = QLabel("Giá trị xác nhận (ACK) mong đợi:")
        self.ack_value_edit = QLineEdit("OK")
        self.ack_value_edit.setFixedHeight(28)
        self.ack_value_edit.setMaximumWidth(220)
        self.ack_value_edit.setPlaceholderText("VD: OK, ok, done, ready...")
        self.ack_value_edit.setToolTip(
            "Chuỗi tín hiệu STM32 gửi về để xác nhận hoàn tất lệnh $0 / $<id>=<val> / $RST "
            "(mặc định là 'OK', không phân biệt chữ hoa/thường, có thể đổi thành bất kỳ giá trị nào)."
        )
        ack_row.addWidget(lbl_ack)
        ack_row.addWidget(self.ack_value_edit)
        ack_row.addStretch()
        action_vlayout.addLayout(ack_row)

        self.status_lbl = QLabel("● Chưa kết nối cổng UART.")
        self.status_lbl.setMinimumHeight(26)
        self.status_lbl.setStyleSheet(
            "font-weight: bold; color: #6c757d; background: #f8f9fa; "
            "padding: 4px 10px; border: 1px solid #dee2e6; border-radius: 4px;"
        )
        action_vlayout.addWidget(self.status_lbl)

        main_layout.addWidget(action_group)

        # --- Nhóm 1: Cơ khí & Giới hạn hành trình ($1 – $7) ---
        mech_group = QGroupBox("1. Cấu hình Cơ khí && Hành trình ($1 – $7)")
        mech_grid = QGridLayout(mech_group)
        mech_grid.setContentsMargins(12, 12, 12, 10)
        mech_grid.setHorizontalSpacing(14)
        mech_grid.setVerticalSpacing(8)

        self._add_numeric_row(
            mech_grid, 0, 0,
            param_id=1,
            label_text="$1 — Vi bước trục X:",
            unit_text="bước/vòng",
            tooltip="Số xung cần cấp để động cơ trục X quay đúng 1 vòng (X_STEPS_PER_REV).",
            is_float=True, decimals=3, min_val=0.001, max_val=10000000.0,
        )
        self._add_numeric_row(
            mech_grid, 0, 1,
            param_id=2,
            label_text="$2 — Vi bước trục Y:",
            unit_text="bước/vòng",
            tooltip="Số xung cần cấp để động cơ trục Y quay đúng 1 vòng (Y_STEPS_PER_REV).",
            is_float=True, decimals=3, min_val=0.001, max_val=10000000.0,
        )
        self._add_numeric_row(
            mech_grid, 1, 0,
            param_id=3,
            label_text="$3 — Bước tiến cơ khí:",
            unit_text="mm/vòng",
            tooltip="Quãng đường bàn máy đi được khi trục động cơ quay 1 vòng (MM_PER_REV).",
            is_float=True, decimals=3, min_val=0.001, max_val=100000.0,
        )
        self._add_numeric_row(
            mech_grid, 1, 1,
            param_id=6,
            label_text="$6 — Giới hạn tọa độ tối đa:",
            unit_text="mm",
            tooltip="Ngưỡng tọa độ phần mềm an toàn, vượt quá sẽ từ chối lệnh (MAX_COORD_LIMIT_MM).",
            is_float=True, decimals=3, min_val=0.001, max_val=1000000.0,
        )
        self._add_numeric_row(
            mech_grid, 2, 0,
            param_id=4,
            label_text="$4 — Độ phân giải trục X (Chỉ đọc):",
            unit_text="bước/mm",
            tooltip="Số xung trên 1 mm trục X (= $1 / $3), tự động tính trên STM32 (X_STEPS_PER_MM).",
            is_float=True, decimals=3, min_val=0.0, max_val=100000000.0,
            read_only=True,
        )
        self._add_numeric_row(
            mech_grid, 2, 1,
            param_id=5,
            label_text="$5 — Độ phân giải trục Y (Chỉ đọc):",
            unit_text="bước/mm",
            tooltip="Số xung trên 1 mm trục Y (= $2 / $3), tự động tính trên STM32 (Y_STEPS_PER_MM).",
            is_float=True, decimals=3, min_val=0.0, max_val=100000000.0,
            read_only=True,
        )
        self._add_numeric_row(
            mech_grid, 3, 0,
            param_id=7,
            label_text="$7 — Giới hạn bước Homing:",
            unit_text="bước",
            tooltip="Tổng xung tối đa khi dò gốc trước khi báo lỗi kẹt/hỏng Endstop (MAX_HOMING_STEPS >= 100).",
            is_float=False, decimals=0, min_val=100, max_val=100000000,
        )

        mech_grid.setColumnStretch(1, 1)
        mech_grid.setColumnStretch(4, 1)
        main_layout.addWidget(mech_group)

        # --- Nhóm 2: Động học & Tốc độ Động cơ ($10 – $16) ---
        kin_group = QGroupBox("2. Động học && Tốc độ Động cơ ($10 – $16)")
        kin_grid = QGridLayout(kin_group)
        kin_grid.setContentsMargins(12, 12, 12, 10)
        kin_grid.setHorizontalSpacing(14)
        kin_grid.setVerticalSpacing(8)

        self._add_numeric_row(
            kin_grid, 0, 0,
            param_id=10,
            label_text="$10 — Tần số bước tối thiểu:",
            unit_text="Hz",
            tooltip="Tốc độ xung sàn thấp nhất cho phép, không đặt dưới 8 Hz để tránh tràn TIM2 16-bit (MIN_STEP_FREQ).",
            is_float=True, decimals=1, min_val=8.0, max_val=50000.0,
        )
        self._add_numeric_row(
            kin_grid, 0, 1,
            param_id=11,
            label_text="$11 — Tần số khởi động (Đề-pa):",
            unit_text="Hz",
            tooltip="Tần số nhảy bậc ngay lúc bắt đầu chạy và trước khi dừng hẳn (START_STEP_FREQ).",
            is_float=True, decimals=1, min_val=8.0, max_val=50000.0,
        )
        self._add_numeric_row(
            kin_grid, 1, 0,
            param_id=12,
            label_text="$12 — Tốc độ chạy cắt G1 mặc định:",
            unit_text="Hz",
            tooltip="Tần số chạy G1 khi câu lệnh G-code chưa khai báo tham số F (G1_STEP_FREQ).",
            is_float=True, decimals=1, min_val=8.0, max_val=50000.0,
            with_mm_s_hint=True,
        )
        self._add_numeric_row(
            kin_grid, 1, 1,
            param_id=13,
            label_text="$13 — Tốc độ chạy nhanh G0:",
            unit_text="Hz",
            tooltip="Tần số di chuyển không tải cố định của mọi lệnh G0 (G0_STEP_FREQ).",
            is_float=True, decimals=1, min_val=8.0, max_val=50000.0,
            with_mm_s_hint=True,
        )
        self._add_numeric_row(
            kin_grid, 2, 0,
            param_id=14,
            label_text="$14 — Tốc độ dò gốc Homing:",
            unit_text="Hz",
            tooltip="Tần số lùi chậm về công tắc MIN khi thực thi $H hoặc G28 (HOMING_STEP_FREQ).",
            is_float=True, decimals=1, min_val=8.0, max_val=50000.0,
            with_mm_s_hint=True,
        )
        self._add_numeric_row(
            kin_grid, 2, 1,
            param_id=15,
            label_text="$15 — Tần số bước tối đa:",
            unit_text="Hz",
            tooltip="Trần tốc độ tối đa để kẹp biên tham số F, chống trượt bước (MAX_STEP_FREQ <= 50000 Hz).",
            is_float=True, decimals=1, min_val=8.0, max_val=50000.0,
            with_mm_s_hint=True,
        )
        self._add_numeric_row(
            kin_grid, 3, 0,
            param_id=16,
            label_text="$16 — Gia tốc tăng/giảm tốc:",
            unit_text="steps/s²",
            tooltip="Độ dốc sườn Hình thang/Tam giác — tăng/giảm bao nhiêu Hz mỗi giây (ACCELERATION).",
            is_float=True, decimals=1, min_val=10.0, max_val=500000.0,
        )

        kin_grid.setColumnStretch(1, 1)
        kin_grid.setColumnStretch(4, 1)
        main_layout.addWidget(kin_group)

        # --- Nhóm 3: Cấu hình Chân tín hiệu I/O ($20 – $25) ---
        io_group = QGroupBox("3. Cấu hình Chân tín hiệu I/O ($20 – $25)")
        io_grid = QGridLayout(io_group)
        io_grid.setContentsMargins(12, 12, 12, 10)
        io_grid.setHorizontalSpacing(14)
        io_grid.setVerticalSpacing(8)

        dir_items = [
            ("0 — Mức Thấp (RESET)", "0"),
            ("1 — Mức Cao (SET)", "1"),
        ]
        endstop_items = [
            ("0 — Thường mở (NO - GND)", "0"),
            ("1 — Thường đóng (NC - VCC)", "1"),
        ]

        self._add_combo_row(
            io_grid, 0, 0,
            param_id=20,
            label_text="$20 — Mức logic chiều dương X+:",
            items=dir_items,
            tooltip="Mức điện áp xuất ra chân DIR_X khi trục X tiến về hướng dương (DIR_X_POS_LEVEL).",
        )
        self._add_combo_row(
            io_grid, 0, 1,
            param_id=21,
            label_text="$21 — Mức logic chiều dương Y+:",
            items=dir_items,
            tooltip="Mức điện áp xuất ra chân DIR_Y khi trục Y tiến về hướng dương (DIR_Y_POS_LEVEL).",
        )
        self._add_combo_row(
            io_grid, 1, 0,
            param_id=22,
            label_text="$22 — Kích hoạt Endstop MIN_X:",
            items=endstop_items,
            tooltip="Mức tín hiệu báo công tắc gốc trục X đang bị chạm (MIN_X_TRIGGER).",
        )
        self._add_combo_row(
            io_grid, 1, 1,
            param_id=23,
            label_text="$23 — Kích hoạt Endstop MAX_X:",
            items=endstop_items,
            tooltip="Mức tín hiệu báo công tắc cuối hành trình trục X đang bị chạm (MAX_X_TRIGGER).",
        )
        self._add_combo_row(
            io_grid, 2, 0,
            param_id=24,
            label_text="$24 — Kích hoạt Endstop MIN_Y:",
            items=endstop_items,
            tooltip="Mức tín hiệu báo công tắc gốc trục Y đang bị chạm (MIN_Y_TRIGGER).",
        )
        self._add_combo_row(
            io_grid, 2, 1,
            param_id=25,
            label_text="$25 — Kích hoạt Endstop MAX_Y:",
            items=endstop_items,
            tooltip="Mức tín hiệu báo công tắc cuối hành trình trục Y đang bị chạm (MAX_Y_TRIGGER).",
        )

        io_grid.setColumnStretch(1, 1)
        io_grid.setColumnStretch(3, 1)
        main_layout.addWidget(io_group)

        main_layout.addStretch(1)
        scroll_area.setWidget(content)
        outer_layout.addWidget(scroll_area)

        # Kết nối tự động tính nhẩm $4, $5 và mm/s trên giao diện
        for pid in (1, 2, 3, 12, 13, 14, 15):
            w = self._inputs.get(pid)
            if isinstance(w, QLineEdit):
                w.textChanged.connect(self._recalculate_derived_previews)

    def _add_numeric_row(
        self,
        grid: QGridLayout,
        row: int,
        col_pair: int,
        param_id: int,
        label_text: str,
        unit_text: str,
        tooltip: str,
        is_float: bool,
        decimals: int,
        min_val: float,
        max_val: float,
        read_only: bool = False,
        with_mm_s_hint: bool = False,
    ) -> None:
        base_col = col_pair * 3
        lbl = QLabel(label_text)
        lbl.setToolTip(tooltip)

        edit = QLineEdit()
        edit.setFixedHeight(28)
        edit.setToolTip(tooltip)

        if read_only:
            edit.setReadOnly(True)
            edit.setStyleSheet(
                "QLineEdit { background-color: #e9ecef; color: #495057; font-weight: bold; }"
            )
        else:
            if is_float:
                validator = QDoubleValidator(min_val, max_val, decimals, edit)
                validator.setNotation(QDoubleValidator.StandardNotation)
                edit.setValidator(validator)
            else:
                edit.setValidator(QIntValidator(int(min_val), int(max_val), edit))

        unit_container = QWidget()
        unit_layout = QHBoxLayout(unit_container)
        unit_layout.setContentsMargins(0, 0, 0, 0)
        unit_layout.setSpacing(6)

        unit_lbl = QLabel(unit_text)
        unit_lbl.setStyleSheet("color: #495057;")
        unit_layout.addWidget(unit_lbl)

        if with_mm_s_hint:
            hint_lbl = QLabel("")
            hint_lbl.setStyleSheet("color: #0d6efd; font-size: 11px;")
            unit_layout.addWidget(hint_lbl)
            self._speed_hint_labels[param_id] = hint_lbl

        unit_layout.addStretch()

        grid.addWidget(lbl, row, base_col)
        grid.addWidget(edit, row, base_col + 1)
        grid.addWidget(unit_container, row, base_col + 2)

        self._inputs[param_id] = edit

    def _add_combo_row(
        self,
        grid: QGridLayout,
        row: int,
        col_pair: int,
        param_id: int,
        label_text: str,
        items: List[Tuple[str, str]],
        tooltip: str,
    ) -> None:
        base_col = col_pair * 2
        lbl = QLabel(label_text)
        lbl.setToolTip(tooltip)

        combo = QComboBox()
        combo.setFixedHeight(28)
        combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        combo.setToolTip(tooltip)
        for text, data in items:
            combo.addItem(text, data)

        grid.addWidget(lbl, row, base_col)
        grid.addWidget(combo, row, base_col + 1)

        self._inputs[param_id] = combo

    def _populate_default_placeholders(self) -> None:
        defaults = {
            1: "1600.000",
            2: "1600.000",
            3: "1.000",
            4: "1600.000",
            5: "1600.000",
            6: "10000.000",
            7: "800000",
            10: "20.0",
            11: "200.0",
            12: "1000.0",
            13: "2500.0",
            14: "500.0",
            15: "8000.0",
            16: "4000.0",
            20: "1",
            21: "0",
            22: "0",
            23: "0",
            24: "0",
            25: "0",
        }
        for pid, val in defaults.items():
            self._set_widget_value(pid, val)
        self._recalculate_derived_previews()

    def _set_widget_value(self, param_id: int, raw_val: str) -> None:
        w = self._inputs.get(param_id)
        if w is None:
            return
        if isinstance(w, QLineEdit):
            w.setText(raw_val.strip())
        elif isinstance(w, QComboBox):
            try:
                bit = str(int(round(float(raw_val.strip()))))
            except ValueError:
                bit = "0"
            idx = w.findData(bit)
            if idx >= 0:
                w.setCurrentIndex(idx)

    def _get_widget_value(self, param_id: int) -> str:
        w = self._inputs.get(param_id)
        if isinstance(w, QLineEdit):
            return w.text().strip().replace(",", ".")
        if isinstance(w, QComboBox):
            return str(w.currentData())
        return ""

    def _recalculate_derived_previews(self) -> None:
        try:
            s_x = float(self._get_widget_value(1))
            s_y = float(self._get_widget_value(2))
            mm_rev = float(self._get_widget_value(3))
            if mm_rev > 0:
                spm_x = s_x / mm_rev
                spm_y = s_y / mm_rev
                w4 = self._inputs.get(4)
                w5 = self._inputs.get(5)
                if isinstance(w4, QLineEdit):
                    w4.setText(f"{spm_x:.3f}")
                if isinstance(w5, QLineEdit):
                    w5.setText(f"{spm_y:.3f}")
            else:
                spm_x = 0.0
        except ValueError:
            try:
                spm_x = float(self._get_widget_value(4))
            except ValueError:
                spm_x = 0.0

        for pid, lbl in self._speed_hint_labels.items():
            try:
                freq = float(self._get_widget_value(pid))
                if spm_x > 0 and freq > 0:
                    lbl.setText(f"(≈ {freq / spm_x:.2f} mm/s)")
                else:
                    lbl.setText("")
            except ValueError:
                lbl.setText("")

    def _set_status(self, text: str, color: str = "#212529") -> None:
        self.status_lbl.setText(f"● {text}")
        self.status_lbl.setStyleSheet(
            f"font-weight: bold; color: {color}; background: #f8f9fa; "
            "padding: 4px 10px; border: 1px solid #dee2e6; border-radius: 4px;"
        )

    def _update_controls_enabled(self) -> None:
        connected = self.serial_mgr.is_connected
        idle = self._mode == "IDLE"
        can_act = connected and idle and (not self._tap_busy)

        self.read_btn.setEnabled(can_act)
        self.apply_changed_btn.setEnabled(can_act)
        self.apply_all_btn.setEnabled(can_act)
        self.reset_default_btn.setEnabled(can_act)
        self.cancel_wait_btn.setVisible(not idle)
        self.cancel_wait_btn.setEnabled(not idle)

        inputs_enabled = idle and (not self._tap_busy)
        self.ack_value_edit.setEnabled(inputs_enabled)
        for pid, w in self._inputs.items():
            if pid in (4, 5):
                continue
            w.setEnabled(inputs_enabled)

    def _on_connection_changed(self, is_connected: bool, message: str) -> None:
        if not is_connected:
            if self._cmd_timeout_timer.isActive():
                self._cmd_timeout_timer.stop()
            self._mode = "IDLE"
            self._write_queue.clear()
            self._current_write = None
            self._device_values.clear()
            self._set_status(f"Chưa kết nối ({message})", "#dc3545")
        else:
            if not self._tap_busy:
                self._set_status(
                    f"{message} — Nhấn 'Đọc cấu hình ($0)' để đồng bộ thông số từ STM32.",
                    "#0f5132",
                )
        self._update_controls_enabled()

    def read_config_from_device(self) -> None:
        if self._tap_busy or self._mode != "IDLE":
            return
        if not self.serial_mgr.is_connected:
            QMessageBox.warning(
                self,
                "Chưa kết nối UART",
                "Vui lòng kết nối cổng COM trên thanh công cụ trước khi đọc cấu hình ($0).",
            )
            return

        self._mode = "READING"
        self._update_controls_enabled()
        self._set_status("Đang gửi $0 và chờ STM32 trả về danh sách thông số...", "#0d6efd")

        if not self.serial_mgr.write_bytes(b"$0\r\n", source="CFG"):
            self._mode = "IDLE"
            self._update_controls_enabled()
            self._set_status("Lỗi khi gửi lệnh $0 qua UART.", "#dc3545")
            return

        self._arm_command_timeout()

    def _validate_editable_values(self) -> Optional[List[Tuple[int, str]]]:
        editable_ids = [1, 2, 3, 6, 7, 10, 11, 12, 13, 14, 15, 16, 20, 21, 22, 23, 24, 25]
        parsed_num: Dict[int, float] = {}
        pairs: List[Tuple[int, str]] = []

        for pid in editable_ids:
            val_str = self._get_widget_value(pid)
            if val_str == "":
                QMessageBox.warning(
                    self, "Thiếu giá trị", f"Ô thông số ${pid} đang để trống."
                )
                return None
            try:
                num = float(val_str)
            except ValueError:
                QMessageBox.warning(
                    self, "Sai định dạng số", f"Giá trị '{val_str}' của ${pid} không hợp lệ."
                )
                return None
            parsed_num[pid] = num

            if pid in (1, 2, 3, 6) and num <= 0.0:
                QMessageBox.warning(
                    self, "Ngoài giới hạn", f"Thông số ${pid} phải lớn hơn 0."
                )
                return None
            if pid == 7 and (num < 100 or num > 100000000):
                QMessageBox.warning(
                    self, "Ngoài giới hạn", "Thông số $7 (Giới hạn bước Homing) phải từ 100 đến 100,000,000."
                )
                return None
            if pid == 16 and (num < 10.0 or num > 500000.0):
                QMessageBox.warning(
                    self, "Ngoài giới hạn", "Thông số $16 (Gia tốc) phải nằm trong khoảng 10 .. 500,000 steps/s²."
                )
                return None

            if pid in (1, 2, 3, 6):
                formatted = f"{num:.3f}"
            elif pid == 7:
                formatted = str(int(round(num)))
            elif 10 <= pid <= 16:
                formatted = f"{num:.1f}"
            else:
                formatted = "1" if int(round(num)) == 1 else "0"

            pairs.append((pid, formatted))

        min_f = parsed_num[10]
        max_f = parsed_num[15]
        if min_f < 8.0:
            QMessageBox.warning(
                self, "Ngoài giới hạn", "Thông số $10 (MIN_STEP_FREQ) không được nhỏ hơn 8.0 Hz."
            )
            return None
        if max_f > 50000.0 or max_f <= min_f:
            QMessageBox.warning(
                self,
                "Ngoài giới hạn",
                "Thông số $15 (MAX_STEP_FREQ) phải lớn hơn $10 và không vượt quá 50,000 Hz.",
            )
            return None

        for pid in (11, 12, 13, 14):
            if not (min_f <= parsed_num[pid] <= max_f):
                QMessageBox.warning(
                    self,
                    "Ngoài giới hạn",
                    f"Thông số ${pid} ({parsed_num[pid]:.1f} Hz) phải nằm trong khoảng [$10={min_f:.1f} .. $15={max_f:.1f}] Hz.",
                )
                return None

        # Sắp xếp thứ tự gửi an toàn cho ràng buộc chéo $10/$15 trên STM32:
        # Nới rộng $15 và hạ $10 trước nếu cần, sau đó mới gửi các tần số $11..$14.
        old_max = float(self._device_values.get(15, "8000.0"))
        if max_f >= old_max:
            freq_order = [15, 10, 11, 12, 13, 14, 16]
        else:
            freq_order = [10, 11, 12, 13, 14, 15, 16]

        ordered_ids = [1, 2, 3, 6, 7] + freq_order + [20, 21, 22, 23, 24, 25]
        pair_map = dict(pairs)
        return [(pid, pair_map[pid]) for pid in ordered_ids]

    @staticmethod
    def _values_differ(param_id: int, new_str: str, old_str: Optional[str]) -> bool:
        if old_str is None:
            return True
        try:
            if param_id in (7, 20, 21, 22, 23, 24, 25):
                return int(round(float(new_str))) != int(round(float(old_str)))
            return abs(float(new_str) - float(old_str)) > 1e-4
        except ValueError:
            return new_str.strip() != old_str.strip()

    def write_config_to_device(self, only_changed: bool = True) -> None:
        if self._tap_busy or self._mode != "IDLE":
            return
        if not self.serial_mgr.is_connected:
            QMessageBox.warning(
                self,
                "Chưa kết nối UART",
                "Vui lòng kết nối cổng COM trước khi ghi cấu hình xuống STM32.",
            )
            return

        validated = self._validate_editable_values()
        if validated is None:
            return

        if only_changed and self._device_values:
            to_send = [
                (pid, val)
                for pid, val in validated
                if self._values_differ(pid, val, self._device_values.get(pid))
            ]
            if not to_send:
                self._set_status("Không có thông số nào thay đổi so với thiết bị.", "#0f5132")
                QMessageBox.information(
                    self,
                    "Không có thay đổi",
                    "Tất cả thông số trên giao diện đều trùng khớp với giá trị đã đọc từ STM32.",
                )
                return
        else:
            to_send = validated

        self._mech_changed_in_batch = any(pid in (1, 2, 3) for pid, _ in to_send)
        self._write_queue = list(to_send)
        self._write_applied_count = 0
        self._mode = "WRITING"
        self._auto_refresh_after_op = True
        self._update_controls_enabled()
        self._send_next_in_write_queue()

    def _send_next_in_write_queue(self) -> None:
        if not self._write_queue:
            self._cmd_timeout_timer.stop()
            count = self._write_applied_count
            mech_changed = getattr(self, "_mech_changed_in_batch", False)
            self._current_write = None
            self._mode = "IDLE"
            self._update_controls_enabled()
            self._set_status(
                f"Đã ghi thành công {count} thông số xuống STM32. Đang đọc lại $0...",
                "#198754",
            )
            if self._auto_refresh_after_op and self.serial_mgr.is_connected:
                self._auto_refresh_after_op = False
                self.read_config_from_device()
            if mech_changed:
                QMessageBox.information(
                    self,
                    "Lưu ý Homing lại",
                    "Bạn vừa thay đổi thông số cơ khí ($1, $2 hoặc $3).\n"
                    "Hãy thực hiện Homing ($H / G28) lại trước khi chạy G-code để đồng bộ gốc tọa độ.",
                )
            return

        pid, val = self._write_queue.pop(0)
        self._current_write = (pid, val)
        cmd_text = f"${pid}={val}"
        self._set_status(
            f"Đang ghi {cmd_text} (còn {len(self._write_queue)} lệnh chờ)...",
            "#0d6efd",
        )
        payload = (cmd_text + "\r\n").encode("ascii", errors="replace")
        if not self.serial_mgr.write_bytes(payload, source="CFG"):
            self._cmd_timeout_timer.stop()
            self._mode = "IDLE"
            self._write_queue.clear()
            self._current_write = None
            self._update_controls_enabled()
            self._set_status(f"Lỗi truyền UART khi gửi {cmd_text}.", "#dc3545")
            return

        self._arm_command_timeout()

    def reset_device_defaults(self) -> None:
        if self._tap_busy or self._mode != "IDLE":
            return
        if not self.serial_mgr.is_connected:
            QMessageBox.warning(
                self,
                "Chưa kết nối UART",
                "Vui lòng kết nối cổng COM trước khi gửi lệnh $RST.",
            )
            return

        ans = QMessageBox.question(
            self,
            "Khôi phục mặc định ($RST)",
            "Bạn có chắc muốn gửi lệnh $RST để khôi phục toàn bộ thông số phần cứng trên STM32 về mặc định gốc?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if ans != QMessageBox.Yes:
            return

        self._mode = "RESETTING"
        self._auto_refresh_after_op = True
        self._update_controls_enabled()
        self._set_status("Đang gửi lệnh $RST xuống STM32...", "#fd7e14")

        if not self.serial_mgr.write_bytes(b"$RST\r\n", source="CFG"):
            self._mode = "IDLE"
            self._update_controls_enabled()
            self._set_status("Lỗi khi gửi lệnh $RST qua UART.", "#dc3545")
            return

        self._arm_command_timeout()

    def _on_line_received(self, line: str) -> None:
        stripped = line.strip()
        if not stripped:
            return

        # Luôn cập nhật các dòng $<id>=<val> kể cả khi gửi $0 từ Terminal hoặc từ nút Đọc
        code_part = stripped.split(";", 1)[0].strip()
        m = _DOLLAR_LINE_RE.match(code_part)
        if m:
            pid = int(m.group(1))
            val_str = m.group(2)
            if pid in self._inputs:
                self._device_values[pid] = val_str
                self._set_widget_value(pid, val_str)
                self._recalculate_derived_previews()
            if self._mode == "READING":
                self._arm_command_timeout()
            return

        if self._mode == "IDLE":
            return

        low = stripped.lower()
        ack_expected = self._expected_ack()
        ack_low = ack_expected.lower()
        is_ack_match = (
            low == ack_low
            or low.startswith(ack_low + ":")
            or low.startswith(ack_low + " ")
        )
        upper = stripped.upper()
        if is_ack_match:
            if self._mode == "READING":
                self._cmd_timeout_timer.stop()
                self._mode = "IDLE"
                self._update_controls_enabled()
                self._set_status(
                    f"Đã đồng bộ thành công cấu hình từ STM32 ({len(self._device_values)} thông số).",
                    "#198754",
                )
            elif self._mode == "WRITING":
                self._cmd_timeout_timer.stop()
                if self._current_write is not None:
                    pid, val = self._current_write
                    self._device_values[pid] = val
                    self._write_applied_count += 1
                self._send_next_in_write_queue()
            elif self._mode == "RESETTING":
                self._cmd_timeout_timer.stop()
                self._mode = "IDLE"
                self._update_controls_enabled()
                self._set_status("Đã khôi phục mặc định ($RST) thành công. Đang đọc lại $0...", "#198754")
                if self._auto_refresh_after_op and self.serial_mgr.is_connected:
                    self._auto_refresh_after_op = False
                    self.read_config_from_device()
            return

        if upper.startswith("ERR:") or upper.startswith("ERROR"):
            self._cmd_timeout_timer.stop()
            failed_cmd = (
                f"${self._current_write[0]}={self._current_write[1]}"
                if (self._mode == "WRITING" and self._current_write)
                else self._mode
            )
            self._mode = "IDLE"
            self._write_queue.clear()
            self._current_write = None
            self._auto_refresh_after_op = False
            self._update_controls_enabled()
            self._set_status(f"STM32 từ chối lệnh ({failed_cmd}): {stripped}", "#dc3545")
            QMessageBox.warning(
                self,
                "STM32 báo lỗi cấu hình",
                f"Thiết bị phản hồi lỗi khi xử lý {failed_cmd}:\n{stripped}",
            )

    def _expected_ack(self) -> str:
        return self.ack_value_edit.text().strip() or "OK"

    def _on_ack_value_changed(self, text: str) -> None:
        clean_ack = text.strip() or "OK"
        if self.settings and hasattr(self.settings, "set_config_ack_value"):
            self.settings.set_config_ack_value(clean_ack)

    def _arm_command_timeout(self) -> None:
        if getattr(self.serial_mgr, "no_timeout", False):
            if self._cmd_timeout_timer.isActive():
                self._cmd_timeout_timer.stop()
            return
        timeout_sec = max(0.5, float(getattr(self.serial_mgr, "current_timeout", 5.0)))
        self._cmd_timeout_timer.setInterval(int(timeout_sec * 1000))
        self._cmd_timeout_timer.start()

    def on_timeout_config_changed(self) -> None:
        """Áp dụng ngay khi người dùng tick/bỏ tick 'Không chờ Timeout (Vô hạn)' hoặc đổi số giây Timeout."""
        if self._mode != "IDLE":
            self._arm_command_timeout()

    def cancel_current_operation(self) -> None:
        if self._mode == "IDLE":
            return
        prev_mode = self._mode
        if self._cmd_timeout_timer.isActive():
            self._cmd_timeout_timer.stop()
        self._mode = "IDLE"
        self._write_queue.clear()
        self._current_write = None
        self._auto_refresh_after_op = False
        self._update_controls_enabled()
        self._set_status(f"Đã hủy chờ phản hồi ({prev_mode}) bởi người dùng.", "#6c757d")

    def _on_command_timeout(self) -> None:
        if self._mode == "IDLE":
            return
        if getattr(self.serial_mgr, "no_timeout", False):
            return
        prev_mode = self._mode
        timeout_sec = float(getattr(self.serial_mgr, "current_timeout", 5.0))
        self._mode = "IDLE"
        self._write_queue.clear()
        self._current_write = None
        self._auto_refresh_after_op = False
        self._update_controls_enabled()
        self._set_status(
            f"Quá thời gian chờ ({timeout_sec:.1f}s) phản hồi '{self._expected_ack()}' từ STM32 ({prev_mode}). Kiểm tra kết nối hoặc Simulation RX.",
            "#dc3545",
        )
