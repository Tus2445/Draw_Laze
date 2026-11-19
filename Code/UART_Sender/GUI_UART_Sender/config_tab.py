import re
from typing import Dict, List, Optional, Set, Tuple

from PySide6.QtCore import Qt, QTimer, Signal
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
_LASER_STATUS_RE = re.compile(
    r"LAZE\s+Mode\s+(\d+)\s*:\s*Duty\s*=\s*([0-9.]+)%\s*,\s*Freq\s*=\s*([0-9.]+)",
    re.IGNORECASE,
)
_COMMAND_TIMEOUT_MS = 2500

# Bảng màu dịu mắt (Soft Slate & Calm Blue) cho các ô nhập liệu
_EDIT_NORMAL_STYLE = (
    "QLineEdit { background-color: #ffffff; color: #334155; "
    "border: 1px solid #cbd5e1; border-radius: 4px; padding: 2px 6px; font-weight: 600; }"
    "QLineEdit:focus { border: 1px solid #64748b; background-color: #f8fafc; }"
    "QLineEdit:disabled { background-color: #f1f5f9; color: #94a3b8; }"
)
_EDIT_MODIFIED_STYLE = (
    "QLineEdit { background-color: #f0f7ff; color: #1e40af; "
    "border: 1.5px solid #7cc4fa; border-radius: 4px; padding: 2px 6px; font-weight: bold; }"
    "QLineEdit:focus { border: 1.5px solid #3b82f6; background-color: #e0f2fe; }"
)
_EDIT_READONLY_STYLE = (
    "QLineEdit { background-color: #f1f5f9; color: #64748b; "
    "border: 1px solid #e2e8f0; border-radius: 4px; padding: 2px 6px; font-weight: 600; }"
)

_COMBO_NORMAL_STYLE = (
    "QComboBox { background-color: #ffffff; color: #334155; "
    "border: 1px solid #cbd5e1; border-radius: 4px; padding: 2px 6px; font-weight: 500; }"
    "QComboBox:focus { border: 1px solid #64748b; }"
    "QComboBox:disabled { background-color: #f1f5f9; color: #94a3b8; }"
)
_COMBO_MODIFIED_STYLE = (
    "QComboBox { background-color: #f0f7ff; color: #1e40af; "
    "border: 1.5px solid #7cc4fa; border-radius: 4px; padding: 2px 6px; font-weight: 600; }"
    "QComboBox:focus { border: 1.5px solid #3b82f6; }"
)

_LABEL_NORMAL_STYLE = "color: #334155; font-weight: 500;"
_LABEL_MODIFIED_STYLE = "color: #1e40af; font-weight: 600;"
_LABEL_READONLY_STYLE = "color: #64748b; font-style: italic;"


class ConfigTab(QWidget):
    """Tab 1: Config — Đọc ($0), Ghi ($<id>=<val>) và Khôi phục mặc định ($RST) cấu hình STM32 qua UART."""

    request_auto_connect = Signal()
    laser_config_synced = Signal(str, str, str)

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

        # Giá trị đã đọc từ STM32 và giá trị mốc để so sánh thay đổi
        self._device_values: Dict[int, str] = {}
        self._baseline_values: Dict[int, str] = {}
        self._modified_pids: Set[int] = set()
        self._suppress_change_tracking: bool = False
        self._has_read_config: bool = False

        # Lưu widget điều khiển, nhãn tên, văn bản gốc và tooltip gốc cho từng mã $id
        self._inputs: Dict[int, QWidget] = {}
        self._param_labels: Dict[int, QLabel] = {}
        self._param_base_texts: Dict[int, str] = {}
        self._param_base_tooltips: Dict[int, str] = {}
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
        self._connect_change_trackers()
        self._refresh_dirty_states()
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
                "#b45309",
            )
        elif self._mode == "IDLE":
            if self.serial_mgr.is_connected:
                self._set_status("Sẵn sàng đọc/ghi cấu hình STM32.", "#15803d")
            else:
                self._set_status("Chưa kết nối cổng UART.", "#64748b")

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
        self.read_btn.setMinimumWidth(170)
        self.read_btn.setStyleSheet(
            "QPushButton { background-color: #3b82f6; color: white; font-weight: 600; "
            "padding: 4px 14px; border-radius: 4px; border: none; }"
            "QPushButton:hover { background-color: #2563eb; }"
            "QPushButton:disabled { background-color: #e2e8f0; color: #94a3b8; }"
        )
        self.read_btn.setToolTip("Gửi lệnh $0 xuống STM32 để đọc toàn bộ thông số $1..$32 hiện tại")
        self.read_btn.clicked.connect(self.read_config_from_device)

        self.apply_changed_btn = QPushButton("💾 Ghi mục đã đổi (0)")
        self.apply_changed_btn.setMinimumHeight(32)
        self.apply_changed_btn.setMinimumWidth(180)
        self.apply_changed_btn.setStyleSheet(
            "QPushButton { background-color: #2b8a62; color: white; font-weight: 600; "
            "padding: 4px 14px; border-radius: 4px; border: none; }"
            "QPushButton:hover { background-color: #23704f; }"
            "QPushButton:disabled { background-color: #e2e8f0; color: #94a3b8; }"
        )
        self.apply_changed_btn.setToolTip(
            "Chỉ bật khi có thông số được chỉnh sửa. Gửi tuần tự các mục đã đổi xuống STM32."
        )
        self.apply_changed_btn.clicked.connect(lambda: self.write_config_to_device(only_changed=True))

        self.revert_changes_btn = QPushButton("↩ Hoàn tác sửa")
        self.revert_changes_btn.setMinimumHeight(32)
        self.revert_changes_btn.setMinimumWidth(135)
        self.revert_changes_btn.setStyleSheet(
            "QPushButton { background-color: #f1f5f9; color: #475569; font-weight: 600; "
            "border: 1px solid #cbd5e1; padding: 4px 12px; border-radius: 4px; }"
            "QPushButton:hover { background-color: #e2e8f0; }"
            "QPushButton:disabled { background-color: #f8fafc; color: #cbd5e1; border: 1px solid #e2e8f0; }"
        )
        self.revert_changes_btn.setToolTip(
            "Khôi phục lại các ô đang chỉnh sửa về giá trị gốc (từ lần đọc $0 gần nhất hoặc mặc định)"
        )
        self.revert_changes_btn.clicked.connect(self.revert_unsaved_changes)

        self.apply_all_btn = QPushButton("📤 Ghi tất cả")
        self.apply_all_btn.setMinimumHeight(32)
        self.apply_all_btn.setMinimumWidth(130)
        self.apply_all_btn.setStyleSheet(
            "QPushButton { background-color: #ffffff; color: #334155; font-weight: 600; "
            "border: 1px solid #cbd5e1; padding: 4px 14px; border-radius: 4px; }"
            "QPushButton:hover { background-color: #f1f5f9; }"
            "QPushButton:disabled { background-color: #f8fafc; color: #cbd5e1; border: 1px solid #e2e8f0; }"
        )
        self.apply_all_btn.setToolTip(
            "Gửi tuần tự toàn bộ các thông số cấu hình xuống STM32, chờ ACK từng dòng"
        )
        self.apply_all_btn.clicked.connect(lambda: self.write_config_to_device(only_changed=False))

        self.reset_default_btn = QPushButton("🔄 Khôi phục mặc định ($RST)")
        self.reset_default_btn.setMinimumHeight(32)
        self.reset_default_btn.setMinimumWidth(195)
        self.reset_default_btn.setStyleSheet(
            "QPushButton { background-color: #e06666; color: white; font-weight: 600; "
            "padding: 4px 14px; border-radius: 4px; border: none; }"
            "QPushButton:hover { background-color: #cc5252; }"
            "QPushButton:disabled { background-color: #f1f5f9; color: #94a3b8; }"
        )
        self.reset_default_btn.setToolTip("Gửi lệnh $RST để STM32 khôi phục toàn bộ thông số gốc và tự đọc lại $0")
        self.reset_default_btn.clicked.connect(self.reset_device_defaults)

        self.cancel_wait_btn = QPushButton("✖ Hủy chờ")
        self.cancel_wait_btn.setMinimumHeight(32)
        self.cancel_wait_btn.setMinimumWidth(110)
        self.cancel_wait_btn.setStyleSheet(
            "QPushButton { font-weight: 600; color: #b91c1c; padding: 4px 12px; }"
        )
        self.cancel_wait_btn.setToolTip("Hủy phiên chờ phản hồi cấu hình hiện tại (hữu ích khi bật chờ vô hạn)")
        self.cancel_wait_btn.setVisible(False)
        self.cancel_wait_btn.clicked.connect(self.cancel_current_operation)

        btn_row.addWidget(self.read_btn)
        btn_row.addWidget(self.apply_changed_btn)
        btn_row.addWidget(self.revert_changes_btn)
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
        self.ack_value_edit.setMaximumWidth(160)
        self.ack_value_edit.setStyleSheet(_EDIT_NORMAL_STYLE)
        self.ack_value_edit.setPlaceholderText("VD: OK, ok, done...")
        self.ack_value_edit.setToolTip(
            "Chuỗi tín hiệu STM32 gửi về để xác nhận hoàn tất lệnh $0 / $<id>=<val> / $RST "
            "(mặc định là 'OK', không phân biệt chữ hoa/thường)."
        )

        self.change_summary_badge = QLabel("✔ Chưa có thông số nào bị chỉnh sửa")
        self.change_summary_badge.setMinimumHeight(28)
        self.change_summary_badge.setStyleSheet(
            "color: #475569; background: #f1f5f9; border: 1px solid #e2e8f0; "
            "padding: 3px 10px; border-radius: 4px; font-weight: 500;"
        )

        ack_row.addWidget(lbl_ack)
        ack_row.addWidget(self.ack_value_edit)
        ack_row.addSpacing(8)
        ack_row.addWidget(self.change_summary_badge, stretch=1)
        action_vlayout.addLayout(ack_row)

        self.status_lbl = QLabel("● Chưa kết nối cổng UART.")
        self.status_lbl.setMinimumHeight(26)
        self.status_lbl.setStyleSheet(
            "font-weight: 600; color: #64748b; background: #f8fafc; "
            "padding: 4px 10px; border: 1px solid #e2e8f0; border-radius: 4px;"
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

        # --- Nhóm 4: Cấu hình Xung Laze PWM — TIM1_CH1 ($30 – $32) ---
        laser_group = QGroupBox("4. Cấu hình Xung Laze PWM — TIM1_CH1 ($30 – $32)")
        laser_vlayout = QVBoxLayout(laser_group)
        laser_vlayout.setContentsMargins(12, 12, 12, 10)
        laser_vlayout.setSpacing(10)

        # Hàng chọn Mode Laze hiện tại ($30)
        mode_row = QHBoxLayout()
        mode_row.setSpacing(12)
        lbl_mode_text = "$30 — Chế độ Laze hiện tại:"
        lbl_mode_tip = "Chọn chế độ điều khiển xung Laze trên STM32 (1: Laze 1 PFM, 2: Laze 2 PWM+PFM)."
        lbl_mode = QLabel(lbl_mode_text)
        lbl_mode.setStyleSheet(_LABEL_NORMAL_STYLE)
        lbl_mode.setToolTip(lbl_mode_tip)
        self._param_labels[30] = lbl_mode
        self._param_base_texts[30] = lbl_mode_text
        self._param_base_tooltips[30] = lbl_mode_tip

        self.laser_mode_combo = QComboBox()
        self.laser_mode_combo.setFixedHeight(28)
        self.laser_mode_combo.setMinimumWidth(360)
        self.laser_mode_combo.setStyleSheet(_COMBO_NORMAL_STYLE)
        self.laser_mode_combo.addItem(
            "1 — Laze 1: Điều tần PFM (G-code: S<Hz>, Duty cố định 70%)", "1"
        )
        self.laser_mode_combo.addItem(
            "2 — Laze 2: Điều xung & Tần số PWM+PFM (G-code: S<Duty%>,<Hz>)", "2"
        )
        self.laser_mode_combo.currentIndexChanged.connect(self._on_laser_mode_ui_changed)
        self._inputs[30] = self.laser_mode_combo

        self.laser_lock_badge = QLabel("")
        self.laser_lock_badge.setStyleSheet(
            "color: #475569; background: #f1f5f9; border: 1px solid #cbd5e1; "
            "padding: 3px 10px; border-radius: 4px; font-weight: 500;"
        )

        mode_row.addWidget(lbl_mode)
        mode_row.addWidget(self.laser_mode_combo)
        mode_row.addWidget(self.laser_lock_badge)
        mode_row.addStretch()
        laser_vlayout.addLayout(mode_row)

        # Hai khung con song song: Khung Laze 1 (Trái) & Khung Laze 2 (Phải)
        modes_split_row = QHBoxLayout()
        modes_split_row.setSpacing(14)

        # --- Khung con 1: Laze 1 (Mode 1) ---
        self.laze1_box = QGroupBox("Cấu hình Mode Laze 1 — Điều tần PFM (Cú pháp G-code: S1000)")
        laze1_grid = QGridLayout(self.laze1_box)
        laze1_grid.setContentsMargins(10, 10, 10, 10)
        laze1_grid.setHorizontalSpacing(10)
        laze1_grid.setVerticalSpacing(8)

        lbl_l1_duty = QLabel("$31 — Duty Cycle mặc định (Cố định):")
        lbl_l1_duty.setStyleSheet(_LABEL_READONLY_STYLE)
        self.laze1_duty_edit = QLineEdit("70.0")
        self.laze1_duty_edit.setFixedHeight(28)
        self.laze1_duty_edit.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.laze1_duty_edit.setReadOnly(True)
        self.laze1_duty_edit.setStyleSheet(_EDIT_READONLY_STYLE)
        self.laze1_duty_edit.setToolTip("Ở chế độ Laze 1, Duty Cycle luôn cố định ở mức 70.0% (Không chỉnh sửa).")

        self.lbl_l1_freq = QLabel("$32 — Tần số xung Laze 1:")
        self.lbl_l1_freq.setStyleSheet(_LABEL_NORMAL_STYLE)
        self.laze1_freq_edit = QLineEdit("0.0")
        self.laze1_freq_edit.setFixedHeight(28)
        self.laze1_freq_edit.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.laze1_freq_edit.setStyleSheet(_EDIT_NORMAL_STYLE)
        v_l1_freq = QDoubleValidator(0.0, 100000.0, 1, self.laze1_freq_edit)
        v_l1_freq.setNotation(QDoubleValidator.StandardNotation)
        self.laze1_freq_edit.setValidator(v_l1_freq)
        self.laze1_freq_edit.setToolTip("Tần số xung Laze 1 (0 .. 100,000 Hz). 0 Hz = Tắt Laze.")

        laze1_grid.addWidget(lbl_l1_duty, 0, 0)
        laze1_grid.addWidget(self.laze1_duty_edit, 0, 1)
        laze1_grid.addWidget(QLabel("% (Khóa ở 70%)"), 0, 2)

        laze1_grid.addWidget(self.lbl_l1_freq, 1, 0)
        laze1_grid.addWidget(self.laze1_freq_edit, 1, 1)
        laze1_grid.addWidget(QLabel("Hz"), 1, 2)
        laze1_grid.setColumnStretch(1, 1)

        # --- Khung con 2: Laze 2 (Mode 2) ---
        self.laze2_box = QGroupBox("Cấu hình Mode Laze 2 — Điều xung & Tần số (Cú pháp G-code: S10,1000)")
        laze2_grid = QGridLayout(self.laze2_box)
        laze2_grid.setContentsMargins(10, 10, 10, 10)
        laze2_grid.setHorizontalSpacing(10)
        laze2_grid.setVerticalSpacing(8)

        lbl_l2_duty_text = "$31 — Chu kỳ xung (Duty Cycle):"
        lbl_l2_duty_tip = "Chu kỳ xung Laze 2 (0.0 .. 100.0 %)."
        self.lbl_l2_duty = QLabel(lbl_l2_duty_text)
        self.lbl_l2_duty.setStyleSheet(_LABEL_NORMAL_STYLE)
        self.lbl_l2_duty.setToolTip(lbl_l2_duty_tip)
        self._param_labels[31] = self.lbl_l2_duty
        self._param_base_texts[31] = lbl_l2_duty_text
        self._param_base_tooltips[31] = lbl_l2_duty_tip

        self.laze2_duty_edit = QLineEdit("70.0")
        self.laze2_duty_edit.setFixedHeight(28)
        self.laze2_duty_edit.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.laze2_duty_edit.setStyleSheet(_EDIT_NORMAL_STYLE)
        v_l2_duty = QDoubleValidator(0.0, 100.0, 1, self.laze2_duty_edit)
        v_l2_duty.setNotation(QDoubleValidator.StandardNotation)
        self.laze2_duty_edit.setValidator(v_l2_duty)
        self.laze2_duty_edit.setToolTip(lbl_l2_duty_tip)

        lbl_l2_freq_text = "$32 — Tần số xung Laze 2:"
        lbl_l2_freq_tip = "Tần số xung Laze (0 .. 100,000 Hz). 0 Hz = Tắt Laze."
        self.lbl_l2_freq = QLabel(lbl_l2_freq_text)
        self.lbl_l2_freq.setStyleSheet(_LABEL_NORMAL_STYLE)
        self.lbl_l2_freq.setToolTip(lbl_l2_freq_tip)
        self._param_labels[32] = self.lbl_l1_freq
        self._param_base_texts[32] = "$32 — Tần số xung Laze:"
        self._param_base_tooltips[32] = lbl_l2_freq_tip

        self.laze2_freq_edit = QLineEdit("0.0")
        self.laze2_freq_edit.setFixedHeight(28)
        self.laze2_freq_edit.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.laze2_freq_edit.setStyleSheet(_EDIT_NORMAL_STYLE)
        v_l2_freq = QDoubleValidator(0.0, 100000.0, 1, self.laze2_freq_edit)
        v_l2_freq.setNotation(QDoubleValidator.StandardNotation)
        self.laze2_freq_edit.setValidator(v_l2_freq)
        self.laze2_freq_edit.setToolTip(lbl_l2_freq_tip)

        laze2_grid.addWidget(self.lbl_l2_duty, 0, 0)
        laze2_grid.addWidget(self.laze2_duty_edit, 0, 1)
        laze2_grid.addWidget(QLabel("% (0 .. 100%)"), 0, 2)

        laze2_grid.addWidget(self.lbl_l2_freq, 1, 0)
        laze2_grid.addWidget(self.laze2_freq_edit, 1, 1)
        laze2_grid.addWidget(QLabel("Hz"), 1, 2)
        laze2_grid.setColumnStretch(1, 1)

        modes_split_row.addWidget(self.laze1_box, stretch=1)
        modes_split_row.addWidget(self.laze2_box, stretch=1)
        laser_vlayout.addLayout(modes_split_row)

        # Đồng bộ giá trị tần số $32 giữa 2 ô khi người dùng gõ ở mode đang mở
        self.laze1_freq_edit.textEdited.connect(self.laze2_freq_edit.setText)
        self.laze2_freq_edit.textEdited.connect(self.laze1_freq_edit.setText)

        self._inputs[31] = self.laze2_duty_edit
        self._inputs[32] = self.laze1_freq_edit

        main_layout.addWidget(laser_group)

        main_layout.addStretch(1)
        scroll_area.setWidget(content)
        outer_layout.addWidget(scroll_area)

        # Kết nối tự động tính nhẩm $4, $5 và mm/s trên giao diện
        for pid in (1, 2, 3, 12, 13, 14, 15):
            w = self._inputs.get(pid)
            if isinstance(w, QLineEdit):
                w.textChanged.connect(self._recalculate_derived_previews)

        self._apply_laser_mode_lock()

    def _connect_change_trackers(self) -> None:
        """Lắng nghe mọi thao tác sửa thông số của người dùng để đánh dấu màu trực quan."""
        for pid, w in self._inputs.items():
            if pid in (4, 5):
                continue
            if isinstance(w, QLineEdit):
                w.textChanged.connect(self._on_any_param_edited)
            elif isinstance(w, QComboBox):
                w.currentIndexChanged.connect(self._on_any_param_edited)

        self.laze2_freq_edit.textChanged.connect(self._on_any_param_edited)

    def _on_any_param_edited(self, *_args) -> None:
        if self._suppress_change_tracking:
            return
        self._refresh_dirty_states()

    def _on_laser_mode_ui_changed(self, _index: int) -> None:
        self._apply_laser_mode_lock()
        if not self._suppress_change_tracking:
            self._refresh_dirty_states()

    def _apply_laser_mode_lock(self) -> None:
        """Khóa hoàn toàn khung cấu hình của Mode Laze không được chọn."""
        inputs_unlocked = (self._mode == "IDLE") and (not self._tap_busy)
        is_mode_1 = str(self.laser_mode_combo.currentData()) == "1"

        self.laze1_box.setEnabled(inputs_unlocked and is_mode_1)
        self.laze2_box.setEnabled(inputs_unlocked and (not is_mode_1))

        if is_mode_1:
            self.laze1_duty_edit.setText("70.0")
            self.laser_lock_badge.setText("🔒 Đang mở Laze 1 — Đã khóa khung Laze 2 (Duty cố định 70%)")
        else:
            self.laser_lock_badge.setText("🔒 Đang mở Laze 2 — Đã khóa khung Laze 1 (Chỉnh được cả Duty & Tần số)")

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
        lbl.setStyleSheet(_LABEL_READONLY_STYLE if read_only else _LABEL_NORMAL_STYLE)

        edit = QLineEdit()
        edit.setFixedHeight(28)
        edit.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        edit.setToolTip(tooltip)

        if read_only:
            edit.setReadOnly(True)
            edit.setStyleSheet(_EDIT_READONLY_STYLE)
        else:
            edit.setStyleSheet(_EDIT_NORMAL_STYLE)
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
        unit_lbl.setStyleSheet("color: #64748b;")
        unit_layout.addWidget(unit_lbl)

        if with_mm_s_hint:
            hint_lbl = QLabel("")
            hint_lbl.setStyleSheet(
                "color: #475569; background: #f1f5f9; padding: 1px 6px; "
                "border-radius: 3px; font-size: 11px;"
            )
            unit_layout.addWidget(hint_lbl)
            self._speed_hint_labels[param_id] = hint_lbl

        unit_layout.addStretch()

        grid.addWidget(lbl, row, base_col)
        grid.addWidget(edit, row, base_col + 1)
        grid.addWidget(unit_container, row, base_col + 2)

        self._inputs[param_id] = edit
        self._param_labels[param_id] = lbl
        self._param_base_texts[param_id] = label_text
        self._param_base_tooltips[param_id] = tooltip

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
        lbl.setStyleSheet(_LABEL_NORMAL_STYLE)

        combo = QComboBox()
        combo.setFixedHeight(28)
        combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        combo.setStyleSheet(_COMBO_NORMAL_STYLE)
        combo.setToolTip(tooltip)
        for text, data in items:
            combo.addItem(text, data)

        grid.addWidget(lbl, row, base_col)
        grid.addWidget(combo, row, base_col + 1)

        self._inputs[param_id] = combo
        self._param_labels[param_id] = lbl
        self._param_base_texts[param_id] = label_text
        self._param_base_tooltips[param_id] = tooltip

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
            30: "1",
            31: "70.0",
            32: "0.0",
        }
        self._suppress_change_tracking = True
        try:
            for pid, val in defaults.items():
                self._baseline_values[pid] = val
                self._set_widget_value(pid, val)
            self._recalculate_derived_previews()
        finally:
            self._suppress_change_tracking = False

    def _get_reference_value(self, param_id: int) -> Optional[str]:
        """Lấy giá trị gốc để so sánh (ưu tiên giá trị đã đọc từ STM32, nếu chưa đọc thì dùng mặc định)."""
        if param_id in self._device_values:
            return self._device_values[param_id]
        return self._baseline_values.get(param_id)

    def _editable_param_ids(self) -> List[int]:
        is_mode_1 = str(self.laser_mode_combo.currentData()) == "1"
        laser_ids = [30, 32] if is_mode_1 else [30, 31, 32]
        return [
            1, 2, 3, 6, 7,
            10, 11, 12, 13, 14, 15, 16,
            20, 21, 22, 23, 24, 25,
        ] + laser_ids

    def _apply_param_visual_state(
        self, param_id: int, is_modified: bool, ref_val: Optional[str], cur_val: str
    ) -> None:
        """Đổi màu nền/viền dịu nhẹ và hiển thị giá trị gốc khi thông số bị chỉnh sửa."""
        base_tip = self._param_base_tooltips.get(param_id, "")
        ref_source = "trên STM32" if param_id in self._device_values else "mặc định"
        if is_modified and ref_val is not None:
            tip = (
                f"{base_tip}\n\n"
                f"● Đang chỉnh sửa:\n"
                f"• Giá trị {ref_source}: {ref_val}\n"
                f"• Giá trị mới nhập: {cur_val or '(trống)'}"
            )
        elif ref_val is not None:
            tip = f"{base_tip}\n\n• Giá trị hiện tại ({ref_source}): {ref_val}"
        else:
            tip = base_tip

        if param_id == 32:
            style = _EDIT_MODIFIED_STYLE if is_modified else _EDIT_NORMAL_STYLE
            lbl_style = _LABEL_MODIFIED_STYLE if is_modified else _LABEL_NORMAL_STYLE
            self.laze1_freq_edit.setStyleSheet(style)
            self.laze2_freq_edit.setStyleSheet(style)
            self.laze1_freq_edit.setToolTip(tip)
            self.laze2_freq_edit.setToolTip(tip)

            l1_text = "$32 — Tần số xung Laze 1:"
            l2_text = "$32 — Tần số xung Laze 2:"
            if is_modified and ref_val is not None:
                self.lbl_l1_freq.setText(f"● {l1_text} (gốc: {ref_val})")
                self.lbl_l2_freq.setText(f"● {l2_text} (gốc: {ref_val})")
            else:
                self.lbl_l1_freq.setText(l1_text)
                self.lbl_l2_freq.setText(l2_text)
            self.lbl_l1_freq.setStyleSheet(lbl_style)
            self.lbl_l2_freq.setStyleSheet(lbl_style)
            return

        w = self._inputs.get(param_id)
        lbl = self._param_labels.get(param_id)
        base_text = self._param_base_texts.get(param_id, f"${param_id}:")

        if isinstance(w, QLineEdit):
            w.setStyleSheet(_EDIT_MODIFIED_STYLE if is_modified else _EDIT_NORMAL_STYLE)
            w.setToolTip(tip)
        elif isinstance(w, QComboBox):
            w.setStyleSheet(_COMBO_MODIFIED_STYLE if is_modified else _COMBO_NORMAL_STYLE)
            w.setToolTip(tip)

        if lbl is not None:
            if is_modified and ref_val is not None:
                if isinstance(w, QLineEdit):
                    lbl.setText(f"● {base_text} (gốc: {ref_val})")
                else:
                    lbl.setText(f"● {base_text}")
                lbl.setStyleSheet(_LABEL_MODIFIED_STYLE)
            else:
                lbl.setText(base_text)
                lbl.setStyleSheet(_LABEL_NORMAL_STYLE)
            lbl.setToolTip(tip)

    def _refresh_dirty_states(self) -> None:
        """Kiểm tra toàn bộ các thông số xem mục nào đang bị chỉnh sửa để cập nhật UI & nút Ghi."""
        editable_set = set(self._editable_param_ids())
        all_check_ids = [
            1, 2, 3, 6, 7,
            10, 11, 12, 13, 14, 15, 16,
            20, 21, 22, 23, 24, 25,
            30, 31, 32,
        ]

        modified: Set[int] = set()
        for pid in all_check_ids:
            if pid not in editable_set:
                self._apply_param_visual_state(pid, False, self._get_reference_value(pid), "")
                continue
            cur_val = self._get_widget_value(pid)
            ref_val = self._get_reference_value(pid)
            is_diff = (cur_val == "") or self._values_differ(pid, cur_val, ref_val)
            if is_diff:
                modified.add(pid)
            self._apply_param_visual_state(pid, is_diff, ref_val, cur_val)

        self._modified_pids = modified
        count = len(modified)

        self.apply_changed_btn.setText(f"💾 Ghi mục đã đổi ({count})")
        if count > 0:
            sorted_tags = ", ".join(f"${pid}" for pid in sorted(modified))
            self.change_summary_badge.setText(
                f"● Đang chỉnh {count} thông số chưa lưu: {sorted_tags}"
            )
            self.change_summary_badge.setStyleSheet(
                "color: #1e40af; background: #eff6ff; border: 1px solid #bfdbfe; "
                "padding: 3px 10px; border-radius: 4px; font-weight: 600;"
            )
        else:
            sync_note = (
                "đồng bộ với STM32"
                if self._device_values
                else "giữ nguyên theo mặc định"
            )
            self.change_summary_badge.setText(
                f"✔ Không có thông số nào bị chỉnh sửa ({sync_note})"
            )
            self.change_summary_badge.setStyleSheet(
                "color: #475569; background: #f1f5f9; border: 1px solid #e2e8f0; "
                "padding: 3px 10px; border-radius: 4px; font-weight: 500;"
            )

        self._update_controls_enabled()

    def revert_unsaved_changes(self) -> None:
        """Hoàn tác toàn bộ các thông số đang sửa dở về giá trị gốc gần nhất."""
        if not self._modified_pids:
            return
        self._suppress_change_tracking = True
        try:
            for pid in (
                1, 2, 3, 6, 7,
                10, 11, 12, 13, 14, 15, 16,
                20, 21, 22, 23, 24, 25,
                30, 31, 32,
            ):
                ref_val = self._get_reference_value(pid)
                if ref_val is not None:
                    self._set_widget_value(pid, ref_val)
            self._recalculate_derived_previews()
        finally:
            self._suppress_change_tracking = False

        self._refresh_dirty_states()
        self._set_status("Đã hoàn tác các thay đổi chưa lưu về giá trị gốc.", "#475569")

    def _set_widget_value(self, param_id: int, raw_val: str) -> None:
        clean_val = raw_val.strip()
        prev_suppress = self._suppress_change_tracking
        self._suppress_change_tracking = True
        try:
            if param_id == 30:
                try:
                    code_str = str(int(round(float(clean_val))))
                except ValueError:
                    code_str = "1"
                idx = self.laser_mode_combo.findData(code_str)
                if idx >= 0:
                    self.laser_mode_combo.blockSignals(True)
                    self.laser_mode_combo.setCurrentIndex(idx)
                    self.laser_mode_combo.blockSignals(False)
                self._apply_laser_mode_lock()
                return

            if param_id == 31:
                self.laze1_duty_edit.setText("70.0")
                self.laze2_duty_edit.setText(clean_val)
                return

            if param_id == 32:
                self.laze1_freq_edit.setText(clean_val)
                self.laze2_freq_edit.setText(clean_val)
                return

            w = self._inputs.get(param_id)
            if w is None:
                return
            if isinstance(w, QLineEdit):
                w.setText(clean_val)
            elif isinstance(w, QComboBox):
                try:
                    bit = str(int(round(float(clean_val))))
                except ValueError:
                    bit = "0"
                idx = w.findData(bit)
                if idx >= 0:
                    w.setCurrentIndex(idx)
        finally:
            self._suppress_change_tracking = prev_suppress

    def _get_widget_value(self, param_id: int) -> str:
        if param_id == 30:
            return str(self.laser_mode_combo.currentData())
        if param_id == 31:
            is_mode_1 = str(self.laser_mode_combo.currentData()) == "1"
            return "70.0" if is_mode_1 else self.laze2_duty_edit.text().strip().replace(",", ".")
        if param_id == 32:
            is_mode_1 = str(self.laser_mode_combo.currentData()) == "1"
            target_edit = self.laze1_freq_edit if is_mode_1 else self.laze2_freq_edit
            return target_edit.text().strip().replace(",", ".")

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
                    lbl.setText(f"≈ {freq / spm_x:.2f} mm/s")
                else:
                    lbl.setText("")
            except ValueError:
                lbl.setText("")

    def _set_status(self, text: str, color: str = "#334155") -> None:
        self.status_lbl.setText(f"● {text}")
        self.status_lbl.setStyleSheet(
            f"font-weight: 600; color: {color}; background: #f8fafc; "
            "padding: 4px 10px; border: 1px solid #e2e8f0; border-radius: 4px;"
        )

    def _update_controls_enabled(self) -> None:
        connected = self.serial_mgr.is_connected
        idle = self._mode == "IDLE"
        ready_ui = idle and (not self._tap_busy)
        can_act_connected = connected and ready_ui
        has_changes = len(self._modified_pids) > 0

        self.read_btn.setEnabled(can_act_connected)
        # Mở khóa ngay khi có ít nhất 1 thông số thay đổi (kể cả khi chưa bấm Connect,
        # khi bấm nút sẽ tự động yêu cầu kết nối hoặc nhắc kết nối rõ ràng).
        self.apply_changed_btn.setEnabled(ready_ui and has_changes)
        self.revert_changes_btn.setEnabled(ready_ui and has_changes)
        self.apply_all_btn.setEnabled(can_act_connected)
        self.reset_default_btn.setEnabled(can_act_connected)
        self.cancel_wait_btn.setVisible(not idle)
        self.cancel_wait_btn.setEnabled(not idle)

        self.ack_value_edit.setEnabled(ready_ui)
        for pid, w in self._inputs.items():
            if pid in (4, 5, 31, 32):
                continue
            w.setEnabled(ready_ui)

        self._apply_laser_mode_lock()

    def _on_connection_changed(self, is_connected: bool, message: str) -> None:
        if not is_connected:
            if self._cmd_timeout_timer.isActive():
                self._cmd_timeout_timer.stop()
            self._mode = "IDLE"
            self._write_queue.clear()
            self._current_write = None
            self._device_values.clear()
            self._has_read_config = False
            self._set_status(f"Chưa kết nối ({message})", "#b91c1c")
        else:
            if not self._tap_busy:
                self._set_status(
                    f"{message} — Nhấn 'Đọc cấu hình ($0)' để đồng bộ thông số từ STM32.",
                    "#15803d",
                )
        self._refresh_dirty_states()
        self._update_controls_enabled()

    def _ensure_connected_or_warn(self, action_name: str) -> bool:
        """Tự động thử kết nối cổng COM nếu người dùng bấm Ghi khi chưa kết nối."""
        if self.serial_mgr.is_connected:
            return True
        self.request_auto_connect.emit()
        if self.serial_mgr.is_connected:
            return True
        QMessageBox.warning(
            self,
            "Chưa kết nối UART",
            f"Vui lòng bấm 'Kết nối (Connect)' cổng COM (hoặc bật Simulation Mode) "
            f"trên thanh công cụ phía trên trước khi {action_name}.",
        )
        return False

    def read_config_from_device(self) -> None:
        if self._tap_busy or self._mode != "IDLE":
            return
        if not self._ensure_connected_or_warn("đọc cấu hình ($0)"):
            return

        self._mode = "READING"
        self._update_controls_enabled()
        self._set_status("Đang gửi $0 và chờ STM32 trả về danh sách thông số...", "#2563eb")

        if not self.serial_mgr.write_bytes(b"$0\r\n", source="CFG"):
            self._mode = "IDLE"
            self._update_controls_enabled()
            self._set_status("Lỗi khi gửi lệnh $0 qua UART.", "#b91c1c")
            return

        self._arm_command_timeout()

    def _validate_editable_values(self) -> Optional[List[Tuple[int, str]]]:
        is_mode_1 = str(self.laser_mode_combo.currentData()) == "1"
        laser_editable_ids = [30, 32] if is_mode_1 else [30, 31, 32]
        editable_ids = [
            1, 2, 3, 6, 7,
            10, 11, 12, 13, 14, 15, 16,
            20, 21, 22, 23, 24, 25,
        ] + laser_editable_ids

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
            if pid == 30 and int(round(num)) not in (1, 2):
                QMessageBox.warning(
                    self, "Ngoài giới hạn", "Thông số $30 (Chế độ Laze) chỉ nhận 1 (Laze 1) hoặc 2 (Laze 2)."
                )
                return None
            if pid == 31 and (num < 0.0 or num > 100.0):
                QMessageBox.warning(
                    self, "Ngoài giới hạn", "Thông số $31 (Duty Cycle Laze 2) phải nằm trong khoảng 0 .. 100%."
                )
                return None
            if pid == 32 and (num < 0.0 or num > 100000.0):
                QMessageBox.warning(
                    self, "Ngoài giới hạn", "Thông số $32 (Tần số Laze) phải nằm trong khoảng 0 .. 100,000 Hz."
                )
                return None

            if pid in (1, 2, 3, 6):
                formatted = f"{num:.3f}"
            elif pid == 7:
                formatted = str(int(round(num)))
            elif (10 <= pid <= 16) or pid in (31, 32):
                formatted = f"{num:.1f}"
            elif pid == 30:
                formatted = "1" if int(round(num)) == 1 else "2"
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

        old_max = float(self._get_reference_value(15) or "8000.0")
        if max_f >= old_max:
            freq_order = [15, 10, 11, 12, 13, 14, 16]
        else:
            freq_order = [10, 11, 12, 13, 14, 15, 16]

        ordered_ids = (
            [1, 2, 3, 6, 7]
            + freq_order
            + [20, 21, 22, 23, 24, 25]
            + laser_editable_ids
        )
        pair_map = dict(pairs)
        return [(pid, pair_map[pid]) for pid in ordered_ids]

    @staticmethod
    def _values_differ(param_id: int, new_str: str, old_str: Optional[str]) -> bool:
        if old_str is None:
            return True
        try:
            if param_id in (7, 20, 21, 22, 23, 24, 25, 30):
                return int(round(float(new_str))) != int(round(float(old_str)))
            return abs(float(new_str) - float(old_str)) > 1e-4
        except ValueError:
            return new_str.strip() != old_str.strip()

    def write_config_to_device(self, only_changed: bool = True) -> None:
        if self._tap_busy or self._mode != "IDLE":
            return

        # 1. Nếu bấm "Ghi mục đã đổi", chỉ lấy đúng các ID nằm trong self._modified_pids
        if only_changed and not self._modified_pids:
            self._set_status("Không có thông số nào được chỉnh sửa để ghi.", "#475569")
            return

        validated = self._validate_editable_values()
        if validated is None:
            return

        if not self._ensure_connected_or_warn("ghi cấu hình xuống STM32"):
            return

        if only_changed:
            # Chỉ giữ lại đúng những thông số đang được đánh dấu là đã chỉnh sửa
            to_send = [
                (pid, val)
                for pid, val in validated
                if pid in self._modified_pids
            ]
            if not to_send:
                self._refresh_dirty_states()
                self._set_status("Không có thông số nào thay đổi so với thiết bị.", "#15803d")
                return
        else:
            to_send = validated

        self._mech_changed_in_batch = any(pid in (1, 2, 3) for pid, _ in to_send)
        self._write_queue = list(to_send)
        self._write_applied_count = 0
        self._mode = "WRITING"
        # Không tự động bắn thêm lệnh $0 sau khi ghi xong để tránh tràn log
        self._auto_refresh_after_op = False
        self._update_controls_enabled()
        self._send_next_in_write_queue()

    def _send_next_in_write_queue(self) -> None:
        if not self._write_queue:
            self._cmd_timeout_timer.stop()
            count = self._write_applied_count
            mech_changed = getattr(self, "_mech_changed_in_batch", False)
            self._current_write = None
            self._mode = "IDLE"
            self._recalculate_derived_previews()
            self._refresh_dirty_states()
            self._update_controls_enabled()
            self._set_status(
                f"Đã ghi thành công {count} thông số đã đổi xuống STM32.",
                "#15803d",
            )
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
            "#2563eb",
        )
        payload = (cmd_text + "\r\n").encode("ascii", errors="replace")
        if not self.serial_mgr.write_bytes(payload, source="CFG"):
            self._cmd_timeout_timer.stop()
            self._mode = "IDLE"
            self._write_queue.clear()
            self._current_write = None
            self._update_controls_enabled()
            self._set_status(f"Lỗi truyền UART khi gửi {cmd_text}.", "#b91c1c")
            return

        self._arm_command_timeout()

    def reset_device_defaults(self) -> None:
        if self._tap_busy or self._mode != "IDLE":
            return
        if not self._ensure_connected_or_warn("gửi lệnh $RST"):
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
        self._set_status("Đang gửi lệnh $RST xuống STM32...", "#d97706")

        if not self.serial_mgr.write_bytes(b"$RST\r\n", source="CFG"):
            self._mode = "IDLE"
            self._update_controls_enabled()
            self._set_status("Lỗi khi gửi lệnh $RST qua UART.", "#b91c1c")
            return

        self._arm_command_timeout()

    def _emit_laser_sync_if_read(self) -> None:
        """Chỉ phát dữ liệu Laze sang Tab 2 nếu đã thực hiện lệnh Đọc ($0) thành công."""
        if not self._has_read_config:
            return
        mode_raw = self._device_values.get(30, self._get_widget_value(30))
        try:
            mode_str = "2" if int(round(float(mode_raw))) == 2 else "1"
        except ValueError:
            mode_str = "1"
        duty_str = (
            "70.0"
            if mode_str == "1"
            else self._device_values.get(31, self._get_widget_value(31))
        )
        freq_str = self._device_values.get(32, self._get_widget_value(32))
        self.laser_config_synced.emit(mode_str, duty_str, freq_str)

    def _on_line_received(self, line: str) -> None:
        stripped = line.strip()
        if not stripped:
            return

        upper_line = stripped.upper()
        if "LAZE1" in upper_line:
            self._device_values[30] = "1"
            self._device_values[31] = "70.0"
            self._baseline_values[30] = "1"
            self._baseline_values[31] = "70.0"
            self._set_widget_value(30, "1")
            self._set_widget_value(31, "70.0")
            self._refresh_dirty_states()
            self._emit_laser_sync_if_read()
        elif "LAZE2" in upper_line:
            self._device_values[30] = "2"
            self._baseline_values[30] = "2"
            self._set_widget_value(30, "2")
            self._refresh_dirty_states()
            self._emit_laser_sync_if_read()
        else:
            m_laser = _LASER_STATUS_RE.search(stripped)
            if m_laser:
                mode_s, duty_s, freq_s = m_laser.group(1), m_laser.group(2), m_laser.group(3)
                self._device_values[30] = mode_s
                self._device_values[31] = duty_s
                self._device_values[32] = freq_s
                self._baseline_values[30] = mode_s
                self._baseline_values[31] = duty_s
                self._baseline_values[32] = freq_s
                self._set_widget_value(30, mode_s)
                self._set_widget_value(31, duty_s)
                self._set_widget_value(32, freq_s)
                self._refresh_dirty_states()
                self._emit_laser_sync_if_read()

        code_part = stripped.split(";", 1)[0].strip()
        m = _DOLLAR_LINE_RE.match(code_part)
        if m:
            pid = int(m.group(1))
            val_str = m.group(2)
            if pid in self._inputs or pid in (30, 31, 32):
                self._device_values[pid] = val_str
                self._baseline_values[pid] = val_str
                self._set_widget_value(pid, val_str)
                self._recalculate_derived_previews()
                self._refresh_dirty_states()
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
        if is_ack_match:
            if self._mode == "READING":
                self._cmd_timeout_timer.stop()
                self._mode = "IDLE"
                self._has_read_config = True
                self._refresh_dirty_states()
                self._update_controls_enabled()
                self._set_status(
                    f"Đã đồng bộ thành công cấu hình từ STM32 ({len(self._device_values)} thông số).",
                    "#15803d",
                )
                self._emit_laser_sync_if_read()
            elif self._mode == "WRITING":
                self._cmd_timeout_timer.stop()
                if self._current_write is not None:
                    pid, val = self._current_write
                    self._device_values[pid] = val
                    self._baseline_values[pid] = val
                    if pid == 30 and val == "1":
                        self._device_values[31] = "70.0"
                        self._baseline_values[31] = "70.0"
                    self._set_widget_value(pid, val)
                    self._write_applied_count += 1
                    self._refresh_dirty_states()
                    if pid in (30, 31, 32):
                        self._emit_laser_sync_if_read()
                self._send_next_in_write_queue()
            elif self._mode == "RESETTING":
                self._cmd_timeout_timer.stop()
                self._mode = "IDLE"
                self._update_controls_enabled()
                self._set_status("Đã khôi phục mặc định ($RST) thành công. Đang đọc lại $0...", "#15803d")
                if self._auto_refresh_after_op and self.serial_mgr.is_connected:
                    self._auto_refresh_after_op = False
                    self.read_config_from_device()
            return

        if upper_line.startswith("ERR:") or upper_line.startswith("ERROR"):
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
            self._refresh_dirty_states()
            self._update_controls_enabled()
            self._set_status(f"STM32 từ chối lệnh ({failed_cmd}): {stripped}", "#b91c1c")
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
        self._refresh_dirty_states()
        self._update_controls_enabled()
        self._set_status(f"Đã hủy chờ phản hồi ({prev_mode}) bởi người dùng.", "#64748b")

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
        self._refresh_dirty_states()
        self._update_controls_enabled()
        self._set_status(
            f"Quá thời gian chờ ({timeout_sec:.1f}s) phản hồi '{self._expected_ack()}' từ STM32 ({prev_mode}). Kiểm tra kết nối hoặc Simulation RX.",
            "#b91c1c",
        )