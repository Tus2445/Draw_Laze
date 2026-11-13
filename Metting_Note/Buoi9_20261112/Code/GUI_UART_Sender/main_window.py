from typing import List

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QCloseEvent
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from app_settings import AppSettings
from serial_manager import (
    COMMON_BAUDRATES,
    DEFAULT_BAUDRATE,
    DEFAULT_TIMEOUT,
    SerialManager,
    scan_serial_ports,
)
from tap_file_sender import TapFileSenderTab
from uart_terminal import UartTerminalTab


class MainWindow(QMainWindow):
    """Cửa sổ chính của ứng dụng UART Communication GUI."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("UART / TAP File Sender & Hercules Terminal Tool")
        self.resize(1100, 860)
        self.setMinimumSize(940, 680)

        self.settings = AppSettings()
        self.serial_mgr = SerialManager(self)
        self._known_port_devices: List[str] = []

        self._build_ui()
        self._connect_signals()

        # Quét cổng lần đầu khi mở ứng dụng, sau đó khôi phục lựa chọn đã lưu (nếu còn tồn tại)
        self.refresh_ports(initial=True)
        self._restore_saved_settings()

        # Tự động kiểm tra thiết bị cắm mới / rút ra mỗi 2.5 giây
        self._port_watch_timer = QTimer(self)
        self._port_watch_timer.setInterval(2500)
        self._port_watch_timer.timeout.connect(self._auto_poll_ports)
        self._port_watch_timer.start()

    def _build_ui(self) -> None:
        central = QWidget(self)
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(12, 12, 12, 12)
        root_layout.setSpacing(10)

        # --- Thanh công cụ kết nối dùng chung (Shared Connection Toolbar) ---
        toolbar_frame = QFrame()
        toolbar_frame.setObjectName("toolbarFrame")
        toolbar_frame.setStyleSheet(
            "#toolbarFrame { background-color: #f8f9fa; border: 1px solid #ced4da; border-radius: 6px; }"
        )
        tb_grid = QGridLayout(toolbar_frame)
        tb_grid.setContentsMargins(12, 10, 12, 10)
        tb_grid.setHorizontalSpacing(12)
        tb_grid.setVerticalSpacing(8)

        # Hàng 1: Chọn Cổng COM, Làm mới, Baudrate, Timeout (7 cột: 0..6)
        lbl_port = QLabel("<b>Cổng COM:</b>")
        self.port_combo = QComboBox()
        self.port_combo.setMinimumHeight(28)
        self.port_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        self.refresh_btn = QPushButton("Làm mới (Refresh)")
        self.refresh_btn.setMinimumHeight(28)
        self.refresh_btn.setMinimumWidth(140)
        self.refresh_btn.setToolTip("Quét lại danh sách cổng UART/Serial")
        self.refresh_btn.clicked.connect(lambda: self.refresh_ports(initial=False))

        lbl_baud = QLabel("<b>Baudrate:</b>")
        self.baud_combo = QComboBox()
        self.baud_combo.setEditable(True)
        self.baud_combo.setMinimumHeight(28)
        self.baud_combo.setMinimumWidth(115)
        for b in COMMON_BAUDRATES:
            self.baud_combo.addItem(str(b), b)
        self.baud_combo.setCurrentText(str(DEFAULT_BAUDRATE))

        lbl_timeout = QLabel("<b>Timeout (s):</b>")
        self.timeout_spin = QDoubleSpinBox()
        self.timeout_spin.setRange(0.5, 60.0)
        self.timeout_spin.setSingleStep(0.5)
        self.timeout_spin.setValue(DEFAULT_TIMEOUT)
        self.timeout_spin.setMinimumHeight(28)
        self.timeout_spin.setMinimumWidth(85)
        self.timeout_spin.setToolTip(
            "Thời gian chờ tối đa cho phản hồi 'ok' của mỗi lệnh .tap"
        )
        self.timeout_spin.valueChanged.connect(self.serial_mgr.set_timeout)
        self.timeout_spin.valueChanged.connect(self.settings.set_last_timeout)

        tb_grid.addWidget(lbl_port, 0, 0)
        tb_grid.addWidget(self.port_combo, 0, 1)
        tb_grid.addWidget(self.refresh_btn, 0, 2)
        tb_grid.addWidget(lbl_baud, 0, 3)
        tb_grid.addWidget(self.baud_combo, 0, 4)
        tb_grid.addWidget(lbl_timeout, 0, 5)
        tb_grid.addWidget(self.timeout_spin, 0, 6)
        tb_grid.setColumnStretch(1, 1)

        # Hàng 2: Nút Kết nối, Ngắt kết nối, Chế độ mô phỏng và Nhãn trạng thái
        action_row = QHBoxLayout()
        action_row.setSpacing(12)

        self.connect_btn = QPushButton("Kết nối (Connect)")
        self.connect_btn.setMinimumHeight(32)
        self.connect_btn.setMinimumWidth(150)
        self.connect_btn.setStyleSheet(
            "QPushButton { background-color: #0d6efd; color: white; font-weight: bold; "
            "padding: 4px 16px; border-radius: 4px; }"
            "QPushButton:disabled { background-color: #9ec5fe; color: #ffffff; }"
        )
        self.connect_btn.clicked.connect(self.on_connect_clicked)

        self.disconnect_btn = QPushButton("Ngắt kết nối (Disconnect)")
        self.disconnect_btn.setMinimumHeight(32)
        self.disconnect_btn.setMinimumWidth(180)
        self.disconnect_btn.setStyleSheet(
            "QPushButton { padding: 4px 16px; font-weight: bold; }"
        )
        self.disconnect_btn.setEnabled(False)
        self.disconnect_btn.clicked.connect(self.on_disconnect_clicked)

        self.simulation_mode_chk = QCheckBox("Chế độ mô phỏng (Simulation Mode)")
        self.simulation_mode_chk.setToolTip(
            "Kết nối tới một thiết bị UART giả lập (không cần phần cứng thật) "
            "để kiểm thử TAP Sender và Terminal."
        )
        self.simulation_mode_chk.toggled.connect(self._on_simulation_mode_toggled)

        self.conn_status_badge = QLabel("● Chưa kết nối")
        self.conn_status_badge.setAlignment(Qt.AlignCenter)
        self.conn_status_badge.setMinimumHeight(30)
        self.conn_status_badge.setMinimumWidth(240)
        self.conn_status_badge.setStyleSheet(
            "font-weight: bold; color: #dc3545; padding: 4px 14px; "
            "background: #f8d7da; border: 1px solid #f5c2c7; border-radius: 4px;"
        )

        action_row.addWidget(self.connect_btn)
        action_row.addWidget(self.disconnect_btn)
        action_row.addSpacing(8)
        action_row.addWidget(self.simulation_mode_chk)
        action_row.addStretch()
        action_row.addWidget(QLabel("<b>Trạng thái cổng:</b>"))
        action_row.addWidget(self.conn_status_badge)

        tb_grid.addLayout(action_row, 1, 0, 1, 7)

        # Hàng 3 (Ẩn mặc định): Khung điều khiển kịch bản lỗi chia 2 dòng rộng rãi
        self.sim_ctrl_frame = QFrame()
        self.sim_ctrl_frame.setObjectName("simCtrlFrame")
        self.sim_ctrl_frame.setStyleSheet(
            "#simCtrlFrame { background-color: #fff3cd; border: 1px solid #ffecb5; border-radius: 4px; }"
        )
        sim_grid = QGridLayout(self.sim_ctrl_frame)
        sim_grid.setContentsMargins(10, 8, 10, 8)
        sim_grid.setHorizontalSpacing(10)
        sim_grid.setVerticalSpacing(6)

        # Dòng 1 của khung mô phỏng: Nhập tự do nội dung RX + tùy chọn CRLF + nút Gửi RX
        sim_row1 = QHBoxLayout()
        sim_row1.setSpacing(10)
        sim_row1.addWidget(QLabel("<b>🛠 Giả lập thiết bị gửi về (RX):</b>"))

        self.sim_custom_rx_edit = QLineEdit()
        self.sim_custom_rx_edit.setPlaceholderText(
            "Nhập đúng nội dung RX muốn giả lập (vd: ok, done, error:20, <Idle|MPos:0,0,0>) rồi nhấn Enter..."
        )
        self.sim_custom_rx_edit.setMinimumHeight(28)
        self.sim_custom_rx_edit.returnPressed.connect(self._on_sim_send_rx)
        sim_row1.addWidget(self.sim_custom_rx_edit, stretch=1)

        self.sim_auto_crlf_chk = QCheckBox("Tự thêm CRLF")
        self.sim_auto_crlf_chk.setChecked(True)
        self.sim_auto_crlf_chk.setToolTip(
            "Tự động nối \\r\\n vào cuối nội dung RX nếu chưa có. Bỏ chọn để gửi "
            "chính xác từng byte đã nhập, không thêm bớt gì."
        )
        sim_row1.addWidget(self.sim_auto_crlf_chk)

        self.sim_send_rx_btn = QPushButton("Gửi RX")
        self.sim_send_rx_btn.setMinimumHeight(28)
        self.sim_send_rx_btn.setMinimumWidth(110)
        self.sim_send_rx_btn.clicked.connect(self._on_sim_send_rx)
        sim_row1.addWidget(self.sim_send_rx_btn)

        sim_grid.addLayout(sim_row1, 0, 0)

        # Dòng 2 của khung mô phỏng: Nút gửi nhanh các phản hồi thường dùng
        sim_row2 = QHBoxLayout()
        sim_row2.setSpacing(10)
        sim_row2.addWidget(QLabel("Gửi nhanh:"))

        self.sim_quick_ok_btn = QPushButton("Gửi ok")
        self.sim_quick_ok_btn.setMinimumHeight(28)
        self.sim_quick_ok_btn.setMinimumWidth(90)
        self.sim_quick_ok_btn.clicked.connect(lambda: self._on_sim_quick_send("ok"))
        sim_row2.addWidget(self.sim_quick_ok_btn)

        self.sim_quick_error_btn = QPushButton("Gửi error:20")
        self.sim_quick_error_btn.setMinimumHeight(28)
        self.sim_quick_error_btn.setMinimumWidth(110)
        self.sim_quick_error_btn.clicked.connect(lambda: self._on_sim_quick_send("error:20"))
        sim_row2.addWidget(self.sim_quick_error_btn)

        self.sim_quick_alarm_btn = QPushButton("Gửi ALARM:1")
        self.sim_quick_alarm_btn.setMinimumHeight(28)
        self.sim_quick_alarm_btn.setMinimumWidth(110)
        self.sim_quick_alarm_btn.clicked.connect(lambda: self._on_sim_quick_send("ALARM:1"))
        sim_row2.addWidget(self.sim_quick_alarm_btn)

        sim_hint = QLabel(
            "Không có phản hồi tự động nào được gửi — nếu không bấm Gửi RX, "
            "lệnh đang chờ sẽ Timeout như với thiết bị thật."
        )
        sim_hint.setStyleSheet("color: #664d03; font-style: italic;")
        sim_row2.addWidget(sim_hint)
        sim_row2.addStretch()

        sim_grid.addLayout(sim_row2, 1, 0)

        self.sim_ctrl_frame.setVisible(False)
        tb_grid.addWidget(self.sim_ctrl_frame, 2, 0, 1, 7)

        root_layout.addWidget(toolbar_frame)

        # --- Hai Tab chức năng chính ---
        self.tabs = QTabWidget()
        self.tap_tab = TapFileSenderTab(
            self.serial_mgr, settings=self.settings, parent=self
        )
        self.terminal_tab = UartTerminalTab(
            self.serial_mgr, settings=self.settings, parent=self
        )

        self.tabs.addTab(self.tap_tab, "Tab 1: TAP File Sender")
        self.tabs.addTab(self.terminal_tab, "Tab 2: UART Terminal (Hercules)")
        root_layout.addWidget(self.tabs, stretch=1)

    def _connect_signals(self) -> None:
        self.serial_mgr.connection_changed.connect(self._on_connection_changed)
        self.serial_mgr.port_error.connect(self._on_port_error)
        self.tap_tab.request_auto_connect.connect(self.on_connect_clicked)
        self.tap_tab.transmission_state_changed.connect(
            self.terminal_tab.set_tap_transmission_active
        )
        self.port_combo.currentIndexChanged.connect(self._save_port_selection)
        self.baud_combo.currentTextChanged.connect(self._save_baudrate_selection)

    def _on_sim_send_rx(self) -> None:
        sim = self.serial_mgr.simulated_port
        if sim is None:
            QMessageBox.information(
                self, "Chưa kết nối mô phỏng", "Hãy bấm 'Kết nối' ở Chế độ mô phỏng trước."
            )
            return
        text = self.sim_custom_rx_edit.text()
        if not text:
            return
        if self.sim_auto_crlf_chk.isChecked() and not text.endswith(("\r", "\n")):
            text += "\r\n"
        sim.inject_rx(text)
        self.sim_custom_rx_edit.clear()

    def _on_sim_quick_send(self, response_text: str) -> None:
        sim = self.serial_mgr.simulated_port
        if sim is None:
            QMessageBox.information(
                self, "Chưa kết nối mô phỏng", "Hãy bấm 'Kết nối' ở Chế độ mô phỏng trước."
            )
            return
        sim.inject_rx(response_text + "\r\n")

    def _save_port_selection(self, _index: int) -> None:
        device = self.port_combo.currentData()
        if device:
            self.settings.set_last_port(device)

    def _save_baudrate_selection(self, text: str) -> None:
        try:
            self.settings.set_last_baudrate(int(text.strip()))
        except ValueError:
            pass

    def _restore_saved_settings(self) -> None:
        saved_baud = self.settings.last_baudrate(DEFAULT_BAUDRATE)
        self.baud_combo.setCurrentText(str(saved_baud))

        saved_timeout = self.settings.last_timeout(DEFAULT_TIMEOUT)
        self.timeout_spin.blockSignals(True)
        self.timeout_spin.setValue(saved_timeout)
        self.timeout_spin.blockSignals(False)
        self.serial_mgr.set_timeout(saved_timeout)

        saved_port = self.settings.last_port()
        if saved_port and saved_port in self._known_port_devices:
            idx = self.port_combo.findData(saved_port)
            if idx >= 0:
                self.port_combo.setCurrentIndex(idx)

        geometry = self.settings.window_geometry()
        if geometry:
            self.restoreGeometry(geometry)

    def refresh_ports(self, initial: bool = False) -> None:
        ports, recommended = scan_serial_ports()
        self._known_port_devices = [p.device for p in ports]

        previous_selection = self.port_combo.currentData()
        self.port_combo.blockSignals(True)
        self.port_combo.clear()

        if not ports:
            self.port_combo.addItem("Không phát hiện cổng COM nào", "")
            self.connect_btn.setEnabled(bool(self.simulation_mode_chk.isChecked()))
        else:
            for p in ports:
                self.port_combo.addItem(p.display_name, p.device)
            if not self.serial_mgr.is_connected:
                self.connect_btn.setEnabled(True)

            target_device = (
                previous_selection
                if previous_selection in self._known_port_devices
                else recommended
            )
            if target_device:
                idx = self.port_combo.findData(target_device)
                if idx >= 0:
                    self.port_combo.setCurrentIndex(idx)

        self.port_combo.blockSignals(False)

    def _auto_poll_ports(self) -> None:
        if self.serial_mgr.is_connected:
            return
        ports, _ = scan_serial_ports()
        current_devices = [p.device for p in ports]
        if current_devices != self._known_port_devices:
            self.refresh_ports(initial=False)

    def on_connect_clicked(self) -> None:
        if self.serial_mgr.is_connected:
            return

        timeout = self.timeout_spin.value()

        if self.simulation_mode_chk.isChecked():
            self.connect_btn.setEnabled(False)
            self.serial_mgr.connect_simulated(timeout=timeout)
            return

        port = self.port_combo.currentData()
        if not port:
            QMessageBox.warning(
                self,
                "Không có cổng Serial",
                "Không tìm thấy cổng UART/Serial hợp lệ. Vui lòng kiểm tra cáp USB-UART và nhấn Làm mới.",
            )
            return

        try:
            baudrate = int(self.baud_combo.currentText().strip())
        except ValueError:
            QMessageBox.warning(
                self,
                "Baudrate không hợp lệ",
                "Vui lòng chọn hoặc nhập số Baudrate hợp lệ.",
            )
            return

        self.connect_btn.setEnabled(False)
        self.serial_mgr.connect_port(port=port, baudrate=baudrate, timeout=timeout)

    def _on_simulation_mode_toggled(self, checked: bool) -> None:
        if self.serial_mgr.is_connected:
            self.simulation_mode_chk.blockSignals(True)
            self.simulation_mode_chk.setChecked(not checked)
            self.simulation_mode_chk.blockSignals(False)
            QMessageBox.warning(
                self,
                "Đang kết nối",
                "Vui lòng ngắt kết nối trước khi chuyển đổi Chế độ mô phỏng.",
            )
            return
        self.port_combo.setEnabled(not checked)
        self.baud_combo.setEnabled(not checked)
        self.refresh_btn.setEnabled(not checked)
        self.sim_ctrl_frame.setVisible(checked)
        self.connect_btn.setEnabled(
            checked or bool(self.port_combo.currentData())
        )

    def on_disconnect_clicked(self) -> None:
        if self.tap_tab.is_transmitting:
            ans = QMessageBox.question(
                self,
                "Đang truyền file .TAP",
                "Hệ thống đang truyền file .tap. Ngắt kết nối sẽ dừng quá trình truyền ngay lập tức.\nBạn có muốn tiếp tục ngắt kết nối?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if ans != QMessageBox.Yes:
                return
            self.tap_tab.force_stop_worker()

        self.serial_mgr.disconnect_port("Đã ngắt kết nối bởi người dùng")

    def _on_connection_changed(self, is_connected: bool, message: str) -> None:
        is_sim = self.serial_mgr.is_simulated
        self.port_combo.setEnabled(
            not is_connected and not self.simulation_mode_chk.isChecked()
        )
        self.baud_combo.setEnabled(
            not is_connected and not self.simulation_mode_chk.isChecked()
        )
        self.refresh_btn.setEnabled(
            not is_connected and not self.simulation_mode_chk.isChecked()
        )
        self.simulation_mode_chk.setEnabled(not is_connected)
        self.connect_btn.setEnabled(
            not is_connected
            and (
                self.simulation_mode_chk.isChecked()
                or bool(self.port_combo.currentData())
            )
        )
        self.disconnect_btn.setEnabled(is_connected)

        # Nhóm điều khiển Gửi RX thủ công chỉ dùng được khi THỰC SỰ đã kết nối ở
        # Chế độ mô phỏng (không chỉ khi checkbox được tick) — tránh gửi nhầm vào cổng thật.
        self.sim_ctrl_frame.setEnabled(is_connected and is_sim)
        if not is_connected:
            self.sim_custom_rx_edit.clear()

        badge_prefix = "● [MÔ PHỎNG] " if is_sim else "● "
        if is_connected:
            self.conn_status_badge.setText(f"{badge_prefix}{message}")
            self.conn_status_badge.setStyleSheet(
                "font-weight: bold; color: #0f5132; padding: 4px 14px; "
                "background: #d1e7dd; border: 1px solid #badbcc; border-radius: 4px;"
            )
        else:
            self.conn_status_badge.setText(f"{badge_prefix}{message}")
            self.conn_status_badge.setStyleSheet(
                "font-weight: bold; color: #dc3545; padding: 4px 14px; "
                "background: #f8d7da; border: 1px solid #f5c2c7; border-radius: 4px;"
            )
            self.refresh_ports(initial=False)

    def _on_port_error(self, error_msg: str) -> None:
        self.tap_tab._append_log("ERR", error_msg)

    def closeEvent(self, event: QCloseEvent) -> None:
        if self.tap_tab.is_transmitting:
            ans = QMessageBox.question(
                self,
                "Xác nhận thoát",
                "Quá trình truyền file .TAP đang diễn ra. Bạn có chắc muốn dừng truyền và đóng ứng dụng?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if ans != QMessageBox.Yes:
                event.ignore()
                return
            self.tap_tab.force_stop_worker()

        self._port_watch_timer.stop()
        self.terminal_tab.shutdown()
        if self.serial_mgr.is_connected:
            self.serial_mgr.disconnect_port("Đóng ứng dụng")

        self.settings.set_window_geometry(self.saveGeometry())
        self.settings.sync()
        event.accept()