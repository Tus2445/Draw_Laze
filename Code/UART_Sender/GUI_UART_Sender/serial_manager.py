import threading
import time
from dataclasses import dataclass
from typing import Callable, List, Optional

import serial
import serial.tools.list_ports
from PySide6.QtCore import QObject, Signal

DEFAULT_BAUDRATE = 115200
DEFAULT_TIMEOUT = 5.0
COMMON_BAUDRATES = [9600, 19200, 38400, 57600, 115200, 230400, 460800]
PRIORITY_KEYWORDS = ["usb", "ch340", "cp210", "ftdi", "arduino", "serial"]


@dataclass
class PortInfo:
    device: str
    description: str
    hwid: str
    is_preferred: bool

    @property
    def display_name(self) -> str:
        tag = " [Ưu tiên]" if self.is_preferred else ""
        desc = self.description if self.description else "Unknown Device"
        return f"{self.device} — {desc}{tag}"


def scan_serial_ports() -> tuple[List[PortInfo], Optional[str]]:
    """
    Quét danh sách cổng COM khả dụng, bảo toàn logic ưu tiên của find_serial_port()
    trong test_UART.py và trả về (danh_sách_cổng, cổng_đề_xuất).
    """
    raw_ports = list(serial.tools.list_ports.comports())
    port_list: List[PortInfo] = []
    recommended_port: Optional[str] = None

    for p in raw_ports:
        desc_lower = (p.description or "").lower()
        hwid_lower = (p.hwid or "").lower()
        is_pref = any(
            kw in desc_lower or kw in hwid_lower for kw in PRIORITY_KEYWORDS
        )
        info = PortInfo(
            device=p.device,
            description=p.description or "",
            hwid=p.hwid or "",
            is_preferred=is_pref,
        )
        port_list.append(info)
        if is_pref and recommended_port is None:
            recommended_port = p.device

    if recommended_port is None and port_list:
        recommended_port = port_list[0].device

    return port_list, recommended_port


class SimulatedSerialPort:
    """
    Cổng Serial giả lập (không cần phần cứng thật), tuân theo nguyên tắc Manual RX:
    write() CHỈ ghi nhận dữ liệu TX, không bao giờ tự sinh phản hồi (không 'ok', không
    lỗi/alarm, không độ trễ giả lập). Toàn bộ dữ liệu RX phải được đưa vào thông qua
    inject_rx() — thường là do người dùng chủ động nhập ở UI. Triển khai interface tối
    thiểu của pyserial.Serial mà SerialManager._reader_loop() (luồng đọc nền duy nhất) cần.
    """

    def __init__(self, port: str = "SIM", baudrate: int = 115200, **kwargs):
        self.port = port
        self.baudrate = baudrate
        self.is_open = True
        self.timeout = kwargs.get("timeout", 0.05)
        self.write_timeout = kwargs.get("write_timeout")

        self._lock = threading.Lock()
        self._rx_buffer = bytearray()
        # Lưu lại dữ liệu TX đã nhận để quan sát/kiểm thử (không ảnh hưởng RX).
        self.tx_log: List[bytes] = []

    @property
    def in_waiting(self) -> int:
        with self._lock:
            return len(self._rx_buffer)

    def read(self, size: int = 1) -> bytes:
        with self._lock:
            if not self._rx_buffer:
                return b""
            chunk = bytes(self._rx_buffer[:size])
            del self._rx_buffer[:size]
            return chunk

    def inject_rx(self, data) -> None:
        """Đưa dữ liệu RX do người dùng chủ động nhập vào buffer nhận (API Manual RX)."""
        if not self.is_open:
            return
        if isinstance(data, str):
            data = data.encode("latin-1", errors="replace")
        with self._lock:
            self._rx_buffer += data

    def write(self, data: bytes) -> int:
        """Chỉ ghi nhận dữ liệu TX; KHÔNG tự động sinh bất kỳ phản hồi RX nào."""
        with self._lock:
            self.tx_log.append(bytes(data))
        return len(data)

    def flush(self) -> None:
        pass

    def reset_input_buffer(self) -> None:
        with self._lock:
            self._rx_buffer.clear()

    def reset_output_buffer(self) -> None:
        with self._lock:
            self.tx_log.clear()

    def close(self) -> None:
        with self._lock:
            self._rx_buffer.clear()
        self.is_open = False


class SerialManager(QObject):
    """
    Chủ sở hữu duy nhất của kết nối Serial.
    Phát tín hiệu thread-safe cho cả TAP File Sender và UART Terminal.
    """

    connection_changed = Signal(bool, str)  # (is_connected, status_message)
    raw_data_received = Signal(bytes)       # Dữ liệu thô cho UART Terminal
    line_received = Signal(str)             # Dữ liệu từng dòng cho TAP Sender & Log
    data_sent = Signal(bytes, str)          # (raw_bytes, source_tag: 'TAP' | 'TERM')
    port_error = Signal(str)                # Thông báo lỗi kết nối / rút cáp đột ngột

    def __init__(
        self,
        parent: Optional[QObject] = None,
        serial_factory: Optional[Callable[..., serial.Serial]] = None,
    ):
        super().__init__(parent)
        self._serial_factory = serial_factory or serial.Serial
        self._ser: Optional[serial.Serial] = None
        self._write_lock = threading.Lock()
        self._reader_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._line_buffer = ""
        self.is_simulated: bool = False

        self._port: str = ""
        self._baudrate: int = DEFAULT_BAUDRATE
        self._timeout: float = DEFAULT_TIMEOUT

    @property
    def is_connected(self) -> bool:
        return self._ser is not None and bool(self._ser.is_open)

    @property
    def current_port(self) -> str:
        return self._port if self.is_connected else ""

    @property
    def current_baudrate(self) -> int:
        return self._baudrate

    @property
    def current_timeout(self) -> float:
        return self._timeout

    @property
    def simulated_port(self) -> Optional[SimulatedSerialPort]:
        """Trả về đối tượng SimulatedSerialPort đang dùng (nếu đang ở Chế độ mô phỏng)."""
        if self.is_simulated and isinstance(self._ser, SimulatedSerialPort):
            return self._ser
        return None

    def set_timeout(self, timeout_sec: float) -> None:
        self._timeout = max(0.1, float(timeout_sec))
        if self._ser and self._ser.is_open:
            try:
                self._ser.timeout = self._timeout
            except Exception:
                pass

    def connect_simulated(
        self, timeout: float = DEFAULT_TIMEOUT
    ) -> tuple[bool, str]:
        """Kết nối tới một SimulatedSerialPort thay vì cổng COM thật."""
        original_factory = self._serial_factory
        self._serial_factory = SimulatedSerialPort
        try:
            ok, msg = self.connect_port(
                port="SIM",
                baudrate=115200,
                timeout=timeout,
                is_simulated_target=True,
            )
        finally:
            self._serial_factory = original_factory
        return ok, msg

    def connect_port(
        self,
        port: str,
        baudrate: int = DEFAULT_BAUDRATE,
        timeout: float = DEFAULT_TIMEOUT,
        is_simulated_target: bool = False,
    ) -> tuple[bool, str]:
        """Mở cổng Serial và khởi chạy luồng đọc nền duy nhất."""
        if self.is_connected:
            self.disconnect_port()

        if not port:
            msg = "Chưa chọn cổng COM."
            self.port_error.emit(msg)
            return False, msg

        try:
            baud = int(baudrate)
            if baud <= 0:
                raise ValueError("Baudrate phải là số nguyên dương.")
        except ValueError as exc:
            msg = f"Baudrate không hợp lệ: {exc}"
            self.port_error.emit(msg)
            return False, msg

        self._port = port
        self._baudrate = baud
        self._timeout = max(0.1, float(timeout))

        try:
            self._ser = self._serial_factory(
                port=self._port,
                baudrate=self._baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05,
                write_timeout=2.0,
            )
        except Exception as exc:
            self._ser = None
            self.is_simulated = False
            msg = f"Không thể mở cổng {port} @ {baudrate}: {exc}"
            self.port_error.emit(msg)
            self.connection_changed.emit(False, "Kết nối thất bại")
            return False, msg

        self.is_simulated = bool(
            is_simulated_target or isinstance(self._ser, SimulatedSerialPort)
        )
        self._line_buffer = ""
        self._stop_event.clear()
        self._reader_thread = threading.Thread(
            target=self._reader_loop, name="UARTReaderThread", daemon=True
        )
        self._reader_thread.start()

        status_msg = f"Đã kết nối {self._port} @ {self._baudrate} bps"
        self.connection_changed.emit(True, status_msg)
        return True, status_msg

    def disconnect_port(self, reason: str = "Đã ngắt kết nối") -> None:
        """Đóng cổng Serial an toàn và dừng luồng đọc."""
        self._stop_event.set()
        if (
            self._reader_thread
            and self._reader_thread.is_alive()
            and threading.current_thread() is not self._reader_thread
        ):
            self._reader_thread.join(timeout=1.0)
        self._reader_thread = None

        with self._write_lock:
            if self._ser is not None:
                try:
                    if self._ser.is_open:
                        self._ser.close()
                except Exception:
                    pass
                finally:
                    self._ser = None

        self._line_buffer = ""
        self.is_simulated = False
        self.connection_changed.emit(False, reason)

    def reset_buffers(self) -> None:
        """Xóa bộ đệm vào/ra của cổng Serial (giống logic khởi tạo trong test_UART.py)."""
        with self._write_lock:
            if self._ser and self._ser.is_open:
                try:
                    self._ser.reset_input_buffer()
                    self._ser.reset_output_buffer()
                    self._line_buffer = ""
                except Exception as exc:
                    self.port_error.emit(f"Không thể xóa buffer UART: {exc}")

    def write_bytes(self, data: bytes, source: str = "TERM") -> bool:
        """Gửi mảng byte qua UART một cách thread-safe."""
        if not data:
            return False
        if not self.is_connected or self._ser is None:
            self.port_error.emit("Không thể gửi dữ liệu: Chưa kết nối cổng UART.")
            return False

        with self._write_lock:
            try:
                self._ser.write(data)
                self._ser.flush()
            except Exception as exc:
                err_msg = f"Lỗi truyền UART (có thể thiết bị đã bị rút cáp): {exc}"
                self.port_error.emit(err_msg)
                threading.Thread(
                    target=self.disconnect_port,
                    args=("Mất kết nối đột ngột",),
                    daemon=True,
                ).start()
                return False

        self.data_sent.emit(data, source)
        return True

    def _reader_loop(self) -> None:
        """Luồng nền duy nhất đọc dữ liệu từ cổng Serial."""
        while not self._stop_event.is_set():
            ser = self._ser
            if ser is None or not ser.is_open:
                break

            try:
                waiting = ser.in_waiting
                chunk = ser.read(waiting if waiting > 0 else 1)
            except Exception as exc:
                if not self._stop_event.is_set():
                    self.port_error.emit(
                        f"Mất kết nối với cổng {self._port}: {exc}"
                    )
                    self._stop_event.set()
                    with self._write_lock:
                        try:
                            if self._ser and self._ser.is_open:
                                self._ser.close()
                        except Exception:
                            pass
                        self._ser = None
                    self.is_simulated = False
                    self.connection_changed.emit(False, "Mất kết nối thiết bị")
                break

            if chunk:
                # 1. Phát dữ liệu thô tức thì cho UART Terminal
                self.raw_data_received.emit(chunk)

                # 2. Tách dòng theo '\n' hoặc '\r' cho bộ xử lý lệnh TAP và log
                decoded = chunk.decode("latin-1", errors="ignore")
                self._line_buffer += decoded
                while "\n" in self._line_buffer or "\r" in self._line_buffer:
                    idx_n = self._line_buffer.find("\n")
                    idx_r = self._line_buffer.find("\r")

                    if idx_r != -1 and (idx_n == -1 or idx_r < idx_n):
                        if idx_n == idx_r + 1:
                            line = self._line_buffer[:idx_r]
                            self._line_buffer = self._line_buffer[idx_n + 1 :]
                        else:
                            line = self._line_buffer[:idx_r]
                            self._line_buffer = self._line_buffer[idx_r + 1 :]
                    else:
                        line = self._line_buffer[:idx_n]
                        self._line_buffer = self._line_buffer[idx_n + 1 :]

                    clean_resp = line.strip()
                    if clean_resp:
                        self.line_received.emit(clean_resp)
            else:
                time.sleep(0.005)