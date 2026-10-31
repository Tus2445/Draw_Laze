import os
import shutil
import tempfile
import time
import unittest

from PySide6.QtWidgets import QApplication

from app_settings import AppSettings
from log_export import LogExportWorker
from serial_manager import SerialManager, SimulatedSerialPort
from tap_file_sender import (
    ParsedTapCommand,
    TapFileSenderTab,
    TapSenderWorker,
    classify_response,
    clean_gcode_line,
    parse_tap_file,
)
from uart_terminal import UartTerminalTab, format_terminal_payload

# UartTerminalTab là QWidget nên cần một QApplication còn sống trong suốt tiến trình test.
_qapp = QApplication.instance() or QApplication([])


class FakeSerialDevice:
    """Mock Serial Transport dùng để kiểm thử logic đọc/ghi không cần phần cứng."""

    def __init__(self, port, baudrate, **kwargs):
        self.port = port
        self.baudrate = baudrate
        self.is_open = True
        self.written_payloads = []
        self._rx_bytes = b"Grbl 1.1h ['$' for help]\r\nok\r\n"

    @property
    def in_waiting(self) -> int:
        return len(self._rx_bytes)

    def read(self, size: int = 1) -> bytes:
        if not self._rx_bytes:
            return b""
        chunk = self._rx_bytes[:size]
        self._rx_bytes = self._rx_bytes[size:]
        return chunk

    def write(self, data: bytes) -> int:
        self.written_payloads.append(data)
        self._rx_bytes += b"ok\r\n"
        return len(data)

    def flush(self) -> None:
        pass

    def reset_input_buffer(self) -> None:
        pass

    def reset_output_buffer(self) -> None:
        pass

    def close(self) -> None:
        self.is_open = False


class TestUartGuiCoreLogic(unittest.TestCase):
    def test_clean_gcode_line_preserves_original_logic(self):
        self.assertEqual(clean_gcode_line("  G0 X10 Y20 ; di chuyen nhanh "), "G0 X10 Y20")
        self.assertEqual(clean_gcode_line("(Bat dau chuong trinh)"), "")
        self.assertEqual(clean_gcode_line("G1 X5.5 (cat ngang) F200"), "G1 X5.5 F200")
        self.assertEqual(
            clean_gcode_line("  M3 S1000 (Bat truc chinh) ; comment cuoi dong\n"),
            "M3 S1000",
        )
        self.assertEqual(clean_gcode_line("   \r\n"), "")

    def test_clean_gcode_line_multiple_parenthesized_comments(self):
        self.assertEqual(
            clean_gcode_line("G1 X5.5 (horizontal movement) F200"), "G1 X5.5 F200"
        )
        self.assertEqual(
            clean_gcode_line("G1 X1 (first) Y2 (second) F300"), "G1 X1 Y2 F300"
        )
        self.assertEqual(
            clean_gcode_line("(start) G0 X0 (mid) Y0 (end)"), "G0 X0 Y0"
        )

    def test_clean_gcode_line_normalizes_excess_whitespace(self):
        self.assertEqual(clean_gcode_line("G0    X1     Y2"), "G0 X1 Y2")
        self.assertEqual(clean_gcode_line("   G1   Z5   "), "G1 Z5")

    def test_parse_tap_file(self):
        sample_content = (
            "(Header comment)\n"
            "; Full line semicolon comment\n"
            "\n"
            "G21 ; Millimeters\n"
            "G90 (Absolute positioning)\n"
            "G0 X0 Y0\n"
        )
        with tempfile.NamedTemporaryFile(
            "w", suffix=".tap", encoding="utf-8", delete=False
        ) as tmp:
            tmp.write(sample_content)
            tmp_path = tmp.name

        try:
            total_lines, cmds = parse_tap_file(tmp_path)
            self.assertEqual(total_lines, 6)
            self.assertEqual(len(cmds), 3)
            self.assertEqual(cmds[0].line_number, 4)
            self.assertEqual(cmds[0].cleaned_command, "G21")
            self.assertEqual(cmds[1].line_number, 5)
            self.assertEqual(cmds[1].cleaned_command, "G90")
            self.assertEqual(cmds[2].line_number, 6)
            self.assertEqual(cmds[2].cleaned_command, "G0 X0 Y0")
        finally:
            os.remove(tmp_path)

    def test_terminal_line_endings_and_auto_lf(self):
        self.assertEqual(
            format_terminal_payload("G0 X10", line_ending_mode="LF", auto_lf_enabled=True),
            b"G0 X10\n",
        )
        self.assertEqual(
            format_terminal_payload("G0 X10", line_ending_mode="LF", auto_lf_enabled=False),
            b"G0 X10",
        )
        self.assertEqual(
            format_terminal_payload("G0 X10\n", line_ending_mode="LF", auto_lf_enabled=True),
            b"G0 X10\n",
        )
        self.assertEqual(
            format_terminal_payload("$$", line_ending_mode="CR", auto_lf_enabled=False),
            b"$$\r",
        )
        self.assertEqual(
            format_terminal_payload("$$", line_ending_mode="CRLF", auto_lf_enabled=False),
            b"$$\r\n",
        )
        self.assertEqual(
            format_terminal_payload(
                "47 30", line_ending_mode="LF", auto_lf_enabled=True, is_hex_mode=True
            ),
            b"G0\n",
        )

    def test_classify_response(self):
        self.assertEqual(classify_response("ok"), "ok")
        self.assertEqual(classify_response("OK"), "ok")
        self.assertEqual(classify_response("error:20"), "error")
        self.assertEqual(classify_response("ALARM:1"), "alarm")
        self.assertEqual(classify_response("Token valid", strict_ack=True), "info")
        self.assertEqual(classify_response("Grbl 1.1h ['$' for help]", strict_ack=True), "info")

    def test_classify_response_custom_ack_value(self):
        self.assertEqual(classify_response("done", ack_value="done"), "ok")
        self.assertEqual(classify_response("DONE:1", ack_value="done"), "ok")
        self.assertEqual(classify_response("ready", ack_value="ready"), "ok")
        self.assertEqual(classify_response("ok", ack_value="done"), "info")
        self.assertEqual(classify_response("error:1", ack_value="done"), "error")


class TestUartTerminalPartialBuffer(unittest.TestCase):
    """Kiểm thử cơ chế xả buffer dở dang của UART Terminal (CR/LF/CRLF + idle timer + disconnect)."""

    def setUp(self):
        self.serial_mgr = SerialManager(serial_factory=FakeSerialDevice)
        self.tab = UartTerminalTab(self.serial_mgr)

    def tearDown(self):
        self.tab.shutdown()

    def _displayed_text(self) -> str:
        return self.tab.terminal_output.toPlainText()

    def test_cr_terminated_message_flushes_immediately(self):
        self.tab._on_raw_data_received(b"ok\r")
        self.assertIn("ok", self._displayed_text())
        self.assertEqual(self.tab._rx_partial_buffer, "")

    def test_lf_terminated_message_flushes_immediately(self):
        self.tab._on_raw_data_received(b"Hello\n")
        self.assertIn("Hello", self._displayed_text())
        self.assertEqual(self.tab._rx_partial_buffer, "")

    def test_crlf_terminated_message_flushes_once_without_blank_line(self):
        self.tab._on_raw_data_received(b"Hello\r\n")
        text = self._displayed_text()
        self.assertEqual(text.count("Hello"), 1)
        self.assertEqual(self.tab._rx_partial_buffer, "")

    def test_message_without_line_ending_waits_for_idle_flush(self):
        self.tab._on_raw_data_received(b"Ready...")
        self.assertNotIn("Ready...", self._displayed_text())
        self.assertEqual(self.tab._rx_partial_buffer, "Ready...")

        self.tab._rx_last_data_time = time.monotonic() - 1.0
        self.tab._flush_idle_partial_buffer()

        self.assertIn("Ready...", self._displayed_text())
        self.assertEqual(self.tab._rx_partial_buffer, "")

    def test_disconnect_flushes_partial_buffer_immediately_even_before_idle_timeout(self):
        self.tab._on_raw_data_received(b"LastBytesBeforeUnplug")
        # Ngắt kết nối ngay lập tức khi chưa tới 70ms
        self.tab._on_connection_changed(False, "Disconnected")
        self.assertIn("LastBytesBeforeUnplug", self._displayed_text())
        self.assertEqual(self.tab._rx_partial_buffer, "")

    def test_clear_terminal_clears_export_records(self):
        self.tab._on_raw_data_received(b"Line1\r\n")
        self.assertEqual(len(self.tab._records), 1)
        self.tab.clear_terminal()
        self.assertEqual(len(self.tab._records), 0)

    def test_fragmented_chunks_do_not_lose_or_duplicate_bytes(self):
        self.tab._on_raw_data_received(b"Hel")
        self.tab._on_raw_data_received(b"lo\r\n")
        text = self._displayed_text()
        self.assertEqual(text.count("Hello"), 1)

    def test_idle_flush_does_not_trigger_while_data_still_streaming(self):
        self.tab._on_raw_data_received(b"partial")
        self.tab._flush_idle_partial_buffer()
        self.assertEqual(self.tab._rx_partial_buffer, "partial")
        self.assertNotIn("partial", self._displayed_text())


class TestAppSettingsPersistence(unittest.TestCase):
    def setUp(self):
        self._tmp_dir = tempfile.mkdtemp()
        self._ini_path = os.path.join(self._tmp_dir, "test_settings.ini")

    def tearDown(self):
        shutil.rmtree(self._tmp_dir, ignore_errors=True)

    def test_round_trip_persist_and_restore(self):
        settings = AppSettings(settings_path=self._ini_path)
        settings.set_last_port("COM5")
        settings.set_last_baudrate(57600)
        settings.set_last_timeout(7.5)
        settings.set_last_tap_dir(r"C:\gcode")
        settings.set_ack_value("done")
        settings.set_terminal_hex_mode(True)
        settings.set_terminal_auto_scroll(False)
        settings.set_terminal_append_lf(False)
        settings.set_terminal_line_ending_index(2)
        settings.sync()

        reloaded = AppSettings(settings_path=self._ini_path)
        self.assertEqual(reloaded.last_port(), "COM5")
        self.assertEqual(reloaded.last_baudrate(115200), 57600)
        self.assertEqual(reloaded.last_timeout(5.0), 7.5)
        self.assertEqual(reloaded.last_tap_dir(), r"C:\gcode")
        self.assertEqual(reloaded.ack_value(), "done")
        self.assertTrue(reloaded.terminal_hex_mode())
        self.assertFalse(reloaded.terminal_auto_scroll())
        self.assertFalse(reloaded.terminal_append_lf())
        self.assertEqual(reloaded.terminal_line_ending_index(), 2)

    def test_missing_or_invalid_values_fall_back_to_defaults(self):
        settings = AppSettings(settings_path=self._ini_path)
        self.assertEqual(settings.last_port(), "")
        self.assertEqual(settings.last_baudrate(115200), 115200)
        self.assertEqual(settings.last_timeout(5.0), 5.0)
        self.assertEqual(settings.ack_value(), "ok")
        self.assertTrue(settings.terminal_auto_scroll(True))
        self.assertIsNone(settings.window_geometry())

        with open(self._ini_path, "w", encoding="utf-8") as f:
            f.write("[serial]\nbaudrate=not_a_number\n")
        broken = AppSettings(settings_path=self._ini_path)
        self.assertEqual(broken.last_baudrate(9600), 9600)


class TestLogExportWorker(unittest.TestCase):
    def setUp(self):
        self._tmp_dir = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self._tmp_dir, ignore_errors=True)

    def _run_and_collect(self, worker: LogExportWorker):
        results = []
        worker.export_finished.connect(lambda ok, msg: results.append((ok, msg)))
        worker.run()
        return results

    def test_export_csv(self):
        csv_path = os.path.join(self._tmp_dir, "out.csv")
        worker = LogExportWorker(
            file_path=csv_path,
            fmt="csv",
            columns=["Timestamp", "Direction", "Data"],
            rows=[["10:00:00.000", "TX", "G0 X1"], ["10:00:00.100", "RX", "ok"]],
        )
        results = self._run_and_collect(worker)
        self.assertTrue(results[0][0])
        with open(csv_path, "r", encoding="utf-8") as f:
            content = f.read()
        self.assertIn("Timestamp,Direction,Data", content)
        self.assertIn("TX,G0 X1", content)
        self.assertIn("RX,ok", content)

    def test_export_txt(self):
        txt_path = os.path.join(self._tmp_dir, "out.txt")
        worker = LogExportWorker(
            file_path=txt_path,
            fmt="txt",
            columns=["Line", "Command", "Status"],
            rows=[["4", "G21", "OK"]],
        )
        results = self._run_and_collect(worker)
        self.assertTrue(results[0][0])
        with open(txt_path, "r", encoding="utf-8") as f:
            content = f.read()
        self.assertIn("Line | Command | Status", content)
        self.assertIn("4 | G21 | OK", content)

    def test_export_empty_log_reports_failure_without_writing_file(self):
        csv_path = os.path.join(self._tmp_dir, "empty.csv")
        worker = LogExportWorker(file_path=csv_path, fmt="csv", columns=["A"], rows=[])
        results = self._run_and_collect(worker)
        self.assertFalse(results[0][0])
        self.assertFalse(os.path.exists(csv_path))

    def test_export_write_error_is_reported_not_raised(self):
        bad_path = os.path.join(self._tmp_dir, "missing_dir", "out.csv")
        worker = LogExportWorker(file_path=bad_path, fmt="csv", columns=["A"], rows=[["1"]])
        results = self._run_and_collect(worker)
        self.assertFalse(results[0][0])
        self.assertIn("Lỗi", results[0][1])


class TestSimulatedSerialTransport(unittest.TestCase):
    """Kiểm thử SimulatedSerialPort theo nguyên tắc Manual RX: TX không bao giờ tự sinh RX,
    toàn bộ dữ liệu nhận phải được đưa vào thủ công qua inject_rx()."""

    def _wait_until(self, predicate, timeout=2.0, interval=0.02) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            _qapp.processEvents()
            if predicate():
                return True
            time.sleep(interval)
        _qapp.processEvents()
        return predicate()

    def test_simulated_connect_reports_distinct_simulated_state(self):
        mgr = SerialManager()
        ok, _ = mgr.connect_simulated(timeout=1.0)
        try:
            self.assertTrue(ok)
            self.assertTrue(mgr.is_simulated)
            self.assertIsNotNone(mgr.simulated_port)
            self.assertEqual(mgr.current_port, "SIM")
        finally:
            mgr.disconnect_port()
        self.assertFalse(mgr.is_simulated)

    def test_write_never_auto_generates_rx(self):
        # (1) write() chỉ ghi nhận TX, tuyệt đối không tự sinh bất kỳ dữ liệu RX nào.
        mgr = SerialManager()
        mgr.connect_simulated(timeout=1.0)
        sim = mgr.simulated_port
        try:
            self.assertEqual(sim.in_waiting, 0)
            mgr.write_bytes(b"G0 X1\n")
            # Chờ một khoảng đủ lâu để chắc chắn không có luồng nền nào tự thêm ACK/lỗi
            time.sleep(0.3)
            self.assertEqual(sim.in_waiting, 0)
            self.assertEqual(sim.tx_log, [b"G0 X1\n"])
        finally:
            mgr.disconnect_port()

    def test_inject_rx_bytes_appear_in_buffer_and_are_readable(self):
        # (2) + (3) inject_rx(b"ok\n") đưa dữ liệu vào buffer; read() đọc lại đúng nội dung.
        sim = SimulatedSerialPort()
        sim.inject_rx(b"ok\n")
        self.assertEqual(sim.in_waiting, 3)
        self.assertEqual(sim.read(10), b"ok\n")
        self.assertEqual(sim.in_waiting, 0)

    def test_in_waiting_reports_exact_pending_byte_count(self):
        # (4) in_waiting phải phản ánh đúng số byte đang chờ, kể cả khi đọc một phần.
        sim = SimulatedSerialPort()
        sim.inject_rx(b"ALARM:1\r\n")
        self.assertEqual(sim.in_waiting, 9)
        self.assertEqual(sim.read(5), b"ALARM")
        self.assertEqual(sim.in_waiting, 4)

    def test_multiple_consecutive_injections_are_queued_in_order(self):
        # (5) Có thể inject nhiều phản hồi liên tiếp, dữ liệu được giữ đúng thứ tự.
        sim = SimulatedSerialPort()
        sim.inject_rx(b"ok\r\n")
        sim.inject_rx(b"ok\r\n")
        sim.inject_rx(b"error:20\r\n")
        self.assertEqual(sim.in_waiting, len(b"ok\r\nok\r\nerror:20\r\n"))
        self.assertEqual(sim.read(sim.in_waiting), b"ok\r\nok\r\nerror:20\r\n")

    def test_inject_error_and_alarm_text_preserved_verbatim(self):
        # (6) Nội dung error:20 / ALARM:1 phải được truyền nguyên vẹn, không bị biến đổi.
        sim = SimulatedSerialPort()
        sim.inject_rx("error:20\r\n")
        sim.inject_rx("ALARM:1\r\n")
        data = sim.read(sim.in_waiting)
        self.assertEqual(data, b"error:20\r\nALARM:1\r\n")

    def test_no_manual_rx_causes_tap_sender_timeout(self):
        # (7) Nếu không inject RX, TAP Sender phải Timeout đúng theo command_timeout đã cấu hình.
        mgr = SerialManager()
        mgr.connect_simulated(timeout=1.0)
        try:
            cmd = ParsedTapCommand(line_number=1, raw_line="G0 X1", cleaned_command="G0 X1")
            worker = TapSenderWorker(
                serial_mgr=mgr,
                commands=[cmd],
                total_file_lines=1,
                command_timeout=0.3,
                boot_delay=0.0,
                stop_on_timeout=True,
                stop_on_error=True,
                ack_value="ok",
            )
            finished = []
            worker.transmission_finished.connect(lambda ok, msg: finished.append((ok, msg)))
            start = time.monotonic()
            worker.run()  # Chạy đồng bộ trong luồng test, không cần start() luồng riêng
            elapsed = time.monotonic() - start

            self.assertEqual(len(finished), 1)
            self.assertFalse(finished[0][0])
            self.assertIn("Quá thời gian chờ", finished[0][1])
            self.assertGreaterEqual(elapsed, 0.3)
            self.assertLess(elapsed, 1.2)
        finally:
            mgr.disconnect_port()

    def test_manual_rx_unblocks_tap_sender_before_timeout(self):
        # Bổ sung: gửi RX thủ công đúng lúc phải giải phóng TAP Sender ngay lập tức
        # (mô phỏng người dùng bấm 'Gửi RX' sau khi TX được gửi).
        mgr = SerialManager()
        mgr.connect_simulated(timeout=2.0)
        try:
            sim = mgr.simulated_port
            cmd = ParsedTapCommand(line_number=1, raw_line="G0 X1", cleaned_command="G0 X1")
            worker = TapSenderWorker(
                serial_mgr=mgr,
                commands=[cmd],
                total_file_lines=1,
                command_timeout=2.0,
                boot_delay=0.0,
                stop_on_timeout=True,
                ack_value="ok",
            )
            mgr.line_received.connect(worker.on_line_received)

            finished = []
            worker.transmission_finished.connect(lambda ok, msg: finished.append((ok, msg)))

            # Chạy worker trong QThread thật (giống hành vi thực tế của GUI) để tín hiệu
            # line_received phát từ luồng đọc nền có thể được bơm và xử lý ở luồng chính.
            worker.start()
            self.assertTrue(self._wait_until(lambda: len(sim.tx_log) > 0, timeout=2.0))

            # Mô phỏng người dùng bấm 'Gửi RX' ngay sau khi thấy lệnh đã được gửi đi
            sim.inject_rx(b"ok\r\n")

            self.assertTrue(self._wait_until(lambda: len(finished) > 0, timeout=2.0))
            worker.wait(2000)

            self.assertTrue(finished[0][0])
        finally:
            mgr.disconnect_port()

    def test_disconnect_reconnect_does_not_retain_old_rx_data(self):
        # (8) Disconnect/reconnect không được giữ lại dữ liệu RX cũ trong buffer.
        mgr = SerialManager()
        mgr.connect_simulated(timeout=1.0)
        sim1 = mgr.simulated_port
        sim1.inject_rx(b"leftover_data\r\n")
        self.assertGreater(sim1.in_waiting, 0)
        mgr.disconnect_port()

        mgr.connect_simulated(timeout=1.0)
        sim2 = mgr.simulated_port
        try:
            self.assertIsNot(sim1, sim2)
            self.assertEqual(sim2.in_waiting, 0)
        finally:
            mgr.disconnect_port()

    def test_real_serial_path_unaffected_by_simulation_changes(self):
        # (9) UART thật (dùng factory riêng) không bị ảnh hưởng bởi thay đổi của Simulation Mode.
        mgr = SerialManager(serial_factory=FakeSerialDevice)
        ok, _ = mgr.connect_port(port="COM_TEST", baudrate=115200, timeout=1.0)
        try:
            self.assertTrue(ok)
            self.assertFalse(mgr.is_simulated)
            self.assertIsNone(mgr.simulated_port)

            received = []
            mgr.line_received.connect(received.append)
            mgr.write_bytes(b"G0 X1\n")
            # FakeSerialDevice (mô phỏng phần cứng thật trong unit test) tự thêm 'ok' sau mỗi lần ghi
            self.assertTrue(
                self._wait_until(lambda: any(r.strip().lower() == "ok" for r in received))
            )
        finally:
            mgr.disconnect_port()


if __name__ == "__main__":
    unittest.main()