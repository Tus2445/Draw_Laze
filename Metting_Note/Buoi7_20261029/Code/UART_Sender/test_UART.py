import os
import sys
import time
import serial
import serial.tools.list_ports

BAUDRATE = 115200
TIMEOUT = 5  # Timeout (giây) khi chờ phản hồi 'ok'


def find_serial_port():
    """Tự động quét và tìm cổng UART/Serial khả dụng."""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None

    # Hiển thị danh sách cổng tìm thấy
    print("Danh sách cổng phát hiện:")
    for p in ports:
        print(f" - {p.device}: {p.description}")

    # Ưu tiên tìm các cổng USB Serial phổ biến (CH340, CP210x, FTDI, Arduino, USB Serial)
    keywords = ["usb", "ch340", "cp210", "ftdi", "arduino", "serial"]
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if any(kw in desc or kw in hwid for kw in keywords):
            print(f"-> Tự động chọn cổng phù hợp: {p.device}")
            return p.device

    # Nếu không nhận diện được tên cụ thể, chọn cổng đầu tiên
    print(f"-> Mặc định chọn cổng đầu tiên: {ports[0].device}")
    return ports[0].device


def send_gcode_file(file_path: str, baudrate: int = BAUDRATE):
    """Đọc file G-code/tap và gửi qua UART."""
    if not os.path.exists(file_path):
        print(f"Lỗi: Không tìm thấy file '{file_path}'")
        return

    port = find_serial_port()
    if not port:
        print("Lỗi: Không tìm thấy cổng UART/Serial nào đang kết nối.")
        return

    try:
        print(f"\nĐang mở cổng {port} với baudrate {baudrate}...")
        ser = serial.Serial(port, baudrate, timeout=TIMEOUT)

        # Chờ vi điều khiển khởi động lại (GRBL/Arduino thường reset khi mở cổng)
        time.sleep(2)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Đọc phản hồi ban đầu (nếu có chuỗi chào mừng như 'Grbl 1.1h [...]')
        while ser.in_waiting:
            ser.readline()

        print(f"Bắt đầu truyền file: {file_path}")

        with open(file_path, "r", encoding="utf-8", errors="ignore") as f:
            lines = f.readlines()

        total_lines = len(lines)
        for idx, line in enumerate(lines, 1):
            # Xóa khoảng trắng thừa và bỏ phần chú thích (nằm trong ngoặc tròn hoặc sau dấu chấm phẩy)
            clean_line = line.strip()
            if ";" in clean_line:
                clean_line = clean_line.split(";")[0].strip()
            if "(" in clean_line and ")" in clean_line:
                # Bỏ comment dạng (comment)
                import re

                clean_line = re.sub(r"\(.*?\)", "", clean_line).strip()

            if not clean_line:
                continue

            # Gửi lệnh kèm ký tự xuống dòng
            ser.write((clean_line + "\n").encode("ascii"))
            print(f"[{idx}/{total_lines}] Gửi: {clean_line}")

            # Đợi phản hồi 'ok' hoặc thông báo lỗi từ board điều khiển
            while True:
                response = ser.readline().decode("latin-1", errors="ignore").strip()
                if not response:
                    continue
                if "ok" in response.lower():
                    break
                elif "error" in response.lower() or "alarm" in response.lower():
                    print(f"Cảnh báo/Lỗi từ thiết bị: {response}")
                    break

        print("\nHoàn thành truyền toàn bộ file G-code.")

    except serial.SerialException as e:
        print(f"Lỗi kết nối Serial: {e}")
    except KeyboardInterrupt:
        print("\nĐã dừng gửi lệnh theo yêu cầu người dùng (Ctrl+C).")
    finally:
        if "ser" in locals() and ser.is_open:
            ser.close()
            print("Đã đóng kết nối UART.")


if __name__ == "__main__":
    # Đặt tên file .tap của bạn ở đây (hoặc truyền qua tham số dòng lệnh)
    input_file = sys.argv[1] if len(sys.argv) > 1 else "program.tap"
    send_gcode_file(input_file)