# UART / TAP File Sender & Hercules Terminal GUI

Ứng dụng Desktop GUI hoàn chỉnh được xây dựng dựa trên logic truyền file của `test_UART.py`, kết hợp giữa trình truyền file G-code/`.tap` và công cụ **UART Terminal** kiểu Hercules.

## 1. Tính năng chính
* **Tự động dò và ưu tiên cổng UART:** Quét toàn bộ cổng COM, tự động ưu tiên các chip chuyển đổi USB-UART phổ biến (`CH340`, `CP210x`, `FTDI`, `Arduino`, `USB Serial`) và hỗ trợ làm mới hoặc tự phát hiện khi cắm/rút thiết bị.
* **Quản lý kết nối dùng chung (`SerialManager`):** Hai tab dùng chung một kết nối Serial duy nhất, ngăn chặn hoàn toàn lỗi xung đột đọc/ghi hoặc mở trùng cổng COM.
* **Chế độ Mô phỏng (Simulation Mode — Manual RX):**
  * Cho phép kết nối tới cổng ảo `SIM` mà không cần cắm mạch phần cứng thật.
  * Hoạt động theo nguyên tắc **Manual RX**: chiều gửi (TX) chỉ ghi nhận dữ liệu và không tự sinh phản hồi tự động, giúp kiểm thử chính xác kịch bản chờ ACK và Timeout như trên thiết bị thật.
  * Khung điều khiển mô phỏng cho phép nhập chuỗi RX tùy ý (kèm tùy chọn tự nối `CRLF`) hoặc bấm nút gửi nhanh `ok`, `error:20`, `ALARM:1`.
* **Lưu trữ cấu hình tự động (`AppSettings`):** Tự động ghi nhớ cổng COM, Baudrate, Timeout, thư mục mở file `.tap`, chuỗi xác nhận ACK, chế độ hiển thị Terminal và kích thước cửa sổ vào file `uart_gui_settings.ini`.
* **Tab 1 — TAP File Sender:**
  * Đọc file `.tap` / `.gcode` (`UTF-8`), lọc bỏ dòng trống, chú thích sau `;` và chú thích trong ngoặc tròn `(...)`.
  * Xem trước (Preview) danh sách lệnh sạch trước khi truyền.
  * Cho phép tùy chỉnh chuỗi xác nhận **ACK** mong đợi (mặc định là `ok`, có thể đổi thành `done`, `ready`, v.v.).
  * Có cơ chế **Timeout thực tế trên từng lệnh** (có trần bảo vệ chống treo khi thiết bị gửi log liên tục).
  * Hỗ trợ `Start`, `Pause / Resume`, `Stop`, giám sát tín hiệu UART ngoài độc lập và **Xuất Log (Export)** ra `.csv` hoặc `.txt`.
* **Tab 2 — UART Terminal (Hercules-like):**
  * Gửi lệnh trực tiếp bằng phím `Enter` hoặc nút `Send`, lưu lịch sử lệnh bằng phím mũi tên `↑` / `↓`.
  * Hỗ trợ hộp kiểm **Append `<LF>` automatically** (`0x0A`) và bộ chọn kiểu kết thúc dòng (`None`, `LF`, `CR`, `CRLF`), tự động chống nhân đôi ký tự xuống dòng.
  * Hỗ trợ gửi và xem dữ liệu ở cả chế độ **ASCII/UTF-8** và **HEX**, tự động xả gói dữ liệu lẻ (idle flush ~70ms) và xuất nhật ký Terminal ra `.csv` / `.txt`.

## 2. Cài đặt và Khởi chạy

### Yêu cầu hệ thống
* Python 3.9 trở lên.

### Cài đặt thư viện
```bash
pip install -r requirements.txt