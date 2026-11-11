import os
import sys
from PySide6.QtWidgets import QApplication
from main_window import MainWindow


def main() -> None:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    window = MainWindow()

    # Hỗ trợ nạp nhanh file .tap qua tham số dòng lệnh giống test_UART.py
    if len(sys.argv) > 1 and os.path.exists(sys.argv[1]):
        window.tap_tab.load_tap_file(sys.argv[1])

    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()