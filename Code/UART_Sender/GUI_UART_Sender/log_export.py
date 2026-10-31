"""Tiện ích xuất nhật ký (TXT/CSV) chạy nền, dùng chung cho cả hai Tab."""

import csv
from typing import List, Sequence

from PySide6.QtCore import QThread, Signal


class LogExportWorker(QThread):
    """Ghi file log ở luồng nền để không bao giờ làm đứng giao diện khi xuất log lớn."""

    export_finished = Signal(bool, str)

    def __init__(
        self,
        file_path: str,
        fmt: str,
        columns: Sequence[str],
        rows: List[Sequence[str]],
        parent=None,
    ):
        super().__init__(parent)
        self.file_path = file_path
        self.fmt = fmt.lower()
        self.columns = list(columns)
        self.rows = [list(r) for r in rows]

    def run(self) -> None:
        if not self.rows:
            self.export_finished.emit(False, "Không có dữ liệu để xuất.")
            return

        try:
            if self.fmt == "csv":
                with open(self.file_path, "w", newline="", encoding="utf-8") as f:
                    writer = csv.writer(f)
                    writer.writerow(self.columns)
                    writer.writerows(self.rows)
            else:
                with open(self.file_path, "w", encoding="utf-8") as f:
                    f.write(" | ".join(self.columns) + "\n")
                    for row in self.rows:
                        f.write(" | ".join(str(c) for c in row) + "\n")
        except OSError as exc:
            self.export_finished.emit(False, f"Lỗi khi ghi file: {exc}")
            return

        self.export_finished.emit(
            True, f"Đã xuất {len(self.rows)} dòng ra:\n{self.file_path}"
        )
