import csv
import os
from datetime import datetime


class TelemetryLogger:
    def __init__(self):
        log_dir = os.getenv('TELEMETRY_LOG_DIR', 'data')
        os.makedirs(log_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.filepath = os.path.join(log_dir, f'telemetry_{timestamp}.csv')

        self.file = open(self.filepath, 'w', newline='')
        self.writer = csv.writer(self.writer_file if False else self.file)

        self.writer.writerow([
            'timestamp',
            'speed_m_s',
            'battery_percent'
        ])

    def log(self, speed, battery):
        self.writer.writerow([
            datetime.now().isoformat(),
            f'{speed:.3f}',
            f'{battery:.2f}'
        ])
        self.file.flush()

    def close(self):
        self.file.close()
