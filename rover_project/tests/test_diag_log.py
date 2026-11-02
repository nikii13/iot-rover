"""El log no bloquea: con la cola llena descarta muestras en lugar de esperar."""
import csv
import time

import config
from diag_log import DiagLogger


def test_writes_csv_and_never_blocks(tmp_path):
    cfg = config.as_namespace(LOG_DIR=str(tmp_path), LOG_QUEUE_SIZE=10, LOG_CONSOLE_HZ=0)
    lg = DiagLogger("t", cfg, console=False)
    t0 = time.monotonic()
    for i in range(5000):
        lg.log({"t": float(i), "state": "FOLLOWING", "coverage": 0.5})
    lg.event("prueba")
    assert time.monotonic() - t0 < 0.5
    lg.close()
    rows = list(csv.DictReader(open(lg.path)))
    assert rows and rows[0]["state"] == "FOLLOWING"
