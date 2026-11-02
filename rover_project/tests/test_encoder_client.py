"""Cliente del socket UNIX y, si está compilado, el servicio C++ real en modo --no-gpio."""
import os
import socket
import subprocess
import threading
import time

import pytest

from encoder_client import EncoderClient, EncoderError, parse_status

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
BINARY = os.path.join(ROOT, "cpp", "encodered")


def fake_server(path, replies, stop):
    s = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    s.bind(path)
    s.settimeout(0.05)
    while not stop.is_set():
        try:
            msg, addr = s.recvfrom(64)
        except socket.timeout:
            continue
        r = replies(msg)
        if r is not None:
            s.sendto(r, addr)
    s.close()
    os.unlink(path)


@pytest.fixture
def server(tmp_path):
    path = str(tmp_path / "enc.sock")
    stop = threading.Event()
    replies = {"fn": lambda m: b"123,-45"}
    th = threading.Thread(target=fake_server, args=(path, lambda m: replies["fn"](m), stop), daemon=True)
    th.start()
    while not os.path.exists(path):
        time.sleep(0.01)
    yield path, replies
    stop.set()
    th.join()


def test_get_counts(server):
    path, _ = server
    with EncoderClient(path, timeout_s=0.2) as c:
        assert c.get_counts() == (123, -45)
        client_path = c.client_path
        assert os.path.exists(client_path)
    assert not os.path.exists(client_path), "el socket del cliente debe borrarse al cerrar"


def test_timeout(server):
    path, replies = server
    replies["fn"] = lambda m: None
    with EncoderClient(path, timeout_s=0.05, retries=1) as c:
        with pytest.raises(EncoderError, match="timeout"):
            c.get_counts()


def test_invalid_reply(server):
    path, replies = server
    replies["fn"] = lambda m: b"basura"
    with EncoderClient(path, timeout_s=0.1) as c:
        with pytest.raises(EncoderError, match="inválida"):
            c.get_counts()


def test_service_not_running(tmp_path):
    with EncoderClient(str(tmp_path / "noexiste.sock"), timeout_s=0.05) as c:
        with pytest.raises(EncoderError, match="no está en ejecución"):
            c.get_counts()


def test_parse_status():
    st = parse_status("10,20;mode=x1;a1=5/5;a2=0/0;b1=7/6;b2=1/1;inv=0,0;rep=0,0;lost=3")
    assert st["a"] == 10 and st["b"] == 20 and st["mode"] == "x1"
    assert st["a2"] == {"rising": 0, "falling": 0} and st["lost"] == 3


@pytest.mark.skipif(not os.path.exists(BINARY), reason="cpp/encodered no compilado (make -C cpp)")
def test_real_service_protocol_and_single_instance(tmp_path):
    sock, lock = str(tmp_path / "e.sock"), str(tmp_path / "e.lock")
    p = subprocess.Popen([BINARY, "--no-gpio", "--socket", sock, "--lock", lock], stderr=subprocess.PIPE)
    try:
        for _ in range(100):
            if os.path.exists(sock):
                break
            time.sleep(0.02)
        with EncoderClient(sock, timeout_s=0.5) as c:
            assert c.get_counts() == (0, 0)
            assert c.ping()
            assert c.status()["mode"] == "x1"
        second = subprocess.run([BINARY, "--no-gpio", "--socket", sock, "--lock", lock], capture_output=True, timeout=5)
        assert second.returncode == 2, "una segunda instancia debe negarse a arrancar"
        assert os.path.exists(sock), "la segunda instancia no debe borrar el socket de la primera"
    finally:
        p.terminate()
        p.wait(timeout=5)
    assert not os.path.exists(sock), "el servicio debe borrar su socket al terminar"


def lab_encoderd_reply(msg):
    """Imita el encoderd del laboratorio (iot-rover/encoderd.cpp)."""
    if msg.startswith(b"diag"):
        return b"A1=700,A2=0,B1=698,B2=702;A=700,B=702"
    if msg.startswith(b"reset"):
        raise AssertionError("el cliente nunca debe enviar reset")
    return b"700,702"


def test_works_with_lab_encoderd(server):
    path, replies = server
    replies["fn"] = lab_encoderd_reply
    with EncoderClient(path, timeout_s=0.2) as c:
        assert c.get_counts() == (700, 702)
        assert c.ping()
        st = c.status()
    assert st["service"] == "encoderd"
    assert st["a2"]["rising"] == 0 and st["b2"]["rising"] == 702


def test_encoder_status_diagnoses_dead_channel_with_lab_service():
    from encoder_client import parse_diag
    from encoder_status import diagnose
    msgs = diagnose(parse_diag("A1=700,A2=0,B1=698,B2=702;A=700,B=702"))
    assert any("Motor A" in m and "canal 2" in m for m in msgs)
    assert not any("Motor B" in m for m in msgs)
