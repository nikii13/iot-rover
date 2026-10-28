import os
import re
import socket
import tempfile

SOCKET_PATH = "/tmp/encoder.sock"


def _request(message="counts"):
    if not os.path.exists(SOCKET_PATH):
        raise RuntimeError(
            f"{SOCKET_PATH} does not exist. Check: systemctl status encoderd"
        )

    client_path = tempfile.mktemp(prefix="encoder-client-", dir="/tmp")
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        sock.bind(client_path)
        sock.settimeout(1.0)
        sock.connect(SOCKET_PATH)
        sock.send((message + "\n").encode())
        return sock.recv(256).decode(errors="replace").strip()
    finally:
        sock.close()
        try:
            os.unlink(client_path)
        except FileNotFoundError:
            pass


def counts():
    reply = _request("counts")
    match = re.fullmatch(r"\s*(-?\d+)\s*,\s*(-?\d+)\s*", reply)
    if not match:
        raise RuntimeError(f"Unexpected encoder reply: {reply!r}")
    return int(match.group(1)), int(match.group(2))


def diagnostics():
    return _request("diag")


def reset():
    _request("reset")
