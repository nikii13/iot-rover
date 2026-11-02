"""
encoder_client.py — Cliente del servicio C++ "encodered" (socket UNIX datagrama).

Protocolo (sin cambios respecto al original):
    - El cliente crea su propio socket AF_UNIX/SOCK_DGRAM y hace bind() a una
      ruta temporal propia, para que el servicio pueda responder.
    - Envía b"GET" a /tmp/encoder.sock  ->  recibe b"MotorA_steps,MotorB_steps"
    - Diagnóstico por canal: b"STATUS" (encodered de este proyecto) o
      b"diag" (encoderd del laboratorio, iot-rover). status() prueba ambos.
    - NUNCA se envía b"reset": el encoderd del laboratorio pondría a cero sus
      contadores y afectaría a cualquier otro programa que los esté usando.

Este módulo NO lee GPIO: el único responsable de los encoders es encodered.
"""

import os
import socket
import tempfile

import config as _config


class EncoderError(RuntimeError):
    """Fallo de comunicación con encodered (servicio caído, timeout, respuesta inválida)."""


class EncoderClient:
    def __init__(self, socket_path=None, timeout_s=None, retries=None, cfg=None):
        cfg = cfg or _config
        self.server_path = socket_path or cfg.ENCODER_SOCKET
        self.timeout_s = cfg.ENCODER_TIMEOUT_S if timeout_s is None else timeout_s
        self.retries = cfg.ENCODER_RETRIES if retries is None else retries
        self.client_path = os.path.join(
            tempfile.gettempdir(), f"encoder_client_{os.getpid()}_{id(self) & 0xFFFF:x}.sock"
        )
        self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        try:
            os.unlink(self.client_path)
        except FileNotFoundError:
            pass
        self._sock.bind(self.client_path)
        self._sock.settimeout(self.timeout_s)

    # -- bajo nivel ------------------------------------------------------------
    def _drain(self):
        """Descarta respuestas atrasadas de peticiones que expiraron."""
        self._sock.setblocking(False)
        try:
            while True:
                self._sock.recv(256)
        except (BlockingIOError, OSError):
            pass
        finally:
            self._sock.settimeout(self.timeout_s)

    def request(self, message=b"GET"):
        last_err = None
        for _ in range(self.retries + 1):
            self._drain()
            try:
                self._sock.sendto(message, self.server_path)
                return self._sock.recv(256).decode("ascii", "replace").strip()
            except socket.timeout:
                last_err = EncoderError(f"timeout ({self.timeout_s*1000:.0f} ms) esperando a encodered")
            except (FileNotFoundError, ConnectionRefusedError):
                last_err = EncoderError(
                    f"encodered no está en ejecución ({self.server_path} no existe). "
                    "Arrancadlo con scripts/start_services.sh"
                )
            except OSError as e:
                last_err = EncoderError(f"error de socket: {e}")
        raise last_err

    # -- API ---------------------------------------------------------------------
    def get_counts(self):
        """Devuelve (cuentas_A, cuentas_B) como enteros."""
        return parse_counts(self.request(b"GET"))

    # Alias habituales
    read = get_counts
    get_steps = get_counts

    def status(self):
        """Cuentas y flancos por canal, sea cual sea el servicio que responde."""
        text = self.request(b"STATUS")
        if ";mode=" in text:
            return parse_status(text)          # encodered (cpp/ de este proyecto)
        text = self.request(b"diag")
        if text.startswith("A1="):
            return parse_diag(text)            # encoderd del laboratorio
        a, b = parse_counts(text)              # servicio que solo da "A,B"
        return {"a": a, "b": b, "service": "desconocido"}

    def ping(self):
        """True si algún servicio de encoders responde con un formato válido."""
        try:
            text = self.request(b"PING")
            if text == "PONG":
                return True
            parse_counts(text)  # encoderd del laboratorio responde "A,B" a todo
            return True
        except EncoderError:
            return False

    def close(self):
        try:
            self._sock.close()
        finally:
            try:
                os.unlink(self.client_path)
            except FileNotFoundError:
                pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


def parse_counts(text):
    head = text.split(";", 1)[0]
    parts = head.split(",")
    if len(parts) != 2:
        raise EncoderError(f"respuesta inválida de encodered: {text!r}")
    try:
        return int(parts[0]), int(parts[1])
    except ValueError:
        raise EncoderError(f"respuesta inválida de encodered: {text!r}") from None


def parse_diag(text):
    """Respuesta 'diag' del encoderd del laboratorio:
    "A1=n,A2=n,B1=n,B2=n;A=n,B=n"  (flancos de SUBIDA por canal; A/B = max de sus canales)."""
    try:
        chans, totals = text.split(";", 1)
        ch = dict(kv.split("=") for kv in chans.split(","))
        tot = dict(kv.split("=") for kv in totals.split(","))
        out = {"a": int(tot["A"]), "b": int(tot["B"]), "mode": "rising-max (encoderd lab)",
               "service": "encoderd"}
        for k in ("A1", "A2", "B1", "B2"):
            out[k.lower()] = {"rising": int(ch[k]), "falling": 0}
        return out
    except (ValueError, KeyError):
        raise EncoderError(f"respuesta diag inválida: {text!r}") from None


def parse_status(text):
    """Convierte la respuesta STATUS (encodered de este proyecto) en dict."""
    a, b = parse_counts(text)
    out = {"a": a, "b": b, "service": "encodered"}
    for field in text.split(";")[1:]:
        if "=" not in field:
            continue
        k, v = field.split("=", 1)
        if "/" in v:
            r, f = v.split("/", 1)
            out[k] = {"rising": int(r), "falling": int(f)}
        elif "," in v:
            out[k] = tuple(int(x) for x in v.split(","))
        else:
            try:
                out[k] = int(v)
            except ValueError:
                out[k] = v
    return out
