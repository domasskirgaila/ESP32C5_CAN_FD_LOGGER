"""
USB/serial control panel for the ESP-IDF twai_utils console firmware
(https://github.com/espressif/esp-idf/tree/master/examples/peripherals/twai/twai_utils).

The firmware exposes an esp_console command line over the USB serial port:
    twai_init <twai0|twai1> -t <tx_gpio> -r <rx_gpio> -b <bitrate> [-B <fd_bitrate>] [--loopback|--self-test|--listen]
    twai_deinit <twai0|twai1>
    twai_info <twai0|twai1>
    twai_send <twai0|twai1> <id>#<data> | <id>#R<dlc> | <id>##<flags><data>
    twai_dump <twai0|twai1> [--stop]

-b sets the nominal/arbitration bitrate (the ID phase, used by every frame).
-B sets the TWAI-FD data-phase bitrate (only used inside FD frames' BRS payload
section); FD-capable chips only, e.g. ESP32-C5 with CONFIG_EXAMPLE_ENABLE_TWAI_FD.

Standard vs extended CAN ID is decided by the firmware itself from the typed ID
text: extended if more than 3 hex digits were typed OR the value exceeds 0x7FF
(twai_utils_parser.c: parse_hex_id -> is_ext = len > 3 || value > 0x7FF).

twai_dump prints received frames as (twai_utils_parser.c: format_twaidump_frame):
    twai<n>  <id_hex>  [<len>] <byte> <byte> ...          (data frame)
    twai<n>  <id_hex>  [R<dlc>]                           (RTR frame)

Requires: pyserial (pip install pyserial)
"""

import queue
import re
import threading
import time
import tkinter as tk
from tkinter import ttk

import serial
import serial.tools.list_ports

CAN_BITRATES = [
    1000000,
    800000,
    500000,
    250000,
    125000,
    100000,
    50000,
    20000,
    10000,
]

CAN_FD_DATA_BITRATES = [
    8000000,
    5000000,
    4000000,
    2000000,
    1000000,
    500000,
]

DEFAULT_SERIAL_BAUD = 115200

MIN_LOOP_INTERVAL_MS = 10

CHANNELS = ["twai0", "twai1"]

DEFAULT_GPIO = {
    "twai0": {"tx": "4", "rx": "5"},
    "twai1": {"tx": "6", "rx": "7"},
}

# CAN-FD DLC code -> payload length in bytes. Codes 0-8 are shared with
# classic CAN (code == length); codes 9-15 only exist for FD frames.
FD_DLC_LENGTHS = {
    0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8,
    9: 12, 10: 16, 11: 20, 12: 24, 13: 32, 14: 48, 15: 64,
}


def channel_label(channel):
    return "CAN" + channel[-1]


def dlc_codes(fd_enabled):
    return list(range(16)) if fd_enabled else list(range(9))


def dlc_length(code, fd_enabled):
    if not fd_enabled and code > 8:
        raise ValueError("DLC > 8 requires an FD frame")
    return FD_DLC_LENGTHS[code]


def dlc_label(code, fd_enabled):
    return f"{code} ({dlc_length(code, fd_enabled)}B)"


def dlc_code_from_label(label):
    return int(label.split()[0])


def dlc_code_for_length(byte_count, fd_enabled):
    for code in dlc_codes(fd_enabled):
        if dlc_length(code, fd_enabled) >= byte_count:
            return code
    return max(dlc_codes(fd_enabled))


def clean_hex_id(id_text):
    id_text = id_text.strip()
    if id_text.lower().startswith("0x"):
        id_text = id_text[2:]
    if not id_text:
        raise ValueError("CAN ID is required")
    int(id_text, 16)
    return id_text


def id_is_extended(id_text):
    return len(id_text) > 3 or int(id_text, 16) > 0x7FF


def clean_hex_bytes(data_text):
    data_text = "".join(ch for ch in data_text if ch not in " .\t\n")
    if data_text:
        int(data_text, 16)
        if len(data_text) % 2 != 0:
            raise ValueError("Data must be a whole number of bytes")
    return data_text


def pad_or_trim_hex_bytes(data_hex, target_len):
    byte_count = len(data_hex) // 2
    if byte_count < target_len:
        return data_hex + "00" * (target_len - byte_count)
    return data_hex[: target_len * 2]


DUMP_LINE_RE = re.compile(
    r"twai(?P<ch>\d+)\s+(?P<id>[0-9A-Fa-f]+)\s+\["
    r"(?:R(?P<rtr>\d+)|(?P<len>\d+))\](?P<data>(?:\s[0-9A-Fa-f]{2})*)"
)


def parse_dump_line(line):
    """Parse one twai_dump output line, rejecting anything that looks torn/corrupted.

    Under high message rates the console print stream can outrun the serial
    link's bandwidth and get torn mid-line, producing a structurally valid-
    looking line with a truncated ID or a short data payload. The firmware
    only ever prints a 3-digit (standard) or 8-digit (extended) ID and a data
    byte count that exactly matches the declared [len], so anything else is
    treated as corrupted and discarded rather than shown as a fake frame.
    """
    match = DUMP_LINE_RE.search(line)
    if not match:
        return None
    id_text = match.group("id")
    if len(id_text) not in (3, 8):
        return None
    channel = "twai" + match.group("ch")
    can_id = id_text.upper()
    if match.group("rtr") is not None:
        data = f"RTR (dlc={match.group('rtr')})"
    else:
        declared_len = int(match.group("len"))
        byte_tokens = match.group("data").split()
        if len(byte_tokens) != declared_len:
            return None
        data = " ".join(b.upper() for b in byte_tokens)
    return channel, can_id, data


class SerialLink:
    def __init__(self, on_line):
        self._on_line = on_line
        self._ser = None
        self._stop_evt = threading.Event()
        self._thread = None

    @property
    def is_open(self):
        return self._ser is not None and self._ser.is_open

    def open(self, port, baud):
        self.close()
        self._ser = serial.Serial(port, baud, timeout=0.2)
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def close(self):
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=1)
            self._thread = None
        if self._ser is not None:
            try:
                self._ser.close()
            except serial.SerialException:
                pass
            self._ser = None

    def send(self, command):
        if not self.is_open:
            raise RuntimeError("Serial port is not open")
        self._ser.write((command + "\r\n").encode("utf-8", errors="replace"))

    def _read_loop(self):
        buf = b""
        while not self._stop_evt.is_set():
            try:
                chunk = self._ser.read(256)
            except serial.SerialException as exc:
                self._on_line(f"[error] {exc}")
                return
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self._on_line(line.decode("utf-8", errors="replace").rstrip("\r"))


class TwaiControllerPanel(ttk.LabelFrame):
    def __init__(self, master, channel, send_command, default_tx="", default_rx=""):
        super().__init__(master, text=channel_label(channel), padding=8)
        self.channel = channel
        self._send_command = send_command

        ttk.Label(self, text="TX GPIO:").grid(row=0, column=0, sticky="w")
        self.tx_var = tk.StringVar(value=default_tx)
        ttk.Entry(self, textvariable=self.tx_var, width=6).grid(row=0, column=1, padx=(4, 12))

        ttk.Label(self, text="RX GPIO:").grid(row=0, column=2, sticky="w")
        self.rx_var = tk.StringVar(value=default_rx)
        ttk.Entry(self, textvariable=self.rx_var, width=6).grid(row=0, column=3, padx=(4, 12))

        ttk.Label(self, text="ID bitrate (bps):").grid(row=1, column=0, sticky="w", pady=(6, 0))
        self.bitrate_var = tk.StringVar(value=str(CAN_BITRATES[2]))
        ttk.Combobox(
            self, textvariable=self.bitrate_var, values=[str(b) for b in CAN_BITRATES],
            state="readonly", width=10,
        ).grid(row=1, column=1, sticky="w", padx=(4, 12), pady=(6, 0))

        self.status_var = tk.StringVar(value="Stopped")
        ttk.Label(self, textvariable=self.status_var, width=10).grid(row=1, column=3, pady=(6, 0))

        self.fd_enabled_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            self, text="FD data bitrate:", variable=self.fd_enabled_var,
            command=self._on_fd_toggle,
        ).grid(row=2, column=0, sticky="w", pady=(6, 0))
        self.fd_bitrate_var = tk.StringVar(value=str(CAN_FD_DATA_BITRATES[3]))
        self.fd_bitrate_box = ttk.Combobox(
            self, textvariable=self.fd_bitrate_var, values=[str(b) for b in CAN_FD_DATA_BITRATES],
            state="disabled", width=10,
        )
        self.fd_bitrate_box.grid(row=2, column=1, sticky="w", padx=(4, 12), pady=(6, 0))

        btn_frame = ttk.Frame(self)
        btn_frame.grid(row=3, column=0, columnspan=4, pady=(8, 0), sticky="w")
        ttk.Button(btn_frame, text="Start", command=self._start).pack(side="left", padx=(0, 6))
        ttk.Button(btn_frame, text="Stop", command=self._stop).pack(side="left", padx=(0, 6))
        ttk.Button(btn_frame, text="Info", command=self._info).pack(side="left")

    def _on_fd_toggle(self):
        self.fd_bitrate_box.configure(state="readonly" if self.fd_enabled_var.get() else "disabled")

    def _start(self):
        tx = self.tx_var.get().strip()
        rx = self.rx_var.get().strip()
        if not tx or not rx:
            self.status_var.set("Need TX/RX")
            return
        bitrate = self.bitrate_var.get().strip()
        cmd = f"twai_init {self.channel} -t {tx} -r {rx} -b {bitrate}"
        if self.fd_enabled_var.get():
            cmd += f" -B {self.fd_bitrate_var.get().strip()}"
        self._send_command(cmd)
        self._send_command(f"twai_dump {self.channel}")
        self.status_var.set("Running")

    def _stop(self):
        self._send_command(f"twai_dump {self.channel} --stop")
        self._send_command(f"twai_deinit {self.channel}")
        self.status_var.set("Stopped")

    def _info(self):
        self._send_command(f"twai_info {self.channel}")


class SendLoopPanel(ttk.LabelFrame):
    def __init__(self, master, send_command, log, schedule, cancel_schedule, get_channel_fd_enabled):
        super().__init__(master, text="Send / Loop", padding=8)
        self._send_command = send_command
        self._log = log
        self._schedule = schedule
        self._cancel_schedule = cancel_schedule
        self._get_channel_fd_enabled = get_channel_fd_enabled
        self._loop_job = None
        self._suspend_data_trace = False

        ttk.Label(self, text="Channel:").grid(row=0, column=0, sticky="w")
        self.channel_var = tk.StringVar(value=CHANNELS[0])
        channel_box = ttk.Combobox(
            self, textvariable=self.channel_var,
            values=[f"{channel_label(c)} ({c})" for c in CHANNELS],
            state="readonly", width=14,
        )
        channel_box.current(0)
        channel_box.grid(row=0, column=1, padx=(4, 12), sticky="w")
        channel_box.bind("<<ComboboxSelected>>", self._on_channel_selected)

        ttk.Label(self, text="CAN ID (hex):").grid(row=0, column=2, sticky="w")
        self.id_var = tk.StringVar(value="123")
        ttk.Entry(self, textvariable=self.id_var, width=12).grid(row=0, column=3, sticky="w", padx=(4, 6))
        self.id_type_var = tk.StringVar(value="")
        ttk.Label(self, textvariable=self.id_type_var, width=16).grid(row=0, column=4, sticky="w")
        self.id_var.trace_add("write", self._on_id_changed)

        frame_row = ttk.Frame(self)
        frame_row.grid(row=1, column=0, columnspan=5, sticky="w", pady=(6, 0))
        self.rtr_var = tk.BooleanVar(value=False)
        self.rtr_chk = ttk.Checkbutton(frame_row, text="RTR", variable=self.rtr_var, command=self._on_rtr_toggle)
        self.rtr_chk.pack(side="left")
        self.fd_var = tk.BooleanVar(value=False)
        self.fd_chk = ttk.Checkbutton(frame_row, text="FD frame", variable=self.fd_var, command=self._on_fd_toggle)
        self.fd_chk.pack(side="left", padx=(12, 0))
        self.brs_var = tk.BooleanVar(value=True)
        self.brs_chk = ttk.Checkbutton(frame_row, text="BRS", variable=self.brs_var, state="disabled")
        self.brs_chk.pack(side="left", padx=(12, 0))
        self.esi_var = tk.BooleanVar(value=False)
        self.esi_chk = ttk.Checkbutton(frame_row, text="ESI", variable=self.esi_var, state="disabled")
        self.esi_chk.pack(side="left", padx=(6, 0))

        ttk.Label(self, text="Data (hex bytes):").grid(row=2, column=0, sticky="w", pady=(6, 0))
        self.data_var = tk.StringVar(value="01 02 03 04 05 06 07 08")
        self.data_entry = ttk.Entry(self, textvariable=self.data_var, width=40)
        self.data_entry.grid(row=2, column=1, columnspan=3, padx=(4, 6), pady=(6, 0), sticky="w")
        self.data_var.trace_add("write", self._on_data_changed)

        ttk.Label(self, text="DLC:").grid(row=3, column=0, sticky="w", pady=(6, 0))
        self.dlc_var = tk.StringVar()
        self.dlc_box = ttk.Combobox(self, textvariable=self.dlc_var, state="readonly", width=10)
        self.dlc_box.grid(row=3, column=1, sticky="w", padx=(4, 12), pady=(6, 0))
        self.dlc_box.bind("<<ComboboxSelected>>", self._on_dlc_selected)
        self._refresh_dlc_options(select_code=8)

        ttk.Label(self, text="Interval (ms):").grid(row=3, column=2, sticky="w", pady=(6, 0))
        self.interval_var = tk.StringVar(value="1000")
        interval_entry = ttk.Entry(self, textvariable=self.interval_var, width=8)
        interval_entry.grid(row=3, column=3, padx=(4, 12), pady=(6, 0), sticky="w")
        interval_entry.bind("<FocusOut>", self._on_interval_committed)
        interval_entry.bind("<Return>", self._on_interval_committed)

        btn_frame = ttk.Frame(self)
        btn_frame.grid(row=4, column=0, columnspan=5, pady=(8, 0), sticky="w")
        ttk.Button(btn_frame, text="Send Once", command=self._send_once).pack(side="left", padx=(0, 6))
        self.start_loop_btn = ttk.Button(btn_frame, text="Start Loop", command=self._start_loop)
        self.start_loop_btn.pack(side="left", padx=(0, 6))
        self.stop_loop_btn = ttk.Button(btn_frame, text="Stop Loop", command=self.stop_loop, state="disabled")
        self.stop_loop_btn.pack(side="left")

        self.saved = tk.Listbox(self, height=5)
        self.saved.grid(row=5, column=0, columnspan=5, pady=(10, 0), sticky="ew")
        self.saved.bind("<Double-Button-1>", lambda _evt: self._load_selected())

        list_btns = ttk.Frame(self)
        list_btns.grid(row=6, column=0, columnspan=5, pady=(6, 0), sticky="w")
        ttk.Button(list_btns, text="Save", command=self._save_current).pack(side="left", padx=(0, 6))
        ttk.Button(list_btns, text="Load Selected", command=self._load_selected).pack(side="left", padx=(0, 6))
        ttk.Button(list_btns, text="Delete Selected", command=self._delete_selected).pack(side="left")

        self.refresh_fd_state()

    def _selected_channel(self):
        return CHANNELS[[f"{channel_label(c)} ({c})" for c in CHANNELS].index(self.channel_var.get())] \
            if "(" in self.channel_var.get() else self.channel_var.get()

    def _on_channel_selected(self, _evt=None):
        self.refresh_fd_state()

    def refresh_fd_state(self):
        fd_allowed = self._get_channel_fd_enabled(self._selected_channel())
        if fd_allowed:
            self.fd_chk.configure(state="normal")
        else:
            self.fd_var.set(False)
            self.fd_chk.configure(state="disabled")
        self._on_fd_toggle()

    def _on_id_changed(self, *_args):
        try:
            id_text = clean_hex_id(self.id_var.get())
            self.id_type_var.set("Extended (29-bit)" if id_is_extended(id_text) else "Standard (11-bit)")
        except ValueError:
            self.id_type_var.set("")

    def _on_rtr_toggle(self):
        if self.rtr_var.get():
            self.fd_var.set(False)
            self.fd_chk.configure(state="disabled")
            self.data_entry.configure(state="disabled")
        else:
            self.data_entry.configure(state="normal")
            self.refresh_fd_state()
        self._on_fd_toggle()

    def _on_fd_toggle(self):
        fd = self.fd_var.get()
        self.brs_chk.configure(state="normal" if fd else "disabled")
        self.esi_chk.configure(state="normal" if fd else "disabled")
        self._refresh_dlc_options()

    def _max_data_bytes(self):
        return 64 if self.fd_var.get() else 8

    def _refresh_dlc_options(self, select_code=None):
        fd = self.fd_var.get()
        codes = dlc_codes(fd)
        labels = [dlc_label(c, fd) for c in codes]
        self.dlc_box.configure(values=labels)
        if select_code is not None and select_code in codes:
            self.dlc_var.set(dlc_label(select_code, fd))
        elif self.dlc_var.get() not in labels:
            self.dlc_var.set(labels[-1] if fd else labels[min(8, len(labels) - 1)])

    def _on_data_changed(self, *_args):
        if self._suspend_data_trace or self.rtr_var.get():
            return
        try:
            byte_count = len(clean_hex_bytes(self.data_var.get())) // 2
        except ValueError:
            return
        fd = self.fd_var.get()
        code = dlc_code_for_length(byte_count, fd)
        self.dlc_var.set(dlc_label(code, fd))

    def _on_dlc_selected(self, _evt=None):
        if self.rtr_var.get():
            return
        code = dlc_code_from_label(self.dlc_var.get())
        target_len = dlc_length(code, self.fd_var.get())
        try:
            data_hex = clean_hex_bytes(self.data_var.get())
        except ValueError:
            data_hex = ""
        padded = pad_or_trim_hex_bytes(data_hex, target_len)
        self._suspend_data_trace = True
        self.data_var.set(" ".join(padded[i:i + 2] for i in range(0, len(padded), 2)))
        self._suspend_data_trace = False

    def _build_command(self):
        channel = self._selected_channel()
        id_text = clean_hex_id(self.id_var.get())
        fd = self.fd_var.get()
        code = dlc_code_from_label(self.dlc_var.get())

        if self.rtr_var.get():
            frame = f"{id_text}#R{code}"
        else:
            max_len = self._max_data_bytes()
            target_len = dlc_length(code, fd)
            if target_len > max_len:
                raise ValueError(f"DLC {code} needs {target_len} bytes, this channel allows max {max_len}")
            data_hex = pad_or_trim_hex_bytes(clean_hex_bytes(self.data_var.get()), target_len)
            if fd:
                nibble = (int(self.esi_var.get()) << 1) | int(self.brs_var.get())
                frame = f"{id_text}##{nibble:X}{data_hex}"
            else:
                frame = f"{id_text}#{data_hex}"
        return f"twai_send {channel} {frame}"

    def _send_once(self):
        try:
            cmd = self._build_command()
        except ValueError as exc:
            self._log(f"[error] {exc}")
            return
        self._send_command(cmd)

    def _start_loop(self):
        try:
            self._validated_interval()
            self._build_command()
        except ValueError as exc:
            self._log(f"[error] {exc}")
            return
        self.start_loop_btn.configure(state="disabled")
        self.stop_loop_btn.configure(state="normal")
        self._loop_tick()

    def _validated_interval(self):
        interval = int(self.interval_var.get())
        if interval < MIN_LOOP_INTERVAL_MS:
            raise ValueError(f"Interval must be at least {MIN_LOOP_INTERVAL_MS} ms")
        return interval

    def _on_interval_committed(self, _evt=None):
        try:
            value = int(self.interval_var.get())
        except ValueError:
            return
        if value < MIN_LOOP_INTERVAL_MS:
            self.interval_var.set(str(MIN_LOOP_INTERVAL_MS))

    def _loop_tick(self):
        try:
            cmd = self._build_command()
            interval = self._validated_interval()
        except ValueError as exc:
            self._log(f"[error] {exc}")
            self.stop_loop()
            return
        self._send_command(cmd)
        self._loop_job = self._schedule(interval, self._loop_tick)

    def stop_loop(self):
        if self._loop_job is not None:
            self._cancel_schedule(self._loop_job)
            self._loop_job = None
        self.start_loop_btn.configure(state="normal")
        self.stop_loop_btn.configure(state="disabled")

    def _save_current(self):
        try:
            cmd = self._build_command()
        except ValueError as exc:
            self._log(f"[error] {exc}")
            return
        self.saved.insert("end", cmd)

    def _load_selected(self):
        sel = self.saved.curselection()
        if not sel:
            return
        cmd = self.saved.get(sel[0])
        _twai_send, channel, frame = cmd.split(" ", 2)
        self.channel_var.set(f"{channel_label(channel)} ({channel})")
        self.refresh_fd_state()
        if "##" in frame:
            id_text, rest = frame.split("##", 1)
            self.fd_var.set(True)
            self._on_fd_toggle()
            nibble = int(rest[0], 16)
            self.brs_var.set(bool(nibble & 1))
            self.esi_var.set(bool(nibble & 2))
            self.rtr_var.set(False)
            self.id_var.set(id_text)
            self.data_var.set(rest[1:])
        else:
            id_text, rest = frame.split("#", 1)
            self.id_var.set(id_text)
            if rest.startswith("R"):
                self.rtr_var.set(True)
                self._on_rtr_toggle()
                self.dlc_var.set(dlc_label(int(rest[1:]), False))
            else:
                self.rtr_var.set(False)
                self._on_rtr_toggle()
                self.data_var.set(rest)

    def _delete_selected(self):
        sel = self.saved.curselection()
        if sel:
            self.saved.delete(sel[0])


class DumpView(ttk.LabelFrame):
    MAX_TRACE_ROWS = 1000

    def __init__(self, master):
        super().__init__(master, text="Received Frames", padding=8)

        mode_row = ttk.Frame(self)
        mode_row.pack(fill="x")
        self.mode_var = tk.StringVar(value="trace")
        ttk.Radiobutton(
            mode_row, text="Autoscroll (trace)", variable=self.mode_var, value="trace", command=self._switch_mode
        ).pack(side="left")
        ttk.Radiobutton(
            mode_row, text="Last value (grid)", variable=self.mode_var, value="grid", command=self._switch_mode
        ).pack(side="left", padx=(12, 0))
        ttk.Button(mode_row, text="Clear", command=self.clear).pack(side="left", padx=(12, 0))

        self._tree_area = ttk.Frame(self)
        self._tree_area.pack(fill="both", expand=True, pady=(6, 0))

        self.trace_tree = ttk.Treeview(
            self._tree_area, columns=("channel", "id", "data", "time"), show="headings", height=10
        )
        for col, text, width in (
            ("channel", "Channel", 70), ("id", "CAN ID", 100), ("data", "Data", 340), ("time", "Time", 90)
        ):
            self.trace_tree.heading(col, text=text)
            self.trace_tree.column(col, width=width, anchor="w")
        self._trace_scroll = ttk.Scrollbar(self._tree_area, orient="vertical", command=self.trace_tree.yview)
        self.trace_tree.configure(yscrollcommand=self._trace_scroll.set)

        self.grid_tree = ttk.Treeview(
            self._tree_area, columns=("channel", "id", "data", "count", "time"), show="headings", height=10
        )
        for col, text, width in (
            ("channel", "Channel", 70), ("id", "CAN ID", 100), ("data", "Data", 340),
            ("count", "Count", 60), ("time", "Time", 90),
        ):
            self.grid_tree.heading(col, text=text)
            self.grid_tree.column(col, width=width, anchor="w")
        self._grid_scroll = ttk.Scrollbar(self._tree_area, orient="vertical", command=self.grid_tree.yview)
        self.grid_tree.configure(yscrollcommand=self._grid_scroll.set)

        self.trace_tree.pack(side="left", fill="both", expand=True)
        self._trace_scroll.pack(side="right", fill="y")

        self._grid_item_ids = {}
        self._grid_counts = {}
        self._current = "trace"

    def _switch_mode(self):
        mode = self.mode_var.get()
        if mode == self._current:
            return
        if mode == "grid":
            self.trace_tree.pack_forget()
            self._trace_scroll.pack_forget()
            self.grid_tree.pack(side="left", fill="both", expand=True)
            self._grid_scroll.pack(side="right", fill="y")
        else:
            self.grid_tree.pack_forget()
            self._grid_scroll.pack_forget()
            self.trace_tree.pack(side="left", fill="both", expand=True)
            self._trace_scroll.pack(side="right", fill="y")
        self._current = mode

    def clear(self):
        for item in self.trace_tree.get_children():
            self.trace_tree.delete(item)
        for item in self.grid_tree.get_children():
            self.grid_tree.delete(item)
        self._grid_item_ids.clear()
        self._grid_counts.clear()

    def add_frame(self, channel, can_id, data):
        chan_label = channel_label(channel)
        timestamp = time.strftime("%H:%M:%S")

        item = self.trace_tree.insert("", "end", values=(chan_label, can_id, data, timestamp))
        self.trace_tree.see(item)
        children = self.trace_tree.get_children()
        if len(children) > self.MAX_TRACE_ROWS:
            self.trace_tree.delete(children[0])

        key = (chan_label, can_id)
        count = self._grid_counts.get(key, 0) + 1
        self._grid_counts[key] = count
        if key in self._grid_item_ids:
            self.grid_tree.item(self._grid_item_ids[key], values=(chan_label, can_id, data, count, timestamp))
        else:
            self._grid_item_ids[key] = self.grid_tree.insert(
                "", "end", values=(chan_label, can_id, data, count, timestamp)
            )


class App(ttk.Frame):
    def __init__(self, master):
        super().__init__(master, padding=10)
        self.pack(fill="both", expand=True)
        self._msg_queue = queue.Queue()
        self._link = SerialLink(self._msg_queue.put)

        self._build_connection_row()
        self._build_controller_panels()
        self._build_send_loop_panel()
        self._build_dump_view()
        self._build_command_row()
        self._build_log()

        self.after(50, self._drain_queue)
        self._refresh_ports()

    def _build_connection_row(self):
        row = ttk.Frame(self)
        row.pack(fill="x", pady=(0, 10))

        ttk.Label(row, text="COM port:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(row, textvariable=self.port_var, width=30, state="readonly")
        self.port_box.pack(side="left", padx=(4, 6))

        ttk.Button(row, text="Refresh", command=self._refresh_ports).pack(side="left", padx=(0, 12))

        ttk.Label(row, text="Baud:").pack(side="left")
        self.serial_baud_var = tk.StringVar(value=str(DEFAULT_SERIAL_BAUD))
        ttk.Entry(row, textvariable=self.serial_baud_var, width=10).pack(side="left", padx=(4, 12))

        self.connect_btn = ttk.Button(row, text="Connect", command=self._toggle_connection)
        self.connect_btn.pack(side="left")

        self.conn_status_var = tk.StringVar(value="Disconnected")
        ttk.Label(row, textvariable=self.conn_status_var).pack(side="left", padx=(12, 0))

    def _build_controller_panels(self):
        row = ttk.Frame(self)
        row.pack(fill="x", pady=(0, 10))
        self.twai_panels = {}
        for i, channel in enumerate(CHANNELS):
            defaults = DEFAULT_GPIO[channel]
            panel = TwaiControllerPanel(
                row, channel, self._send_command, default_tx=defaults["tx"], default_rx=defaults["rx"]
            )
            panel.pack(side="left", fill="x", expand=True, padx=(0 if i == 0 else 6, 6 if i == 0 else 0))
            panel.fd_enabled_var.trace_add("write", lambda *_a: self.send_loop_panel.refresh_fd_state())
            self.twai_panels[channel] = panel

    def _get_channel_fd_enabled(self, channel):
        return self.twai_panels[channel].fd_enabled_var.get()

    def _build_send_loop_panel(self):
        self.send_loop_panel = SendLoopPanel(
            self, self._send_command, self._log, self.after, self.after_cancel, self._get_channel_fd_enabled
        )
        self.send_loop_panel.pack(fill="x", pady=(0, 10))

    def _build_dump_view(self):
        self.dump_view = DumpView(self)
        self.dump_view.pack(fill="both", expand=True, pady=(0, 10))

    def _build_command_row(self):
        row = ttk.Frame(self)
        row.pack(fill="x", pady=(0, 6))
        ttk.Label(row, text="Command:").pack(side="left")
        self.cmd_var = tk.StringVar()
        entry = ttk.Entry(row, textvariable=self.cmd_var)
        entry.pack(side="left", fill="x", expand=True, padx=(4, 6))
        entry.bind("<Return>", lambda _evt: self._send_manual_command())
        ttk.Button(row, text="Send", command=self._send_manual_command).pack(side="left")

    def _build_log(self):
        frame = ttk.Frame(self)
        frame.pack(fill="both", expand=False)
        self.log_text = tk.Text(frame, height=8, state="disabled", wrap="none")
        scroll = ttk.Scrollbar(frame, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scroll.set)
        self.log_text.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_box["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def _toggle_connection(self):
        if self._link.is_open:
            self.send_loop_panel.stop_loop()
            self._link.close()
            self.conn_status_var.set("Disconnected")
            self.connect_btn.configure(text="Connect")
            return

        port = self.port_var.get()
        if not port:
            self._log("[error] No COM port selected")
            return
        try:
            baud = int(self.serial_baud_var.get())
        except ValueError:
            self._log("[error] Invalid baud rate")
            return
        try:
            self._link.open(port, baud)
        except serial.SerialException as exc:
            self._log(f"[error] Could not open {port}: {exc}")
            return
        self.conn_status_var.set(f"Connected ({port} @ {baud})")
        self.connect_btn.configure(text="Disconnect")

    def _send_command(self, command):
        if not self._link.is_open:
            self._log("[error] Not connected to a COM port")
            return
        self._log(f"> {command}")
        try:
            self._link.send(command)
        except RuntimeError as exc:
            self._log(f"[error] {exc}")

    def _send_manual_command(self):
        cmd = self.cmd_var.get().strip()
        if not cmd:
            return
        self._send_command(cmd)
        self.cmd_var.set("")

    def _log(self, text):
        self.log_text.configure(state="normal")
        self.log_text.insert("end", text + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _drain_queue(self):
        while True:
            try:
                line = self._msg_queue.get_nowait()
            except queue.Empty:
                break
            parsed = parse_dump_line(line)
            if parsed:
                self.dump_view.add_frame(*parsed)
            else:
                self._log(line)
        self.after(50, self._drain_queue)

    def on_close(self):
        self.send_loop_panel.stop_loop()
        self._link.close()


def main():
    root = tk.Tk()
    root.title("TWAI Utils - USB Control Panel")
    root.geometry("820x900")
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.on_close(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()
