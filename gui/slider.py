"""
Concentric Tube Robot — Control GUI
Requires: pip install pyserial
"""

import tkinter as tk
from tkinter import ttk
import serial, serial.tools.list_ports, threading, time

# ── Units / limits ─────────────────────────────────────────────
MAX_IN      = 3.0
MIN_IN      = 0.0
MM_PER_IN   = 25.4

# ── Calibration (must match Arduino) ───────────────────────────
ENC_COUNTS_PER_MM  = 79.26  # measured: ~2014 counts per inch


class RobotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("CTR Controller")
        self.root.configure(bg="#1a1a1a")
        self.ser       = None
        self.connected = False
        self.running   = True
        self.lin_in    = 0.0   # current linear feedback in inches
        self.vars      = {}    # all tk variables keyed by command token
        self._build()
        root.protocol("WM_DELETE_WINDOW", self._close)

    # ── UI ─────────────────────────────────────────────────────
    def _build(self):
        LBG, FG, DIM = "#1a1a1a", "#ddd", "#666"

        # Connection row
        cf = tk.Frame(self.root, bg="#222", padx=6, pady=4)
        cf.pack(fill="x")
        tk.Label(cf, text="Port:", bg="#222", fg=DIM).pack(side="left")
        self.port_var = tk.StringVar()
        self.port_cb  = ttk.Combobox(cf, textvariable=self.port_var, width=13)
        self.port_cb.pack(side="left", padx=2)
        tk.Button(cf, text="↺", command=self._refresh_ports,
                  bg="#222", fg=DIM, relief="flat").pack(side="left")
        self.conn_btn = tk.Button(cf, text="Connect", command=self._toggle_connect,
                                  bg="#4fc3f7", fg="#000", relief="flat", padx=8, pady=2)
        self.conn_btn.pack(side="left", padx=6)
        tk.Button(cf, text="HOME", command=self._home,
                  bg="#333", fg="#ffb74d", relief="flat", padx=8, pady=2).pack(side="left", padx=2)
        tk.Button(cf, text="E-STOP", command=self._estop,
                  bg="#c62828", fg="#fff", relief="flat", padx=8, pady=2).pack(side="left", padx=2)
        self.status_lbl = tk.Label(cf, text="● Disconnected", fg="#ef5350", bg="#222")
        self.status_lbl.pack(side="left", padx=8)
        self._refresh_ports()

        # Position frame
        pf = tk.LabelFrame(self.root, text=" Position ", bg=LBG, fg=DIM, relief="groove")
        pf.pack(fill="x", padx=6, pady=3)

        self._slider_row(pf, 0, "Rotation", "R", -360, 360, 1, "deg", "#4fc3f7")

        # Linear absolute slider (inches)
        lf = tk.Frame(pf, bg=LBG)
        lf.grid(row=1, column=0, columnspan=5, sticky="ew", padx=4, pady=3)
        tk.Label(lf, text="Linear:", bg=LBG, fg=DIM, width=9, anchor="w").pack(side="left")
        self.lin_lbl = tk.Label(lf, text="0.00 in", bg="#252525", fg="#81c784",
                                width=8, relief="sunken", anchor="e")
        self.lin_lbl.pack(side="left", padx=(0, 4))
        self.lin_target_in = tk.DoubleVar(value=0.0)
        self.lin_scale = tk.Scale(
            lf,
            from_=MIN_IN,
            to=MAX_IN,
            resolution=0.005,
            orient="horizontal",
            variable=self.lin_target_in,
            bg=LBG,
            fg="#81c784",
            troughcolor="#333",
            highlightthickness=0,
            showvalue=False,
            length=260,
            command=self._on_linear_target_change,
        )
        self.lin_scale.pack(side="left", padx=2)
        self.lin_target_lbl = tk.Label(
            lf, text="0.00 in", bg="#252525", fg="#81c784",
            width=8, relief="sunken", anchor="e"
        )
        self.lin_target_lbl.pack(side="left", padx=(3, 2))
        tk.Label(lf, text="in", bg=LBG, fg=DIM).pack(side="left", padx=(0, 2))

        self._slider_row(pf, 2, "Stepper",  "S", -360, 360, 1, "deg", "#ffb74d")

        # PID frame — 2 axes × 3 params in a grid
        gf = tk.LabelFrame(self.root, text=" PID Gains ", bg=LBG, fg=DIM, relief="groove")
        gf.pack(fill="x", padx=6, pady=3)
        pid_defs = [
            ("RKP", 0, 2.0,  0.01, 0.10, "#4fc3f7"), ("RKI", 0, 0.5, 0.001, 0.00, "#4fc3f7"),
            ("RKD", 0, 1.0,  0.01, 0.00, "#4fc3f7"), ("LKP", 0, 2.0,  0.01, 0.30, "#81c784"),
            ("LKI", 0, 0.5, 0.001, 0.00, "#81c784"), ("LKD", 0, 1.0,  0.01, 0.00, "#81c784"),
        ]
        for i, (key, mn, mx, res, default, color) in enumerate(pid_defs):
            r, c = divmod(i, 3)
            var = tk.DoubleVar(value=default)
            self.vars[key] = var
            cell = tk.Frame(gf, bg=LBG)
            cell.grid(row=r, column=c, padx=4, pady=1, sticky="ew")
            tk.Label(cell, text=key, bg=LBG, fg=DIM, width=5, anchor="w",
                     font=("Consolas", 8)).pack(side="left")
            tk.Scale(cell, from_=mn, to=mx, resolution=res, orient="horizontal",
                     variable=var, bg=LBG, fg=color, troughcolor="#333",
                     highlightthickness=0, showvalue=False, length=90,
                     command=lambda v: self._send_pid()).pack(side="left")
            e = tk.Entry(cell, textvariable=var, width=6, bg="#252525", fg=color,
                         relief="flat", justify="right", font=("Consolas", 8))
            e.pack(side="left", padx=2)
            e.bind("<Return>",   lambda ev: self._send_pid())
            e.bind("<FocusOut>", lambda ev: self._send_pid())

        # Encoder readback
        ef = tk.LabelFrame(self.root, text=" Live Feedback ", bg=LBG, fg=DIM, relief="groove")
        ef.pack(fill="x", padx=6, pady=3)
        self.enc = {}
        readback_defs = [
            ("enc_rot",  "Rot enc",    "#4fc3f7"), ("enc_lin",  "Lin enc",  "#81c784"),
            ("enc_step", "Steps",      "#ffb74d"), ("sp_rot",   "Rot SP",   DIM),
            ("sp_lin",   "Lin SP",     DIM),       ("out_rot",  "Rot out",  DIM),
            ("out_lin",  "Lin out",    DIM),
        ]
        for i, (key, label, color) in enumerate(readback_defs):
            r, c = divmod(i, 4)
            tk.Label(ef, text=label+":", bg=LBG, fg=DIM,
                     font=("Consolas", 8)).grid(row=r, column=c*2, padx=4, sticky="e")
            lbl = tk.Label(ef, text="—", bg=LBG, fg=color,
                           font=("Consolas", 8), width=8, anchor="e")
            lbl.grid(row=r, column=c*2+1, padx=2, sticky="w")
            self.enc[key] = lbl

        # Serial log
        self.log = tk.Text(self.root, height=4, bg="#111", fg=DIM,
                           font=("Consolas", 8), state="disabled", relief="flat")
        self.log.pack(fill="x", padx=6, pady=(0, 4))

    def _slider_row(self, parent, row, label, key, mn, mx, res, unit, color):
        # target var is always the actual setpoint we send
        target = tk.DoubleVar(value=0.0)
        self.vars[key] = target
        BG = "#1a1a1a"
        tk.Label(parent, text=label+":", bg=BG, fg="#666",
                 width=9, anchor="w").grid(row=row, column=0, padx=4, pady=2, sticky="w")

        tk.Scale(parent, from_=mn, to=mx, resolution=res, orient="horizontal",
                 variable=target, bg=BG, fg=color, troughcolor="#333",
                 highlightthickness=0, showvalue=False, length=220,
                 command=lambda v, k=key: self._send_pos()
                 ).grid(row=row, column=1, padx=4)

        e = tk.Entry(parent, textvariable=target, width=7, bg="#252525", fg=color,
                     relief="flat", justify="right", font=("Consolas", 9))
        e.grid(row=row, column=2, padx=2)
        e.bind("<Return>",   lambda ev: self._send_pos())
        e.bind("<FocusOut>", lambda ev: self._send_pos())
        tk.Label(parent, text=unit, bg=BG, fg="#555",
                 font=("Consolas", 8)).grid(row=row, column=3, padx=2)

    # ── Linear absolute slider controls ────────────────────────
    def _on_linear_target_change(self, val):
        try:
            target_in = float(val)
        except ValueError:
            target_in = self.lin_in
        target_in = max(MIN_IN, min(MAX_IN, target_in))
        self.lin_target_lbl.config(text=f"{target_in:.2f} in")
        # Send absolute linear target (in mm) so slider position is persistent
        # and not dependent on encoder update timing.
        self._send(f"L:{target_in * MM_PER_IN:.2f}\n")

    def _update_lin_display(self):
        self.lin_lbl.config(text=f"{self.lin_in:.2f} in")

    # ── Send helpers ───────────────────────────────────────────
    def _send_pos(self):
        r = self.vars["R"].get()
        s = self.vars["S"].get()
        self._send(f"R:{r:.1f},S:{s:.1f}\n")

    def _send_pid(self):
        keys = ["RKP", "RKI", "RKD", "LKP", "LKI", "LKD"]
        self._send(",".join(f"{k}:{self.vars[k].get():.3f}" for k in keys) + "\n")

    def _home(self):
        self._send("HOME\n")
        self.lin_in = 0.0
        self.lin_target_in.set(0.0)
        self.lin_target_lbl.config(text="0.00 in")
        self._update_lin_display()
        for k in ["R", "S"]:
            self.vars[k].set(0.0)

    def _estop(self):
        self._home()
        self._log("⚠ E-STOP sent")

    def _send(self, cmd):
        if self.ser and self.connected:
            try:
                self.ser.write(cmd.encode())
            except Exception as e:
                self._log(f"Send error: {e}")

    # ── Connection ─────────────────────────────────────────────
    def _toggle_connect(self):
        if self.connected:
            self.connected = False
            self.ser.close()
            self.conn_btn.config(text="Connect", bg="#4fc3f7", fg="#000")
            self.status_lbl.config(text="● Disconnected", fg="#ef5350")
            self._log("Disconnected")
        else:
            port = self.port_var.get()
            if not port:
                self._log("No port selected"); return
            try:
                self.ser = serial.Serial(port, 9600, timeout=0.1)
                self.connected = True
                self.conn_btn.config(text="Disconnect", bg="#c62828", fg="#fff")
                self.status_lbl.config(text=f"● {port}", fg="#81c784")
                self._log(f"Connected: {port} @ 9600")
                threading.Thread(target=self._read_loop, daemon=True).start()
            except Exception as e:
                self._log(f"Connect failed: {e}")

    # ── Serial read loop (background thread) ──────────────────
    def _read_loop(self):
        while self.connected and self.running:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode(errors="ignore").strip()
                    if line.startswith("ENC:"):
                        self._parse_report(line)
                    else:
                        self._log(line)
            except: pass
            time.sleep(0.05)

    def _parse_report(self, line):
        """Parse: ENC:rot,lin,step|SP:rot,lin,step|OUT:rot,lin"""
        try:
            enc, sp, out = [s.split(",") for s in
                            [p.split(":", 1)[1] for p in line.split("|")]]

            updates = dict(enc_rot=enc[0], enc_lin=enc[1], enc_step=enc[2],
                           sp_rot=sp[0],   sp_lin=sp[1],
                           out_rot=out[0], out_lin=out[1])
            for k, v in updates.items():
                self.root.after(0, lambda k=k, v=v:
                                self.enc[k].config(text=f"{float(v):.1f}"))

            # ── Closed-loop position sync ──────────────────────
            # When encoders are calibrated, update lin_in from actual encoder reading.
            # This keeps the display honest even if a manual force moved the tube.
            enc_lin_mm = float(enc[1]) / ENC_COUNTS_PER_MM
            enc_lin_in = enc_lin_mm / MM_PER_IN
            self.root.after(0, lambda v=enc_lin_in: (
                setattr(self, "lin_in", round(max(MIN_IN, min(MAX_IN, v)), 3)),
                self._update_lin_display()
            ))
        except: pass

    # ── Misc ───────────────────────────────────────────────────
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports: self.port_var.set(ports[0])

    def _log(self, msg):
        def _do():
            self.log.config(state="normal")
            self.log.insert("end", msg + "\n")
            self.log.see("end")
            self.log.config(state="disabled")
        self.root.after(0, _do)

    def _close(self):
        self.running = False
        if self.ser: self.ser.close()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("740x560")
    RobotGUI(root)
    root.mainloop()