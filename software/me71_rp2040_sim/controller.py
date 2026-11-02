#!/usr/bin/env python3
"""ME7.1 Pico crank simulator / 12-channel event monitor GUI."""
import argparse
from collections import deque
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
import serial
from serial.tools import list_ports

class Connection:
    def __init__(self, port, inbox):
        self.port = port
        self.inbox = inbox
        self.running = True
        self.outbox = queue.Queue()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def send(self, command):
        self.outbox.put(command)

    def close(self):
        self.running = False

    def run(self):
        try:
            with serial.Serial(self.port, 115200, timeout=0.03, write_timeout=1) as port:
                self.inbox.put(("connected", self.port))
                while self.running:
                    try:
                        while True:
                            port.write((self.outbox.get_nowait() + "\n").encode("ascii"))
                    except queue.Empty:
                        pass
                    line = port.readline().decode("ascii", "replace").strip()
                    if line:
                        self.inbox.put(("line", line))
        except (serial.SerialException, OSError) as e:
            self.inbox.put(("error", str(e)))
        finally:
            self.inbox.put(("disconnected", ""))

class App:
    def __init__(self, root, initial_port=None):
        self.root = root
        root.title("Audi 2.7T ME7.1 | RP2040 12-channel bench monitor")
        root.geometry("1200x960")
        self.inbox = queue.Queue()
        self.connection = None
        self.port = tk.StringVar(value=initial_port or "")
        self.rpm = tk.StringVar(value="800")
        self.crank_rpm = tk.StringVar(value="250")
        self.crank_hold = tk.StringVar(value="1.0")
        self.ramp_seconds = tk.StringVar(value="3.0")
        self.start_status = tk.StringVar(value="Stopped")
        self.start_sequence = None
        self.start_timer = None
        self.tdc = tk.StringVar(value="0")
        self.wheel_teeth = tk.StringVar(value="60")
        self.wheel_missing = tk.StringVar(value="2")
        self.wheel_invert = tk.BooleanVar(value=False)
        self.stream = tk.BooleanVar(value=True)
        self.inj_active_low = tk.StringVar(value="000000")
        self.coil_active_low = tk.StringVar(value="111111")
        self.cam_vars = {"G40": [tk.StringVar(value="120"), tk.StringVar(value="60"), tk.BooleanVar(value=False)],
                         "G163": [tk.StringVar(value="480"), tk.StringVar(value="60"), tk.BooleanVar(value=False)]}
        self.status = tk.StringVar(value="Disconnected")
        self.active = {"I": [False]*6, "S": [False]*6}
        self.counts = [[0, 0, 0] for _ in range(12)]
        self.last_angle = ["—"]*12
        self.last_relative = ["—"]*12
        self.drop = 0
        self.event_drop = 0
        self.counter = 0
        self.raster_events = deque(maxlen=25000)
        self.raster_pending = {}
        self.raster_cycles = tk.IntVar(value=8)
        self.raster_fade = tk.IntVar(value=5)
        self.raster_angle_mode = tk.StringVar(value="Absolute (0–720°)")
        self.raster_show_off = tk.BooleanVar(value=True)
        self.raster_paused = tk.BooleanVar(value=False)
        self.raster_max_epoch = None
        self.raster_dirty = True
        self.build()
        self.refresh_ports()
        if initial_port:
            self.connect()
        root.after(25, self.poll)
        root.protocol("WM_DELETE_WINDOW", self.close)

    def build(self):
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill="x")
        ttk.Label(top, text="Serial port:").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(top, textvariable=self.port, width=21)
        self.port_combo.grid(row=0, column=1, padx=5)
        ttk.Button(top, text="Refresh", command=self.refresh_ports).grid(row=0,column=2,padx=4)
        self.connect_button = ttk.Button(top, text="Connect", command=self.connect)
        self.connect_button.grid(row=0,column=3,padx=8)
        ttk.Checkbutton(top,text="Stream edges",variable=self.stream,command=self.set_stream).grid(row=1,column=0,columnspan=2,sticky="w",pady=8)
        ttk.Label(top,text="Injector active-low (INJ6→INJ1):").grid(row=1,column=2,sticky="e",padx=(8,2))
        ttk.Entry(top,textvariable=self.inj_active_low,width=8).grid(row=1,column=3)
        ttk.Label(top,text="Coil active-low (COIL6→COIL1):").grid(row=1,column=4,sticky="e",padx=(8,2))
        ttk.Entry(top,textvariable=self.coil_active_low,width=8).grid(row=1,column=5)
        ttk.Button(top,text="Apply polarity",command=self.set_polarity).grid(row=1,column=6,padx=5)
        ttk.Label(top,textvariable=self.status).grid(row=1,column=7,sticky="w")
        start = ttk.LabelFrame(self.root, text="Engine start simulation (crank → ramp to idle)", padding=7)
        start.pack(fill="x", padx=10, pady=(0,6))
        ttk.Label(start, text="Cranking RPM:").grid(row=0,column=0,padx=3)
        ttk.Entry(start, textvariable=self.crank_rpm,width=7).grid(row=0,column=1,padx=3)
        ttk.Label(start, text="Crank hold (s):").grid(row=0,column=2,padx=3)
        ttk.Entry(start, textvariable=self.crank_hold,width=7).grid(row=0,column=3,padx=3)
        ttk.Label(start, text="Ramp to idle (s):").grid(row=0,column=4,padx=3)
        ttk.Combobox(start, textvariable=self.ramp_seconds, values=("1","2","3","4","5"),width=6).grid(row=0,column=5,padx=3)
        ttk.Label(start, text="Idle / manual RPM:").grid(row=0,column=6,padx=(10,3))
        ttk.Entry(start, textvariable=self.rpm,width=8).grid(row=0,column=7,padx=3)
        ttk.Button(start,text="Set RPM",command=self.set_rpm).grid(row=0,column=8,padx=5)
        ttk.Button(start,text="START ENGINE",command=self.start_engine).grid(row=0,column=9,padx=5)
        ttk.Button(start,text="ABORT / STOP",command=self.stop_engine).grid(row=0,column=10,padx=5)
        ttk.Label(start,textvariable=self.start_status).grid(row=1,column=0,columnspan=11,sticky="w",pady=(5,0))
        wheel=ttk.LabelFrame(self.root,text="Crank trigger wheel (nominal positions / missing positions per revolution)",padding=7)
        wheel.pack(fill="x",padx=10,pady=(0,6))
        ttk.Label(wheel,text="Total teeth:").grid(row=0,column=0,padx=3)
        ttk.Entry(wheel,textvariable=self.wheel_teeth,width=7).grid(row=0,column=1,padx=3)
        ttk.Label(wheel,text="Missing:").grid(row=0,column=2,padx=3)
        ttk.Entry(wheel,textvariable=self.wheel_missing,width=7).grid(row=0,column=3,padx=3)
        ttk.Checkbutton(wheel,text="Invert crank signal",variable=self.wheel_invert).grid(row=0,column=4,padx=6)
        ttk.Button(wheel,text="Apply wheel (restarts phase)",command=self.set_wheel).grid(row=0,column=5,padx=6)
        ttk.Button(wheel,text="Read wheel",command=lambda:self.send("WHEEL?")).grid(row=0,column=6,padx=6)
        cams=ttk.LabelFrame(self.root,text="Camshaft simulation (angles relative to first tooth after 60-2 gap; provisional phase)",padding=7)
        cams.pack(fill="x",padx=10,pady=(0,6))
        for row, name in enumerate(("G40", "G163")):
            start,width,invert=self.cam_vars[name]
            ttk.Label(cams,text=name).grid(row=row,column=0,padx=5)
            ttk.Label(cams,text="Start °:").grid(row=row,column=1)
            ttk.Entry(cams,width=8,textvariable=start).grid(row=row,column=2,padx=3)
            ttk.Label(cams,text="Width °:").grid(row=row,column=3)
            ttk.Entry(cams,width=8,textvariable=width).grid(row=row,column=4,padx=3)
            ttk.Checkbutton(cams,text="Invert",variable=invert).grid(row=row,column=5,padx=8)
            ttk.Button(cams,text="Apply (restarts phase)",command=lambda name=name:self.set_cam(name)).grid(row=row,column=6,padx=5)
        ttk.Button(cams,text="Read cam settings",command=lambda:self.send("CAM?")).grid(row=0,column=7,padx=8)
        tdcrow=ttk.Frame(self.root,padding=(12,0,12,6));tdcrow.pack(fill="x")
        ttk.Label(tdcrow,text="Cylinder 1 firing TDC angle from 720° marker (calibrate):").pack(side="left")
        ttk.Entry(tdcrow,textvariable=self.tdc,width=8).pack(side="left",padx=6)
        ttk.Button(tdcrow,text="Apply TDC offset",command=self.set_tdc).pack(side="left")
        ttk.Label(tdcrow,text="Relative timing: negative=before TDC / positive=after TDC").pack(side="left",padx=15)
        frame=ttk.LabelFrame(self.root,text="PIO / DMA capture (125 kSamples/s, 8 µs resolution)",padding=8)
        frame.pack(fill="x",padx=10)
        self.table=ttk.Treeview(frame,columns=("signal","on","off","width","angle","relative","state"),show="headings",height=12)
        for key,title,w in [("signal","ECU output",155),("on","Firings (ON)",160),("off","End edges (OFF)",160),("width","Last pulse width (µs)",135),("angle","Latest ON (°/720)",135),("relative","ON vs TDC (°)",130),("state","State",85)]:
            self.table.heading(key,text=title)
            self.table.column(key,width=w,anchor="center")
        for i in range(12):
            label=f"{'Injector' if i < 6 else 'Spark'} {i%6+1}"
            self.table.insert("", "end", iid=str(i),values=(label,0,0,0,"—","—","OFF"))
        self.table.pack(fill="x")
        self.stats=ttk.Label(frame,text="Sample drops: 0 | Event queue drops: 0")
        self.stats.pack(anchor="w",pady=(8,0))
        notebook=ttk.Notebook(self.root)
        notebook.pack(fill="both",expand=True,padx=10,pady=8)
        plot_tab=ttk.Frame(notebook)
        log_tab=ttk.Frame(notebook)
        notebook.add(plot_tab,text="720° firing persistence plot")
        notebook.add(log_tab,text="Event log")
        plot_controls=ttk.Frame(plot_tab,padding=5)
        plot_controls.pack(fill="x")
        ttk.Label(plot_controls,text="Persistence cycles:").pack(side="left")
        ttk.Spinbox(plot_controls,from_=1,to=200,width=6,textvariable=self.raster_cycles,
                    command=self.redraw_raster).pack(side="left",padx=5)
        ttk.Checkbutton(plot_controls,text="Show pulse duration (ON→OFF)",
                        variable=self.raster_show_off,command=self.redraw_raster).pack(side="left",padx=8)
        ttk.Checkbutton(plot_controls,text="Freeze display",variable=self.raster_paused,
                        command=self.redraw_raster).pack(side="left",padx=8)
        ttk.Button(plot_controls,text="Clear plot",command=self.clear_raster).pack(side="left",padx=8)
        ttk.Label(plot_controls,text="INJ=green, SPK=orange; grouped by cylinder").pack(side="right")
        view_controls=ttk.Frame(plot_tab,padding=(5,0,5,5))
        view_controls.pack(fill="x")
        ttk.Label(view_controls,text="Y-axis:").pack(side="left")
        ttk.Combobox(view_controls,textvariable=self.raster_angle_mode,
                     values=("Absolute (0–720°)","Relative to cylinder TDC (−360° to +360°)"),
                     state="readonly",width=45).pack(side="left",padx=5)
        self.raster_angle_mode.trace_add("write",lambda *_:self.redraw_raster())
        ttk.Label(view_controls,text="Fade speed (1=slow, 10=fast):").pack(side="left",padx=(20,2))
        ttk.Scale(view_controls,from_=1,to=10,variable=self.raster_fade,
                  command=lambda value:self.redraw_raster()).pack(side="left",fill="x",expand=True,padx=5)
        ttk.Label(view_controls,textvariable=self.raster_fade,width=2).pack(side="left")
        plot_container=ttk.Frame(plot_tab)
        plot_container.pack(fill="both",expand=True)
        self.raster=tk.Canvas(plot_container,background="#101820",height=340,highlightthickness=0)
        self.raster.pack(fill="both",expand=True)
        self.raster.bind("<Configure>",lambda event:self.redraw_raster())
        self.raster_info=ttk.Label(plot_tab,text="Waiting for synchronized ANG events.",padding=4)
        self.raster_info.pack(anchor="w")
        self.log=tk.Text(log_tab,height=14,wrap="none",font=("Courier",10),state="disabled")
        scrollbar=ttk.Scrollbar(log_tab,orient="vertical",command=self.log.yview)
        self.log.configure(yscrollcommand=scrollbar.set)
        self.log.pack(side="left",fill="both",expand=True)
        scrollbar.pack(side="right",fill="y")

    def clear_raster(self):
        self.raster_events.clear()
        self.raster_pending.clear()
        self.raster_max_epoch=None
        self.redraw_raster()

    def add_raster_edge(self,chan,on,angle,relative,epoch):
        """Only synchronized ANG messages contribute; each cycle has a distinct identity."""
        if self.raster_paused.get():
            return
        if self.raster_max_epoch is not None and epoch < self.raster_max_epoch - 500:
            # Firmware cycle counter reset (new session / waveform restart).
            self.clear_raster()
        self.raster_max_epoch=max(epoch,self.raster_max_epoch if self.raster_max_epoch is not None else epoch)
        self.raster_events.append((epoch,chan,on,angle,relative))
        self.raster_dirty=True

    def redraw_raster(self):
        if not hasattr(self,"raster"):
            return
        c=self.raster
        c.delete("all")
        w=max(300,c.winfo_width()); h=max(220,c.winfo_height())
        l,r,t,b=54,16,42,35
        pw=max(50,w-l-r); ph=max(100,h-t-b)
        cyl_count = 6
        slot_w = pw / cyl_count
        inj_offset = -max(4, min(10, slot_w * 0.12))
        spk_offset = -inj_offset
        relative_mode = self.raster_angle_mode.get().startswith("Relative")
        def y(angle):return t+ph*(angle+360 if relative_mode else angle)/720.0
        def visible_angle(abs_angle,relative):return relative if relative_mode else abs_angle
        def x(ch):
            cyl = ch % 6
            base = l + slot_w * (cyl + 0.5)
            return base + (inj_offset if ch < 6 else spk_offset)
        for deg in range(-360 if relative_mode else 0,361 if relative_mode else 721,60):
            yy=y(deg)
            c.create_line(l,yy,w-r,yy,fill="#344351" if deg%120==0 else "#24313d")
            c.create_text(l-8,yy,text=str(deg)+"°",anchor="e",fill="#b9c9d5",font=("TkDefaultFont",9))
        for cyl in range(cyl_count):
            left = l + slot_w * cyl
            center = left + slot_w / 2
            c.create_rectangle(left,t,left+slot_w,t+ph,outline="#22303b")
            c.create_line(center,t,center,t+ph,fill="#41515d",dash=(2,6))
            c.create_text(center,t-22,text=f"CYL {cyl+1}",fill="#d6e3ec",font=("TkDefaultFont",10,"bold"))
            c.create_text(center+inj_offset,t-8,text="INJ",fill="#54d9b2",font=("TkDefaultFont",8,"bold"))
            c.create_text(center+spk_offset,t-8,text="SPK",fill="#f6b56c",font=("TkDefaultFont",8,"bold"))
        max_epoch=self.raster_max_epoch
        try: window=max(1,min(200,int(self.raster_cycles.get())))
        except (ValueError,tk.TclError):window=20
        if max_epoch is not None:
            visible=[ev for ev in self.raster_events if max_epoch-window+1 <= ev[0] <= max_epoch]
            pending={}
            for epoch,ch,on,absolute,relative in visible:
                angle=visible_angle(absolute,relative)
                if not (-360 <= angle <= 360 if relative_mode else 0 <= angle <= 720):
                    continue
                xx=x(ch)
                age=max_epoch-epoch
                speed=max(1,min(10,int(self.raster_fade.get())))
                # Expanding the decay exponent increases fade speed while keeping
                # the user-selected number of historical cycles as an upper bound.
                progress=age/max(1,window)
                strength=max(0.0,1.0-progress)**(0.5+speed*0.45)
                base=(85,232,179) if ch<6 else (255,179,78)
                background=(16,24,32)
                color="#"+"".join(f"{round(bg+(fg-bg)*strength):02x}" for bg,fg in zip(background,base))
                if on:
                    c.create_oval(xx-4,y(angle)-4,xx+4,y(angle)+4,fill=color,outline="")
                    pending[ch]=(epoch,angle,absolute)
                elif self.raster_show_off.get():
                    c.create_line(xx-5,y(angle),xx+5,y(angle),fill=color,width=2)
                    if ch in pending:
                        start_epoch,start_angle,start_absolute=pending.pop(ch)
                        if epoch==start_epoch and angle >= start_angle and angle-start_angle <= 360:
                            c.create_line(xx,y(start_angle),xx,y(angle),fill=color,width=2)
                        elif not relative_mode and (epoch==start_epoch+1 or (epoch==start_epoch and angle<start_angle)):
                            c.create_line(xx,y(start_angle),xx,y(720),fill=color,width=2)
                            c.create_line(xx,y(0),xx,y(angle),fill=color,width=2)
                        # In relative mode a jump over +/-360 is a coordinate
                        # discontinuity, not a long pulse; never join across it.
            self.raster_info.config(text=f"Showing {len(visible)} edges over {window} cycles, fade speed {self.raster_fade.get()}/10; newest cycle={max_epoch}. "
                                         "Each cylinder column overlays injector (green) and spark (orange); circles=ON, ticks=OFF, lines=pulse duration. Older events fade quickly. Historical ON/OFF pairing is approximate when edges are dropped.")
        else:
            self.raster_info.config(text="Waiting for synchronized ANG events (enable Stream edges; run RPM > 0).")
        self.raster_dirty=False

    def refresh_ports(self):
        ports=[p.device for p in list_ports.comports()]
        self.port_combo["values"]=ports
        if ports and not self.port.get(): self.port.set(ports[0])

    def connect(self):
        if self.connection:
            self.cancel_start_sequence()
            self.connection.close()
            self.connection=None
            self.connect_button.config(text="Connect")
            self.status.set("Disconnected")
            return
        if not self.port.get():
            messagebox.showerror("Port", "Select a COM/tty port first.")
            return
        self.connection=Connection(self.port.get(),self.inbox)
        self.connect_button.config(text="Disconnect")
        self.status.set("Connecting…")

    def send(self, cmd):
        if self.connection:
            self.connection.send(cmd)
        else:
            self.status.set("Not connected")

    def cancel_start_sequence(self):
        self.start_sequence = None
        if self.start_timer is not None:
            self.root.after_cancel(self.start_timer)
            self.start_timer = None

    def stop_engine(self):
        self.cancel_start_sequence()
        self.send("RPM 0")
        self.start_status.set("Stopped (0 RPM)")

    def start_engine(self):
        if not self.connection:
            messagebox.showerror("Start simulation", "Connect to the RP2040 first.")
            return
        try:
            crank = int(self.crank_rpm.get())
            idle = int(self.rpm.get())
            hold = float(self.crank_hold.get())
            ramp = float(self.ramp_seconds.get())
            if not 100 <= crank <= 12000 or not 100 <= idle <= 12000:
                raise ValueError()
            if idle <= crank or not 0 <= hold <= 15 or not 1 <= ramp <= 5:
                raise ValueError()
        except ValueError:
            messagebox.showerror("Start simulation", "Use 100–12000 cranking RPM, a higher idle RPM, 0–15 s crank hold, and 1–5 s ramp.")
            return
        self.cancel_start_sequence()
        now = time.monotonic()
        self.start_sequence = {"crank":crank,"idle":idle,"hold_until":now+hold,
                               "ramp":ramp,"last_sent":crank}
        self.send(f"RPM {crank}")
        self.start_status.set(f"Cranking at {crank} RPM")
        self.start_timer = self.root.after(50,self.advance_start_sequence)

    def advance_start_sequence(self):
        self.start_timer = None
        seq = self.start_sequence
        if seq is None:
            return
        if not self.connection:
            self.cancel_start_sequence()
            self.start_status.set("Disconnected — start cancelled")
            return
        elapsed = time.monotonic()-seq["hold_until"]
        if elapsed < 0:
            self.start_status.set(f"Cranking at {seq['crank']} RPM ({-elapsed:.1f}s remaining)")
        else:
            fraction = min(1.0,max(0.0,elapsed/seq["ramp"]))
            # Ease-out gives a more natural rise to idle without an abrupt final step.
            smooth = 1.0-(1.0-fraction)**2
            target = round(seq["crank"] + (seq["idle"]-seq["crank"])*smooth)
            if target != seq["last_sent"]:
                self.send(f"RPM {target}")
                seq["last_sent"] = target
            if fraction >= 1.0:
                self.start_status.set(f"Running at idle: {seq['idle']} RPM")
                self.start_sequence = None
                return
            self.start_status.set(f"Starting → {target} RPM ({fraction*100:.0f}% ramp)")
        self.start_timer = self.root.after(50,self.advance_start_sequence)

    def set_rpm(self):
        try:
            n=int(self.rpm.get())
            if n != 0 and not 100 <= n <= 12000: raise ValueError()
        except ValueError:
            messagebox.showerror("RPM", "Use 0 or 100–12000 RPM.")
            return
        self.cancel_start_sequence()
        self.start_status.set(f"Manual RPM: {n}")
        self.send(f"RPM {n}")

    def set_wheel(self):
        try:
            teeth=int(self.wheel_teeth.get())
            missing=int(self.wheel_missing.get())
            if not 12 <= teeth <= 120 or not 1 <= missing <= 16 or missing >= teeth:
                raise ValueError()
        except ValueError:
            messagebox.showerror("Crank wheel", "Total teeth: 12–120; missing teeth: 1–16 and fewer than total.")
            return
        self.send(f"WHEEL {teeth} {missing} {int(self.wheel_invert.get())}")

    def set_cam(self,name):
        start,width,invert=self.cam_vars[name]
        try:
            a=int(start.get())
            w=int(width.get())
            if not (0<=w<=720): raise ValueError()
        except ValueError:
            messagebox.showerror("Cam shape", "Start must be an integer; width must be 0..720 degrees.")
            return
        self.send(f"CAM {name} {a} {w} {int(invert.get())}")

    def set_tdc(self):
        try:
            offset=int(self.tdc.get())
            if not -720 <= offset <= 720: raise ValueError()
        except ValueError:
            messagebox.showerror("TDC angle", "Use an integer from -720 to 720 degrees.")
            return
        self.send(f"TDC {offset}")

    def set_stream(self): self.send(f"STREAM {int(self.stream.get())}")

    def set_polarity(self):
        """Combine two logical six-bit masks; physical GPIO ordering is handled in firmware."""
        inj = self.inj_active_low.get().strip()
        coil = self.coil_active_low.get().strip()
        if (len(inj) != 6 or len(coil) != 6
                or any(bit not in "01" for bit in inj + coil)):
            messagebox.showerror("Polarity", "Enter exactly six binary digits (0 or 1) for each group.\n"
                                 "Leftmost bit = cylinder 6; rightmost bit = cylinder 1.")
            return
        mask = (int(coil, 2) << 6) | int(inj, 2)
        self.send(f"POL {mask}")

    def log_line(self,line):
        self.log.config(state="normal")
        self.log.insert("end",line+"\n")
        self.counter+=1
        if self.counter>1500:
            self.log.delete("1.0","501.0")
            self.counter-=500
        self.log.see("end")
        self.log.config(state="disabled")

    def parse(self,line):
        fields=line.split(",")
        if fields[0]=="STAT" and len(fields)==41:
            _,rpm,sample,drops,event_drops,*measurements=fields
            self.drop=int(drops)
            self.event_drop=int(event_drops)
            for ch in range(12):
                idx=3*ch
                on,off,width=map(int,measurements[idx:idx+3])
                self.counts[ch]=[on,off,width]
                state="ON" if self.active["I" if ch<6 else "S"][ch%6] else "OFF"
                self.table.item(str(ch),values=(f"{'Injector' if ch<6 else 'Spark'} {ch%6+1}",on,off,width,self.last_angle[ch],self.last_relative[ch],state))
            self.stats.config(text=f"RPM: {rpm} | Samples processed: {sample} | Sample drops: {drops} | Event queue drops: {event_drops}")
        elif fields[0]=="ANG" and len(fields)==10:
            _,timestamp,kind,cylinder,on,width,angle,relative,valid,epoch=fields
            idx=int(cylinder)-1
            self.active[kind][idx]=(on=="ON")
            chan=idx+(0 if kind=="I" else 6)
            if valid=="1" and 0 <= int(angle) < 720:
                self.add_raster_edge(chan,on=="ON",int(angle),int(relative),int(epoch))
            if on=="ON":
                self.last_angle[chan]=angle+"°" if valid=="1" else "—"
                self.last_relative[chan]=(relative+"°") if valid=="1" else "—"
            vals=list(self.table.item(str(chan),"values"))
            vals[4]=self.last_angle[chan]
            vals[5]=self.last_relative[chan]
            vals[6]=on
            self.table.item(str(chan),values=vals)
            typ="Injector" if kind=="I" else "Spark"
            phase=(f"phase={angle:>3}° rel={relative:>4}°" if valid=="1" else "phase=UNSYNCED")
            self.log_line(f"{int(timestamp)/1000000:10.6f} s | {typ:8} {cylinder} | {on:3} | {phase} | width={width} µs | cycle={epoch}")
        elif fields[0]=="WHEEL" and len(fields)==4:
            _,teeth,missing,inverted=fields
            self.wheel_teeth.set(teeth)
            self.wheel_missing.set(missing)
            self.wheel_invert.set(bool(int(inverted)))
            self.log_line(f"Crank wheel: {teeth}-{missing}, inverted={inverted}")
        elif fields[0]=="TDC" and len(fields)==3:
            self.tdc.set(fields[1])
        elif fields[0]=="EV" and len(fields)==6:
            _,timestamp,kind,cylinder,on,width=fields
            idx=int(cylinder)-1
            self.active[kind][idx]=(on=="ON")
            chan=idx+(0 if kind=="I" else 6)
            vals=list(self.table.item(str(chan),"values"))
            vals[-1]=on
            self.table.item(str(chan),values=vals)
            typ="Injector" if kind=="I" else "Spark   "
            self.log_line(f"{int(timestamp)/1000000:10.6f} s | {typ} {cylinder} | {on:3} | width={width} µs")
        elif fields[0]=="CAMS" and len(fields)==9:
            _,profile,a,w,i,b,x,j,cycles=fields
            self.log_line(f"Profile: {profile}, 720-degree cycles queued: {cycles}")
            for name, values in (("G40",(a,w,i)),("G163",(b,x,j))):
                start,width,invert=self.cam_vars[name]
                start.set(values[0]); width.set(values[1]); invert.set(bool(int(values[2])))
        else:
            if line.startswith("OK") or line.startswith("ERR") or line.startswith("BOOT"):
                self.log_line(line)

    def poll(self):
        for _ in range(250):
            try: kind,value=self.inbox.get_nowait()
            except queue.Empty: break
            if kind=="connected":
                self.status.set(f"Connected: {value}")
                self.send(f"STREAM {int(self.stream.get())}")
                self.set_polarity()
                self.send("TDC?")
                self.send("WHEEL?")
                self.send("CAM?")
            elif kind=="line":
                try:self.parse(value)
                except (ValueError, IndexError):self.log_line("BAD DATA: "+value)
            elif kind=="error":
                self.cancel_start_sequence()
                self.log_line("SERIAL ERROR: "+value)
                self.status.set("Serial error")
            elif kind=="disconnected":
                self.cancel_start_sequence()
                self.connection=None
                self.connect_button.config(text="Connect")
                self.status.set("Disconnected")
        if self.raster_dirty and not self.raster_paused.get():
            self.redraw_raster()
        self.root.after(25,self.poll)

    def close(self):
        self.cancel_start_sequence()
        if self.connection: self.connection.close()
        self.root.destroy()

if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port",help="e.g. COM5 or /dev/ttyACM0")
    args=parser.parse_args()
    root=tk.Tk()
    App(root,args.port)
    root.mainloop()
