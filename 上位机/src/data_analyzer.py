import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import threading
import time
import re
import math
import serial
import serial.tools.list_ports

class RealtimePlotter(tk.Canvas):
    """
    专用实时同步差值示波器组件。
    采用固定大小的滑动窗口实时滚动，中心 Y=0 (目标值) 绘制在最正中间，差值曲线使用荧光绿绘制。
    """
    def __init__(self, parent, **kwargs):
        super().__init__(parent, **kwargs)
        self.bg_color = "#121214"
        self.grid_color = "#23232A"
        self.target_color = "#3B82F6"   # 科技蓝 (0刻度目标虚线)
        self.actual_color = "#10B981"   # 荧光绿 (差值实线)
        self.text_color = "#8E9297"
        
        self.configure(bg=self.bg_color, highlightthickness=0)
        self.bind("<Configure>", self.on_resize)
        
        # 数据缓存 [(t_ms, data_dict), ...]
        self.data_buffer = []
        self.paused = False
        
        # 固定滑动窗口大小：5000 毫秒 (5 秒)
        self.window_size_ms = 5000 
        
        # 降频绘制，限制刷新率防 CPU 拖死主线程渲染
        self.last_draw_time = 0.0
        
    def on_resize(self, event):
        self.redraw()
        
    def clear(self):
        self.data_buffer.clear()
        self.redraw()
        
    def add_point(self, t_ms, data_dict):
        if self.paused:
            return
        self.data_buffer.append((t_ms, data_dict))
        
        # 丢弃超出当前滑动窗口左边界的过期数据，保持内存轻量
        if len(self.data_buffer) > 1:
            t_latest = self.data_buffer[-1][0]
            cutoff = t_latest - self.window_size_ms - 1000 # 留1秒裕量防边界截断
            self.data_buffer = [d for d in self.data_buffer if d[0] >= cutoff]
            
        # 限制绘图最高刷新率为 25 帧 (40ms 重绘一次)，防高频下 delete("all") 导致 Canvas 白屏
        now = time.time()
        if now - self.last_draw_time >= 0.04:
            self.last_draw_time = now
            self.redraw()
        
    def redraw(self):
        self.delete("all")
        
        width = self.winfo_width()
        height = self.winfo_height()
        
        if width < 50 or height < 50:
            return
            
        # 留白边距
        pad_left = 60
        pad_right = 20
        pad_top = 30
        pad_bottom = 40
        
        plot_width = width - pad_left - pad_right
        plot_height = height - pad_top - pad_bottom
        
        # 绘制背景与图表边框
        self.create_rectangle(pad_left, pad_top, pad_left + plot_width, pad_top + plot_height, 
                               fill=self.bg_color, outline="#2A2A35", width=1.5)
        
        if not self.data_buffer:
            self.create_text(pad_left + plot_width/2, pad_top + plot_height/2, 
                             text="等待同步数据流输入...", fill="#5E6268", font=("Segoe UI", 10))
            return
            
        # ==================== 1. 计算滑动窗口 X 轴范围 ====================
        t_latest = self.data_buffer[-1][0]
        x_max = t_latest
        x_min = t_latest - self.window_size_ms
        
        # ==================== 2. 计算各点的差值并获取 Y 轴范围 ====================
        plot_points = [] # [(x_val, y_val), ...]
        
        for t_ms, d in self.data_buffer:
            if t_ms < x_min:
                continue
            # 提取 H0~H3 计算差值
            h_vals = [d[k] for k in ['H0', 'H1', 'H2', 'H3'] if k in d]
            if len(h_vals) >= 2:
                diff_val = max(h_vals) - min(h_vals)
            else:
                diff_val = 0.0
            plot_points.append((t_ms, diff_val))
            
        # 提取窗口内出现的偏差值，求自适应对称范围
        errs = [pt[1] for pt in plot_points]
        max_err = max(errs) if errs else 0.0
        
        # 对称 Y 轴限幅：中心为 0，范围是 [-V_limit, V_limit]
        y_limit = max(max_err, 10.0) # 至少展示 [-10, 10]
        y_max = y_limit
        y_min = -y_limit
        y_range = y_max - y_min
        
        # 转换坐标映射函数
        def to_plot_coords(x_val, y_val):
            x_px = pad_left + ((x_val - x_min) / self.window_size_ms) * plot_width
            y_px = pad_top + plot_height - ((y_val - y_min) / y_range) * plot_height
            return x_px, y_px
            
        # ==================== 3. 绘制垂直网格线 (以秒为单位滚动) ====================
        sec_start = math.ceil(x_min / 1000.0)
        sec_end = math.floor(x_max / 1000.0)
        for s in range(sec_start, sec_end + 1):
            px, py = to_plot_coords(s * 1000.0, y_min)
            self.create_line(px, pad_top, px, pad_top + plot_height, fill=self.grid_color, dash=(2, 2))
            self.create_text(px, pad_top + plot_height + 15, text=f"{s:.1f}s", fill=self.text_color, anchor="n", font=("Consolas", 8))
            
        # ==================== 4. 绘制 Y 轴对称刻度与水平网格 ====================
        # 绘制中心 0 轴网格线 (目标虚线放正中间，蓝色)
        cx, cy = to_plot_coords(x_min, 0.0)
        self.create_line(pad_left, cy, pad_left + plot_width, cy, fill=self.target_color, width=1.5, dash=(4, 3))
        self.create_text(pad_left - 10, cy, text="0 (目标)", fill=self.target_color, anchor="e", font=("Consolas", 8, "bold"))
        
        # 绘制上方最大值网格
        ux, uy = to_plot_coords(x_min, y_max * 0.8)
        self.create_line(pad_left, uy, pad_left + plot_width, uy, fill=self.grid_color, dash=(2, 2))
        self.create_text(pad_left - 10, uy, text=f"+{y_max * 0.8:.1f}", fill=self.text_color, anchor="e", font=("Consolas", 8))
        
        # 绘制下方对称网格
        lx, ly = to_plot_coords(x_min, -y_max * 0.8)
        self.create_line(pad_left, ly, pad_left + plot_width, ly, fill=self.grid_color, dash=(2, 2))
        self.create_text(pad_left - 10, ly, text=f"-{y_max * 0.8:.1f}", fill=self.text_color, anchor="e", font=("Consolas", 8))
        
        # ==================== 5. 绘制当前最大差值曲线 (荧光绿实线) ====================
        if plot_points:
            line_coords = []
            for x_val, y_val in plot_points:
                line_coords.extend(to_plot_coords(x_val, y_val))
                
            if len(line_coords) >= 4:
                self.create_line(line_coords, fill=self.actual_color, width=2.5)
            
        # 6. 图例
        self.create_line(pad_left + 15, pad_top - 15, pad_left + 35, pad_top - 15, fill=self.target_color, width=1.5, dash=(4, 3))
        self.create_text(pad_left + 40, pad_top - 15, text="理想同步状态 (偏差 = 0)", fill=self.text_color, anchor="w", font=("Segoe UI", 8))
        
        self.create_line(pad_left + 180, pad_top - 15, pad_left + 200, pad_top - 15, fill=self.actual_color, width=2.5)
        self.create_text(pad_left + 205, pad_top - 15, text="当前四轴最大偏差", fill=self.text_color, anchor="w", font=("Segoe UI", 8))


class ModernPIDAnalyzerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("2026-273 升降同步差值监控上位机")
        self.root.geometry("1100x650")
        self.root.configure(bg="#121214")
        
        # 通信及监控变量
        self.ser = None
        self.running = False
        self.rx_thread = None
        self.start_time = None
        
        # 分析指标
        self.peak_sync_error = 0.0 # 历史最大同步差值
        
        self.setup_styles()
        self.create_widgets()
        self.refresh_ports()
        
    def setup_styles(self):
        self.style = ttk.Style()
        self.style.theme_use('clam')
        
        self.style.configure('TFrame', background='#121214')
        self.style.configure('Card.TFrame', background='#1E1E24', relief='flat')
        self.style.configure('TPanedwindow', background='#121214')
        
        self.style.configure('TLabel', background='#121214', foreground='#EEEEEE', font=("Segoe UI", 9))
        self.style.configure('Title.TLabel', background='#1E1E24', foreground='#00ADB5', font=("Segoe UI", 10, "bold"))
        self.style.configure('DarkTitle.TLabel', background='#121214', foreground='#00ADB5', font=("Segoe UI", 10, "bold"))
        
        self.style.configure('TButton', background='#3A3F47', foreground='#EEEEEE', borderwidth=0, font=("Segoe UI", 9))
        self.style.map('TButton', background=[('active', '#4E5460')])
        
        self.style.configure('Action.TButton', background='#00ADB5', foreground='#FFFFFF', borderwidth=0, font=("Segoe UI", 9, "bold"))
        self.style.map('Action.TButton', background=[('active', '#00D1D9')])
        
        self.style.configure('Reset.TButton', background='#EF4444', foreground='#FFFFFF', borderwidth=0, font=("Segoe UI", 9, "bold"))
        self.style.map('Reset.TButton', background=[('active', '#F87171')])
        
        self.style.configure('TCombobox', fieldbackground='#3A3F47', background='#3A3F47', foreground='#EEEEEE')
        self.style.configure('TEntry', fieldbackground='#3A3F47', foreground='#EEEEEE')

    def create_widgets(self):
        # ==================== 顶部标题栏 ====================
        top_bar = ttk.Frame(self.root, style='Card.TFrame')
        top_bar.pack(fill="x", padx=10, pady=5)
        
        title_lbl = ttk.Label(top_bar, text="★ 2026-273 MULTI-AXIS SYNC MONITOR ★", style='Title.TLabel', font=("Segoe UI", 11, "bold"))
        title_lbl.pack(side="left", padx=15, pady=10)
        
        self.conn_status_lbl = ttk.Label(top_bar, text="未连接", background="#1E1E24", foreground="#EF4444", font=("Segoe UI", 10, "bold"))
        self.conn_status_lbl.pack(side="right", padx=20, pady=10)
        
        # ==================== 主体双栏容器 ====================
        main_container = ttk.Frame(self.root, style='TFrame')
        main_container.pack(fill="both", expand=True, padx=10, pady=5)
        
        # -------------------- 左侧配置和分析面板 --------------------
        left_panel = ttk.Frame(main_container, style='TFrame', width=300)
        left_panel.pack(side="left", fill="both", padx=(0, 5), pady=5)
        left_panel.pack_propagate(False)
        
        # 1. 连接卡片
        conn_card = ttk.Frame(left_panel, style='Card.TFrame')
        conn_card.pack(fill="x", pady=(0, 4), ipady=5)
        
        ttk.Label(conn_card, text="【接口配置】", style='Title.TLabel').pack(anchor="w", padx=15, pady=6)
        
        port_frame = ttk.Frame(conn_card, style='TFrame')
        port_frame.pack(fill="x", padx=15, pady=3)
        ttk.Label(port_frame, text="串口选择:", width=8).pack(side="left")
        self.port_cb = ttk.Combobox(port_frame, width=12, state="readonly")
        self.port_cb.pack(side="left", fill="x", expand=True)
        
        baud_frame = ttk.Frame(conn_card, style='TFrame')
        baud_frame.pack(fill="x", padx=15, pady=3)
        ttk.Label(baud_frame, text="波特率值:", width=8).pack(side="left")
        self.baud_cb = ttk.Combobox(baud_frame, values=["9600", "115200", "460800", "921600"], width=12, state="readonly")
        self.baud_cb.set("115200")
        self.baud_cb.pack(side="left", fill="x", expand=True)
        
        btn_conn_frame = ttk.Frame(conn_card, style='TFrame')
        btn_conn_frame.pack(fill="x", padx=15, pady=8)
        ttk.Button(btn_conn_frame, text="刷新串口", command=self.refresh_ports).pack(side="left", fill="x", expand=True, padx=(0, 4))
        self.btn_conn = ttk.Button(btn_conn_frame, text="连接设备", style='Action.TButton', command=self.toggle_connection)
        self.btn_conn.pack(side="right", fill="x", expand=True, padx=(4, 0))
        
        # 2. 同步性能分析指标卡片
        ana_card = ttk.Frame(left_panel, style='Card.TFrame')
        ana_card.pack(fill="both", expand=True, pady=(0, 0))
        
        ttk.Label(ana_card, text="【同步差值监视】", style='Title.TLabel').pack(anchor="w", padx=15, pady=6)
        
        metrics_frame = ttk.Frame(ana_card, style='TFrame')
        metrics_frame.pack(fill="both", expand=True, padx=15, pady=2)
        
        self.metric_target = self.create_metric_row(metrics_frame, "目标差值:", "0", "#3B82F6")
        self.metric_current_diff = self.create_metric_row(metrics_frame, "当前差值 (Current):", "0", "#10B981")
        self.metric_max_sync_diff = self.create_metric_row(metrics_frame, "最大差值 (Peak):", "0", "#EF4444")
        self.metric_status = self.create_metric_row(metrics_frame, "运行状态:", "静止", "#EEEEEE")
        
        # 清空数据日志窗口按钮
        self.btn_clear_log = ttk.Button(ana_card, text="清空数据日志窗口", command=self.clear_log)
        self.btn_clear_log.pack(fill="x", padx=15, pady=6)
        
        # 简易控制区
        btn_ana_frame = ttk.Frame(ana_card, style='TFrame')
        btn_ana_frame.pack(fill="x", padx=15, pady=(3, 10))
        ttk.Button(btn_ana_frame, text="清空曲线", command=self.clear_chart).pack(side="left", fill="x", expand=True, padx=(0, 4))
        ttk.Button(btn_ana_frame, text="重置 Peak", style='Reset.TButton', command=self.reset_metrics).pack(side="right", fill="x", expand=True, padx=(4, 0))
        
        # -------------------- 右侧波形与日志面板 (PanedWindow 可上下拉伸) --------------------
        right_paned = ttk.PanedWindow(main_container, orient=tk.VERTICAL)
        right_paned.pack(side="right", fill="both", expand=True, padx=(5, 0), pady=5)
        
        # 1. 实时曲线卡片 (滑动示波器)
        chart_card = ttk.Frame(right_paned, style='Card.TFrame')
        
        chart_bar = ttk.Frame(chart_card, style='TFrame')
        chart_bar.pack(fill="x", padx=15, pady=6)
        ttk.Label(chart_bar, text="【同步差值示波器 (5秒滑动窗口)】", style='Title.TLabel').pack(side="left")
        
        self.btn_pause = ttk.Button(chart_bar, text="暂停刷新", width=10, command=self.toggle_pause)
        self.btn_pause.pack(side="right")
        
        self.plotter = RealtimePlotter(chart_card, width=800, height=350)
        self.plotter.pack(fill="both", expand=True, padx=15, pady=(0, 10))
        
        # 2. 日志流卡片
        log_card = ttk.Frame(right_paned, style='Card.TFrame')
        
        ttk.Label(log_card, text="【数据流与系统日志】 (可拖动分割线调整高度)", style='Title.TLabel').pack(anchor="w", padx=15, pady=6)
        
        self.log_txt = scrolledtext.ScrolledText(log_card, bg="#121214", fg="#EEEEEE", 
                                                 insertbackground="white", font=("Courier New", 9),
                                                 highlightthickness=0, borderwidth=0)
        self.log_txt.pack(fill="both", expand=True, padx=15, pady=(0, 8))
        
        self.log_txt.tag_config("sys", foreground="#00ADB5")
        self.log_txt.tag_config("info", foreground="#10B981")
        self.log_txt.tag_config("warn", foreground="#FFC312")
        self.log_txt.tag_config("err", foreground="#EF4444")
        
        # 加入 PanedWindow，设置初始权重 proportion
        right_paned.add(chart_card, weight=3)
        right_paned.add(log_card, weight=2)
        
    def create_metric_row(self, parent, label_text, val_text, val_color):
        row = ttk.Frame(parent, style='TFrame')
        row.pack(fill="x", pady=5)
        ttk.Label(row, text=label_text, foreground="#8E9297", font=("Segoe UI", 9)).pack(side="left")
        lbl_val = ttk.Label(row, text=val_text, foreground=val_color, font=("Consolas", 11, "bold"))
        lbl_val.pack(side="right")
        return lbl_val

    def refresh_ports(self):
        ports = list(serial.tools.list_ports.comports())
        port_list = [p.device for p in ports]
        
        self.port_cb['values'] = port_list
        if port_list:
            self.port_cb.set(port_list[0])
        else:
            self.port_cb.set("")
            
        self.log_message("[系统] 已刷新可用物理串口列表。", "sys")

    def toggle_connection(self):
        if not self.running:
            port = self.port_cb.get()
            if not port:
                messagebox.showerror("连接错误", "未检测到可用串口！")
                return
                
            self.start_time = None # 每次连接重置时间起点
            try:
                self.ser = serial.Serial(
                    port=port,
                    baudrate=int(self.baud_cb.get()),
                    bytesize=serial.EIGHTBITS,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.1
                )
                self.running = True
                self.conn_status_lbl.configure(text=f"已连接: {port}", foreground="#10B981")
                self.btn_conn.configure(text="断开设备", style='TButton')
                self.log_message(f"[系统] 成功打开串口 {port}，波特率 {self.baud_cb.get()} bps。", "info")
                
                self.peak_sync_error = 0.0
                self.plotter.clear()
                
                self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
                self.rx_thread.start()
            except Exception as e:
                self.log_message(f"[连接异常]: {str(e)}", "err")
                messagebox.showerror("物理连接错误", f"无法打开串口 {port}:\n{str(e)}")
        else:
            self.stop_connection()

    def stop_connection(self):
        self.running = False
        self.start_time = None
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None
        self.conn_status_lbl.configure(text="未连接", foreground="#EF4444")
        self.btn_conn.configure(text="连接设备", style='Action.TButton')
        self.log_message("[系统] 已断开物理串口连接。", "sys")
        self.plotter.redraw() # 渲染离线状态

    def toggle_pause(self):
        self.plotter.paused = not self.plotter.paused
        if self.plotter.paused:
            self.btn_pause.configure(text="继续刷新", style='Action.TButton')
        else:
            self.btn_pause.configure(text="暂停刷新", style='TButton')

    def clear_chart(self):
        self.plotter.clear()
        self.log_message("[系统] 滑动示波器历史缓存已清空。", "sys")

    def clear_log(self):
        """清空数据日志窗口的文本内容"""
        self.log_txt.delete('1.0', tk.END)

    def reset_metrics(self):
        self.peak_sync_error = 0.0
        self.metric_max_sync_diff.configure(text="0")
        self.log_message("[系统] 最大差值 Peak 峰值监控已重置。", "sys")

    def process_new_data(self, t_ms, data_dict):
        """
        处理每一帧新数据，计算并呈现轴间最大偏差。
        """
        # 1. 提取四轴霍尔计算最大差值
        h_vals = [data_dict[k] for k in ['H0', 'H1', 'H2', 'H3'] if k in data_dict]
        if len(h_vals) >= 2:
            curr_diff = max(h_vals) - min(h_vals)
        else:
            curr_diff = 0.0
            
        # 更新实时差值
        self.metric_current_diff.configure(text=f"{int(curr_diff)}")
        
        # 2. 更新历史最大峰值指标 Peak
        if curr_diff > self.peak_sync_error:
            self.peak_sync_error = curr_diff
            self.metric_max_sync_diff.configure(text=f"{int(self.peak_sync_error)}")
            
        # 3. 简单判定系统状态
        if curr_diff > 15:
            self.metric_status.configure(text="失步风险高", foreground="#EF4444")
        elif curr_diff > 5:
            self.metric_status.configure(text="动态微调中", foreground="#FFC312")
        else:
            self.metric_status.configure(text="稳定运行", foreground="#10B981")
            
        # 4. 把点提交给滚动示波器 Canvas 绘制
        self.plotter.add_point(t_ms, data_dict)

    def rx_loop(self):
        """物理串口后台多线程安全接收"""
        line_buffer = ""
        while self.running:
            if self.ser and self.ser.is_open:
                try:
                    waiting = self.ser.in_waiting
                    if waiting > 0:
                        rx_bytes = self.ser.read(waiting)
                        try:
                            decoded = rx_bytes.decode('utf-8', errors='ignore')
                        except Exception:
                            decoded = ""
                            
                        line_buffer += decoded
                        
                        while "\n" in line_buffer:
                            line, line_buffer = line_buffer.split("\n", 1)
                            line = line.strip()
                            if not line:
                                continue
                                
                            self.log_message(f"Rx: {line}", "info")
                            
                            data_dict = self.parse_line_data(line)
                            if data_dict:
                                if self.start_time is None:
                                    self.start_time = time.time()
                                    
                                curr_t_ms = int((time.time() - self.start_time) * 1000)
                                self.root.after(0, self.process_new_data, curr_t_ms, data_dict)
                except Exception as e:
                    if self.running:
                        self.log_message(f"[接收异常]: {str(e)}", "err")
                        self.root.after(0, self.stop_connection)
                    break
            time.sleep(0.005)

    def parse_line_data(self, line: str):
        data_dict = {}
        pairs = re.findall(r'([a-zA-Z_0-9]+)\s*[:=]\s*([-+]?\d*\.\d+|\d+)', line)
        if pairs:
            for k, v in pairs:
                try:
                    data_dict[k] = float(v)
                except ValueError:
                    pass
            return data_dict
            
        parts = re.split(r'[\s,;]+', line)
        nums = []
        for p in parts:
            p = p.strip()
            if p:
                try:
                    nums.append(float(p))
                except ValueError:
                    pass
        if nums:
            for i, n in enumerate(nums):
                data_dict[f"Val_{i}"] = n
            return data_dict
            
        return None

    def log_message(self, text: str, level: str = "sys"):
        time_str = time.strftime("[%H:%M:%S] ")
        full_line = f"{time_str}{text}\n"
        
        def append():
            self.log_txt.insert(tk.END, full_line, level)
            if float(self.log_txt.index('end-1c')) > 200.0:
                self.log_txt.delete('1.0', '5.0')
            self.log_txt.see(tk.END)
        self.root.after(0, append)

def main():
    root = tk.Tk()
    app = ModernPIDAnalyzerApp(root)
    
    def on_closing():
        app.stop_connection()
        root.destroy()
        
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()
