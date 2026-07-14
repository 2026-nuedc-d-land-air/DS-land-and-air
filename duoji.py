# # -*- coding: utf-8 -*-
# import serial
# import time
# import threading
# import tkinter as tk
# from tkinter import scrolledtext

# SERVO_PORT = "COM7"
# BAUD_RATE = 115200

# ACTION_GROUPS = [
#     ("基础动作", [
#         ("夹板归位", "5,600,500 2,2130,500"),
#         ("中间关节向下", "4,870,1500 1,870,1500"),
#         ("中间关节抬升", "4,2200,1500 1,2200,1500"),
#         ("根关节合拢", "3,1375,300 0,1375,300"),
#         ("根关节张开", "3,1500,300 0,1500,300"),
#     ]),
#     ("组合动作", [
#         ("初始位", "3,1500,300 0,1500,300 | 5,600,500 2,2130,500 | 4,870,700 1,870,700"),
#         ("夹取抬升", "3,1370,500 0,1370,500 | 4,2200,3000 1,2200,3000"),
#         ("放下释放", "4,870,3000 1,870,3000 | 3,1500,200 0,1500,200"),
#         ("悬空释放", "4,1750,1500 1,1725,1500 | 3,1500,500 0,1500,500"),
#     ]),
#     ("回正操作", [
#         ("全回正", "3,1500,300 0,1500,300 | 4,1500,500 1,1500,500 | 5,1500,500 2,1500,500"),
#         ("夹板回正", "5,1500,500 2,1500,500"),
#         ("中间关节回正", "4,1500,1500 1,1500,1500"),
#         ("根关节回正", "3,1500,300 0,1500,300"),
#     ]),
# ]

# GROUP_COLORS = {
#     "基础动作": "#4A90D9",
#     "组合动作": "#E8913A",
#     "回正操作": "#5CB85C",
# }


# def send_commands_batch(ser, cmds):
#     for cmd in cmds:
#         ser.write(cmd.encode())
#     time.sleep(0.3)


# def parse_group(text):
#     pairs = []
#     for item in text.split():
#         parts = item.split(",")
#         if len(parts) not in (2, 3):
#             return None
#         try:
#             servo_id = int(parts[0])
#             position = int(parts[1])
#             move_time = int(parts[2]) if len(parts) == 3 else 2000
#         except ValueError:
#             return None
#         pairs.append((servo_id, position, move_time))
#     return pairs


# def parse_sequence(text):
#     groups = []
#     for seg in text.split("|"):
#         seg = seg.strip()
#         if not seg:
#             return None
#         pairs = parse_group(seg)
#         if pairs is None:
#             return None
#         groups.append(pairs)
#     return groups


# def set_servo_positions_batch(ser, pairs, log):
#     cmds = []
#     for servo_id, position, move_time in pairs:
#         cmd = f"#{servo_id:03d}P{position:04d}T{move_time:04d}!"
#         log(f"  发送: {cmd}")
#         cmds.append(cmd)
#     send_commands_batch(ser, cmds)


# def execute_sequence(ser, groups, log):
#     for i, pairs in enumerate(groups):
#         max_time = max(t for _, _, t in pairs)
#         log(f"  --- 第 {i + 1}/{len(groups)} 步 ---")
#         set_servo_positions_batch(ser, pairs, log)
#         if i < len(groups) - 1:
#             wait_sec = max_time / 1000.0
#             log(f"  等待 {wait_sec:.1f}s ...")
#             time.sleep(wait_sec)
#     log(f"  全部完成 ({len(groups)} 步)")


# class ServoApp:
#     def __init__(self, root):
#         self.root = root
#         self.root.title("舵机控制面板")
#         self.root.resizable(True, True)
#         self.root.configure(bg="#F5F5F5")
#         self.ser = None
#         self.running = False

#         self.build_ui()
#         self.connect()

#     def build_ui(self):
#         header = tk.Frame(self.root, bg="#333333", height=50)
#         header.pack(fill=tk.X)
#         header.pack_propagate(False)

#         tk.Label(header, text="⚙ 舵机控制面板", font=("Microsoft YaHei UI", 16, "bold"),
#                  bg="#333333", fg="white").pack(side=tk.LEFT, padx=15, pady=10)

#         self.status_dot = tk.Label(header, text="●", font=("Arial", 14), bg="#333333", fg="red")
#         self.status_dot.pack(side=tk.RIGHT, padx=(0, 5), pady=10)
#         self.status_label = tk.Label(header, text="未连接", font=("Microsoft YaHei UI", 10),
#                                      bg="#333333", fg="#CCCCCC")
#         self.status_label.pack(side=tk.RIGHT, padx=(0, 5), pady=10)

#         tk.Label(header, text=f"{SERVO_PORT} | {BAUD_RATE}", font=("Consolas", 10),
#                  bg="#333333", fg="#AAAAAA").pack(side=tk.RIGHT, padx=15, pady=10)

#         btn_container = tk.Frame(self.root, bg="#F5F5F5")
#         btn_container.pack(fill=tk.X, padx=15, pady=(10, 5))

#         for group_name, actions in ACTION_GROUPS:
#             color = GROUP_COLORS.get(group_name, "#666666")
#             self._build_group(btn_container, group_name, actions, color)

#         custom_frame = tk.LabelFrame(self.root, text=" 自定义指令 ", font=("Microsoft YaHei UI", 10, "bold"),
#                                      padx=12, pady=8, bg="#F5F5F5", fg="#555555")
#         custom_frame.pack(fill=tk.X, padx=15, pady=5)

#         entry_row = tk.Frame(custom_frame, bg="#F5F5F5")
#         entry_row.pack(fill=tk.X)

#         self.custom_entry = tk.Entry(entry_row, font=("Consolas", 12), relief=tk.SOLID, bd=1)
#         self.custom_entry.pack(fill=tk.X, side=tk.LEFT, expand=True, ipady=4)
#         self.custom_entry.bind("<Return>", lambda e: self.run_custom())

#         tk.Button(entry_row, text="▶ 执行", font=("Microsoft YaHei UI", 10, "bold"),
#                   bg="#4A90D9", fg="white", activebackground="#3A7BC8",
#                   relief=tk.FLAT, padx=15, cursor="hand2",
#                   command=self.run_custom).pack(side=tk.RIGHT, padx=(10, 0))

#         tk.Label(custom_frame, text="格式: ID,位置[,速度] 空格分隔多组 | 分步执行",
#                  fg="#999999", font=("Microsoft YaHei UI", 9), bg="#F5F5F5").pack(anchor=tk.W, pady=(5, 0))

#         log_frame = tk.LabelFrame(self.root, text=" 运行日志 ", font=("Microsoft YaHei UI", 10, "bold"),
#                                   padx=10, pady=8, bg="#F5F5F5", fg="#555555")
#         log_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=(5, 15))

#         self.log_text = scrolledtext.ScrolledText(
#             log_frame, height=8, font=("Consolas", 10),
#             bg="#2B2B2B", fg="#D4D4D4", insertbackground="white",
#             relief=tk.FLAT, state=tk.DISABLED
#         )
#         self.log_text.pack(fill=tk.BOTH, expand=True)

#     def _build_group(self, parent, group_name, actions, color):
#         frame = tk.LabelFrame(parent, text=f" {group_name} ", font=("Microsoft YaHei UI", 10, "bold"),
#                               padx=10, pady=8, bg="#F5F5F5", fg=color)
#         frame.pack(fill=tk.X, pady=4)

#         for i, (name, cmd_text) in enumerate(actions):
#             btn = tk.Button(
#                 frame, text=name, font=("Microsoft YaHei UI", 11),
#                 bg=color, fg="white", activebackground=self._darken(color),
#                 activeforeground="white", relief=tk.FLAT, padx=12, pady=6,
#                 cursor="hand2",
#                 command=lambda n=name, c=cmd_text: self.run_action(n, c)
#             )
#             btn.grid(row=0, column=i, padx=5, pady=2, sticky="nsew")

#         for c in range(len(actions)):
#             frame.columnconfigure(c, weight=1)

#     def _darken(self, hex_color):
#         r = max(0, int(hex_color[1:3], 16) - 30)
#         g = max(0, int(hex_color[3:5], 16) - 30)
#         b = max(0, int(hex_color[5:7], 16) - 30)
#         return f"#{r:02x}{g:02x}{b:02x}"

#     def log(self, msg):
#         def _update():
#             self.log_text.config(state=tk.NORMAL)
#             self.log_text.insert(tk.END, msg + "\n")
#             self.log_text.see(tk.END)
#             self.log_text.config(state=tk.DISABLED)
#         self.root.after(0, _update)

#     def connect(self):
#         try:
#             self.ser = serial.Serial(SERVO_PORT, BAUD_RATE, timeout=1)
#             self.status_dot.config(fg="#5CB85C")
#             self.status_label.config(text="已连接")
#             self.log(f"[连接成功] {SERVO_PORT} @ {BAUD_RATE}")
#         except Exception as e:
#             self.status_dot.config(fg="red")
#             self.status_label.config(text="连接失败")
#             self.log(f"[连接失败] {e}")

#     def run_action(self, name, cmd_text):
#         if self.ser is None or not self.ser.is_open:
#             self.log("[错误] 串口未连接!")
#             return
#         if self.running:
#             self.log("[提示] 有动作正在执行，请等待完成")
#             return

#         def _run():
#             self.running = True
#             self.log(f"\n{'─' * 30}")
#             self.log(f"▶ {name}")
#             self.log(f"  指令: {cmd_text}")
#             try:
#                 groups = parse_sequence(cmd_text)
#                 if groups is None:
#                     self.log("[错误] 指令解析失败")
#                     return
#                 if len(groups) == 1:
#                     set_servo_positions_batch(self.ser, groups[0], self.log)
#                     self.log(f"  已设置 {len(groups[0])} 个舵机")
#                 else:
#                     execute_sequence(self.ser, groups, self.log)
#             except Exception as e:
#                 self.log(f"[错误] {e}")
#             finally:
#                 self.running = False
#                 self.log("✓ 完成")

#         threading.Thread(target=_run, daemon=True).start()

#     def run_custom(self):
#         cmd_text = self.custom_entry.get().strip()
#         if not cmd_text:
#             return
#         self.run_action("自定义", cmd_text)

#     def on_close(self):
#         if self.ser and self.ser.is_open:
#             self.ser.close()
#             self.log("[断开] 串口已关闭")
#         self.root.destroy()


# if __name__ == "__main__":
#     root = tk.Tk()
#     root.geometry("620x600")
#     app = ServoApp(root)
#     root.protocol("WM_DELETE_WINDOW", app.on_close)
#     root.mainloop()



# -*- coding: utf-8 -*-
import serial
import time
import threading
import tkinter as tk
from tkinter import scrolledtext

SERVO_PORT = "COM7"
BAUD_RATE = 115200

# ===================== 表格动作常量定义 =====================
# 初始位置 S0=0,S1=1,S2=2,S3=3,S4=4,S5=5
INITIAL_POS_CMD = "0,1700,500 1,1800,500 2,986,500 3,1600,500 4,1178,500 5,1400,500"
# 动作1 夹取放下系列
ACTION_CATCH    = "0,1550,500 1,1450,500 2,1020,500 3,1450,500 4,846,500  5,1485,500"  # 抓取
ACTION_RELEASE  = "0,1650,500 1,1450,500 2,1020,500 3,1550,500 4,846,500  5,1485,500"  # 放开
ACTION_LIFT     = "0,1520,500 1,1720,500 2,1020,500 3,1400,500 4,1100,500 5,1485,500"  # 夹起来
ACTION_DOWN     = "0,1520,500 1,1700,500 2,1020,500 3,1400,500 4,1100,500 5,1485,500"  # 降一点
ACTION_STACK    = "0,1560,500 1,1700,500 2,1020,500 3,1450,500 4,1100,500 5,1485,500"  # 叠加

def send_commands_batch(ser, cmds):
    for cmd in cmds:
        ser.write(cmd.encode())
    time.sleep(0.3)

def parse_group(text):
    pairs = []
    for item in text.split():
        parts = item.split(",")
        if len(parts) not in (2, 3):
            return None
        try:
            servo_id = int(parts[0])
            position = int(parts[1])
            move_time = int(parts[2]) if len(parts) == 3 else 2000
        except ValueError:
            return None
        pairs.append((servo_id, position, move_time))
    return pairs

def set_servo_positions_batch(ser, pairs, log):
    cmds = []
    for servo_id, position, move_time in pairs:
        cmd = f"#{servo_id:03d}P{position:04d}T{move_time:04d}!"
        log(f"  发送: {cmd}")
        cmds.append(cmd)
    send_commands_batch(ser, cmds)

class ServoApp:
    def __init__(self, root):
        self.root = root
        self.root.title("六舵机夹取控制程序")
        self.root.resizable(True, True)
        self.root.configure(bg="#F5F5F5")
        self.ser = None
        self.running = False

        self.build_ui()
        self.connect()

    def build_ui(self):
        # 头部状态栏
        header = tk.Frame(self.root, bg="#333333", height=50)
        header.pack(fill=tk.X)
        header.pack_propagate(False)

        tk.Label(header, text="⚙ 舵机夹取动作控制", font=("Microsoft YaHei UI", 16, "bold"),
                 bg="#333333", fg="white").pack(side=tk.LEFT, padx=15, pady=10)

        self.status_dot = tk.Label(header, text="●", font=("Arial", 14), bg="#333333", fg="red")
        self.status_dot.pack(side=tk.RIGHT, padx=(0, 5), pady=10)
        self.status_label = tk.Label(header, text="未连接", font=("Microsoft YaHei UI", 10),
                                     bg="#333333", fg="#CCCCCC")
        self.status_label.pack(side=tk.RIGHT, padx=(0, 5), pady=10)

        tk.Label(header, text=f"{SERVO_PORT} | {BAUD_RATE}", font=("Consolas", 10),
                 bg="#333333", fg="#AAAAAA").pack(side=tk.RIGHT, padx=15, pady=10)

        # 动作按钮区域
        btn_frame = tk.LabelFrame(self.root, text=" 标准动作库 ", font=("Microsoft YaHei UI", 10, "bold"),
                                  padx=12, pady=12, bg="#F5F5F5", fg="#555555")
        btn_frame.pack(fill=tk.X, padx=15, pady=10)

        # 第一行按钮
        row1 = tk.Frame(btn_frame, bg="#F5F5F5")
        row1.pack(fill=tk.X, pady=(0,8))
        tk.Button(row1, text="▶ 回到初始位置", font=("Microsoft YaHei UI", 11, "bold"),
                  bg="#5CB85C", fg="white", activebackground="#4CAE4C", relief=tk.RAISED,
                  padx=10, pady=10, cursor="hand2",
                  command=lambda: self.run_action("回到初始位置", INITIAL_POS_CMD)).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=4)
        tk.Button(row1, text="抓取", font=("Microsoft YaHei UI", 11, "bold"),
                  bg="#4A90D9", fg="white", activebackground="#3A7BC8", relief=tk.RAISED,
                  padx=10, pady=10, cursor="hand2",
                  command=lambda: self.run_action("抓取", ACTION_CATCH)).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=4)
        tk.Button(row1, text="放开", font=("Microsoft YaHei UI", 11, "bold"),
                  bg="#f0ad4e", fg="white", activebackground="#ec971f", relief=tk.RAISED,
                  padx=10, pady=10, cursor="hand2",
                  command=lambda: self.run_action("放开", ACTION_RELEASE)).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=4)

        # 第二行按钮
        row2 = tk.Frame(btn_frame, bg="#F5F5F5")
        row2.pack(fill=tk.X)
        tk.Button(row2, text="夹起来", font=("Microsoft YaHei UI", 11, "bold"),
                  bg="#9632b8", fg="white", activebackground="#80299e", relief=tk.RAISED,
                  padx=10, pady=10, cursor="hand2",
                  command=lambda: self.run_action("夹起来", ACTION_LIFT)).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=4)
        tk.Button(row2, text="降一点", font=("Microsoft YaHei UI", 11, "bold"),
                  bg="#d9534f", fg="white", activebackground="#c9302c", relief=tk.RAISED,
                  padx=10, pady=10, cursor="hand2",
                  command=lambda: self.run_action("降一点", ACTION_DOWN)).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=4)
        tk.Button(row2, text="叠加", font=("Microsoft YaHei UI", 11, "bold"),
                  bg="#27ae60", fg="white", activebackground="#219653", relief=tk.RAISED,
                  padx=10, pady=10, cursor="hand2",
                  command=lambda: self.run_action("叠加", ACTION_STACK)).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=4)

        # 自定义指令输入区域
        custom_frame = tk.LabelFrame(self.root, text=" 自定义指令（可选） ", font=("Microsoft YaHei UI", 10, "bold"),
                                     padx=12, pady=8, bg="#F5F5F5", fg="#555555")
        custom_frame.pack(fill=tk.X, padx=15, pady=5)

        entry_row = tk.Frame(custom_frame, bg="#F5F5F5")
        entry_row.pack(fill=tk.X)

        self.custom_entry = tk.Entry(entry_row, font=("Consolas", 12), relief=tk.SOLID, bd=1)
        self.custom_entry.pack(fill=tk.X, side=tk.LEFT, expand=True, ipady=4)
        self.custom_entry.bind("<Return>", lambda e: self.run_custom())

        tk.Button(entry_row, text="▶ 执行", font=("Microsoft YaHei UI", 10, "bold"),
                  bg="#4A90D9", fg="white", activebackground="#3A7BC8",
                  relief=tk.FLAT, padx=15, cursor="hand2",
                  command=self.run_custom).pack(side=tk.RIGHT, padx=(10, 0))

        tk.Label(custom_frame, text="格式: ID,位置[,速度] 空格分隔多组",
                 fg="#999999", font=("Microsoft YaHei UI", 9), bg="#F5F5F5").pack(anchor=tk.W, pady=(5, 0))

        # 运行日志框
        log_frame = tk.LabelFrame(self.root, text=" 运行日志 ", font=("Microsoft YaHei UI", 10, "bold"),
                                  padx=10, pady=8, bg="#F5F5F5", fg="#555555")
        log_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=(5, 15))

        self.log_text = scrolledtext.ScrolledText(
            log_frame, height=8, font=("Consolas", 10),
            bg="#2B2B2B", fg="#D4D4D4", insertbackground="white",
            relief=tk.FLAT, state=tk.DISABLED
        )
        self.log_text.pack(fill=tk.BOTH, expand=True)

    def log(self, msg):
        def _update():
            self.log_text.config(state=tk.NORMAL)
            self.log_text.insert(tk.END, msg + "\n")
            self.log_text.see(tk.END)
            self.log_text.config(state=tk.DISABLED)
        self.root.after(0, _update)

    def connect(self):
        try:
            self.ser = serial.Serial(SERVO_PORT, BAUD_RATE, timeout=1)
            self.status_dot.config(fg="#5CB85C")
            self.status_label.config(text="已连接")
            self.log(f"[连接成功] {SERVO_PORT} @ {BAUD_RATE}")
        except Exception as e:
            self.status_dot.config(fg="red")
            self.status_label.config(text="连接失败")
            self.log(f"[连接失败] {e}")

    def run_action(self, name, cmd_text):
        if self.ser is None or not self.ser.is_open:
            self.log("[错误] 串口未连接!")
            return
        if self.running:
            self.log("[提示] 有动作正在执行，请等待完成")
            return

        def _run():
            self.running = True
            self.log(f"\n{'─' * 30}")
            self.log(f"▶ {name}")
            self.log(f"  指令: {cmd_text}")
            try:
                pairs = parse_group(cmd_text)
                if pairs is None:
                    self.log("[错误] 指令解析失败")
                    return
                set_servo_positions_batch(self.ser, pairs, self.log)
                self.log(f"  已设置 {len(pairs)} 个舵机")
            except Exception as e:
                self.log(f"[错误] {e}")
            finally:
                self.running = False
                self.log("✓ 动作完成")

        threading.Thread(target=_run, daemon=True).start()

    def run_custom(self):
        cmd_text = self.custom_entry.get().strip()
        if not cmd_text:
            return
        self.run_action("自定义动作", cmd_text)

    def on_close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.log("[断开] 串口已关闭")
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("720x580")
    app = ServoApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
