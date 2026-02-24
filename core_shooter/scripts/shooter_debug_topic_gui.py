#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile
from core_msgs.msg import CAN, CANArray
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Int8, Int32


class DebugTopicPublisher(Node):
    def __init__(self) -> None:
        super().__init__("shooter_debug_topic_gui")
        self.monitor_listeners = {}
        self.monitor_subscriptions = []
        self.joint_states_listener = None
        self.can_tx_listener = None
        self.monitor_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.bool_publishers = {
            "/test_mode": self.create_publisher(Bool, "/test_mode", 10),
            "/manual_mode": self.create_publisher(Bool, "/manual_mode", 10),
            "/system/emergency/hazard_status": self.create_publisher(
                Bool, "/system/emergency/hazard_status", 10
            ),
            "/left_shoot_once": self.create_publisher(Bool, "/left_shoot_once", 10),
            "/left_shoot_burst": self.create_publisher(Bool, "/left_shoot_burst", 10),
            "/left_shoot_fullauto": self.create_publisher(Bool, "/left_shoot_fullauto", 10),
            "/right_shoot_once": self.create_publisher(Bool, "/right_shoot_once", 10),
            "/right_shoot_burst": self.create_publisher(Bool, "/right_shoot_burst", 10),
            "/right_shoot_fullauto": self.create_publisher(Bool, "/right_shoot_fullauto", 10),
            "/left/disk_hold_state": self.create_publisher(Bool, "/left/disk_hold_state", 10),
            "/right/disk_hold_state": self.create_publisher(Bool, "/right/disk_hold_state", 10),
        }
        self.float_publishers = {
            "/left/test_yaw_angle": self.create_publisher(Float32, "/left/test_yaw_angle", 10),
            "/left/test_pitch_angle": self.create_publisher(Float32, "/left/test_pitch_angle", 10),
            "/right/test_yaw_angle": self.create_publisher(Float32, "/right/test_yaw_angle", 10),
            "/right/test_pitch_angle": self.create_publisher(
                Float32, "/right/test_pitch_angle", 10
            ),
            "/left/shoot_motor": self.create_publisher(Float32, "/left/shoot_motor", 10),
            "/right/shoot_motor": self.create_publisher(Float32, "/right/shoot_motor", 10),
        }
        self.int8_publishers = {
            "/left/reloading": self.create_publisher(Int8, "/left/reloading", 10),
            "/right/reloading": self.create_publisher(Int8, "/right/reloading", 10),
        }
        self.point_publishers = {
            "/left/target_image_position": self.create_publisher(
                PointStamped, "/left/target_image_position", 10
            ),
            "/right/target_image_position": self.create_publisher(
                PointStamped, "/right/target_image_position", 10
            ),
        }
        self.can_tx_pub = self.create_publisher(CANArray, "/can/tx", 10)
        self._add_monitor_subscription("/left/remaining_disk", Int8)
        self._add_monitor_subscription("/right/remaining_disk", Int8)
        self._add_monitor_subscription("/left/shoot_status", Bool)
        self._add_monitor_subscription("/right/shoot_status", Bool)
        self._add_monitor_subscription("/left/shoot_cmd", Int32)
        self._add_monitor_subscription("/right/shoot_cmd", Int32)
        self._add_monitor_subscription("/left/distance", Int32)
        self._add_monitor_subscription("/right/distance", Int32)
        self.joint_states_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_states_callback, self.monitor_qos
        )
        self.can_tx_monitor_sub = self.create_subscription(
            CANArray, "/can/tx", self._can_tx_callback, self.monitor_qos
        )

    def publish_bool(self, topic: str, value: bool) -> None:
        msg = Bool()
        msg.data = value
        self.bool_publishers[topic].publish(msg)
        self.get_logger().info(f"publish {topic}={value}")

    def publish_float(self, topic: str, value: float) -> None:
        msg = Float32()
        msg.data = float(value)
        self.float_publishers[topic].publish(msg)
        self.get_logger().info(f"publish {topic}={msg.data:.4f}")

    def publish_can_single(self, can_id: int, data: float) -> None:
        can_array = CANArray()
        can = CAN()
        can.id = int(can_id)
        can.data.append(float(data))
        can_array.array.append(can)
        self.can_tx_pub.publish(can_array)
        self.get_logger().info(f"publish /can/tx: id={can.id}, data=[{float(data):.4f}]")

    def publish_int8(self, topic: str, value: int) -> None:
        msg = Int8()
        msg.data = int(value)
        self.int8_publishers[topic].publish(msg)
        self.get_logger().info(f"publish {topic}={msg.data}")

    def publish_point_stamped(self, topic: str, x: float, y: float, z: float = 0.0) -> None:
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "image_center"
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = 0.0
        self.point_publishers[topic].publish(msg)
        self.get_logger().info(
            f"publish {topic}: point=({msg.point.x:.1f}, {msg.point.y:.1f}, {msg.point.z:.1f})"
        )

    def set_monitor_listener(self, topic: str, listener) -> None:
        self.monitor_listeners[topic] = listener

    def set_joint_states_listener(self, listener) -> None:
        self.joint_states_listener = listener

    def set_can_tx_listener(self, listener) -> None:
        self.can_tx_listener = listener

    def _add_monitor_subscription(self, topic: str, msg_type) -> None:
        sub = self.create_subscription(
            msg_type,
            topic,
            lambda msg, t=topic: self._monitor_callback(t, msg),
            self.monitor_qos,
        )
        self.monitor_subscriptions.append(sub)

    def _monitor_callback(self, topic: str, msg) -> None:
        listener = self.monitor_listeners.get(topic)
        if listener is None:
            return
        listener(msg.data)

    def _joint_states_callback(self, msg: JointState) -> None:
        if self.joint_states_listener is None:
            return

        pos_values = {i: "--" for i in range(17)}
        vel_values = {i: "--" for i in range(17)}

        for i in range(min(17, len(msg.position))):
            pos_values[i] = f"{msg.position[i]:.4f}"
        for i in range(min(17, len(msg.velocity))):
            vel_values[i] = f"{msg.velocity[i]:.4f}"

        meta_text = (
            f"name={len(msg.name)} "
            f"pos={len(msg.position)} "
            f"vel={len(msg.velocity)} "
            f"eff={len(msg.effort)}"
        )
        self.joint_states_listener(pos_values, vel_values, meta_text)

    def _can_tx_callback(self, msg: CANArray) -> None:
        if self.can_tx_listener is None:
            return

        if not msg.array:
            self.can_tx_listener({}, "frames=0", "")
            return

        updates = {}
        extras = []
        for frame in msg.array:
            if not frame.data:
                value_text = "-"
            elif len(frame.data) == 1:
                value_text = f"{frame.data[0]:.4f}"
            else:
                preview = ", ".join(f"{x:.3f}" for x in frame.data[:3])
                suffix = "..." if len(frame.data) > 3 else ""
                value_text = f"[{preview}{suffix}]"

            frame_id = int(frame.id)
            if 0 <= frame_id <= 16:
                updates[frame_id] = value_text
            else:
                extras.append(f"id={frame_id}:{value_text}")

        extra_text = "; ".join(extras)
        self.can_tx_listener(updates, f"frames={len(msg.array)}", extra_text)


class DebugGui:
    def __init__(self, root: tk.Tk, node: DebugTopicPublisher) -> None:
        self.root = root
        self.node = node
        self.root.title("ROS2 Shooter Debug Topic Publisher")

        self.status_var = tk.StringVar(value="Ready")
        self.monitor_vars = {
            "/left/remaining_disk": tk.StringVar(value="--"),
            "/right/remaining_disk": tk.StringVar(value="--"),
            "/left/shoot_status": tk.StringVar(value="--"),
            "/right/shoot_status": tk.StringVar(value="--"),
            "/left/shoot_cmd": tk.StringVar(value="--"),
            "/right/shoot_cmd": tk.StringVar(value="--"),
            "/left/distance": tk.StringVar(value="--"),
            "/right/distance": tk.StringVar(value="--"),
        }
        self.joint_states_meta_var = tk.StringVar(value="name=0 pos=0 vel=0 eff=0")
        self.joint_state_pos_vars = {i: tk.StringVar(value="--") for i in range(17)}
        self.joint_state_vel_vars = {i: tk.StringVar(value="--") for i in range(17)}
        self.can_tx_frames_var = tk.StringVar(value="frames=0")
        self.can_tx_extra_var = tk.StringVar(value="")
        self.can_tx_id_vars = {i: tk.StringVar(value="--") for i in range(17)}
        self.left_yaw_var = tk.DoubleVar(value=0.0)
        self.left_pitch_var = tk.DoubleVar(value=0.0)
        self.right_yaw_var = tk.DoubleVar(value=0.0)
        self.right_pitch_var = tk.DoubleVar(value=0.0)
        self.left_shoot_motor_var = tk.DoubleVar(value=0.0)
        self.right_shoot_motor_var = tk.DoubleVar(value=0.0)
        self.left_yaw_entry_var = tk.StringVar(value="0.0")
        self.left_pitch_entry_var = tk.StringVar(value="0.0")
        self.right_yaw_entry_var = tk.StringVar(value="0.0")
        self.right_pitch_entry_var = tk.StringVar(value="0.0")
        self.left_shoot_motor_entry_var = tk.StringVar(value="0.0")
        self.right_shoot_motor_entry_var = tk.StringVar(value="0.0")
        self.can_tx_id_entry_var = tk.StringVar(value="0")
        self.can_tx_data_entry_var = tk.StringVar(value="0.0")
        self.target_pad_side_var = tk.StringVar(value="left")
        self.target_pad_coord_var = tk.StringVar(
            value="x=0.0, y=0.0 (center origin, +x right, +y up)"
        )
        self.target_pad_topic_var = tk.StringVar(value="/left/target_image_position")
        self.target_pad_display_w = 640
        self.target_pad_display_h = 360
        self.target_pad_canvas = None
        self.target_pad_marker = None
        self.target_pad_return_job = None
        self.target_pad_current_x = 0.0
        self.target_pad_current_y = 0.0

        self.test_mode_var = tk.BooleanVar(value=False)
        self.manual_mode_var = tk.BooleanVar(value=False)
        self.left_disk_hold_state_var = tk.BooleanVar(value=False)
        self.right_disk_hold_state_var = tk.BooleanVar(value=False)
        self.hazard_status_var = tk.BooleanVar(value=False)
        self.left_reloading_entry_var = tk.StringVar(value="0")
        self.right_reloading_entry_var = tk.StringVar(value="0")
        self.left_fullauto_var = tk.BooleanVar(value=False)
        self.right_fullauto_var = tk.BooleanVar(value=False)

        for topic in self.monitor_vars:
            self.node.set_monitor_listener(
                topic, lambda value, topic=topic: self._on_monitor_value(topic, value)
            )
        self.node.set_joint_states_listener(self._on_joint_states_grid)
        self.node.set_can_tx_listener(self._on_can_tx_grid)
        self._build_ui()
        self._fit_window_to_content()
        self._schedule_spin()

    def _build_ui(self) -> None:
        outer = ttk.Frame(self.root)
        outer.pack(fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(outer, highlightthickness=0)
        scrollbar = ttk.Scrollbar(outer, orient=tk.VERTICAL, command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=scrollbar.set)
        self.v_scrollbar = scrollbar

        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        container = ttk.Frame(self.canvas, padding=12)
        self.scroll_container = container
        self.canvas_window_id = self.canvas.create_window((0, 0), window=container, anchor="nw")
        container.bind("<Configure>", self._on_scroll_frame_configure)
        self.canvas.bind("<Configure>", self._on_canvas_configure)
        self.root.bind_all("<MouseWheel>", self._on_mousewheel)
        self.root.bind_all("<Button-4>", self._on_mousewheel)
        self.root.bind_all("<Button-5>", self._on_mousewheel)

        notes = (
            "Bool/Shooter topics are published to actual runtime topic names (absolute).\n"
            "Monitor shows left/right runtime values plus /joint_states and /can/tx grid."
        )
        ttk.Label(container, text=notes, justify=tk.LEFT).pack(anchor=tk.W, pady=(0, 10))

        content_area = ttk.Frame(container)
        content_area.pack(fill=tk.BOTH, expand=True)

        left_column = ttk.Frame(content_area)
        right_column = ttk.Frame(content_area)
        left_column.grid(row=0, column=0, sticky=tk.NSEW, padx=(0, 8))
        right_column.grid(row=0, column=1, sticky=tk.NSEW)

        content_area.columnconfigure(0, weight=2)
        content_area.columnconfigure(1, weight=3)
        content_area.rowconfigure(0, weight=1)

        self._build_can_tx_frame(left_column)
        self._build_state_frame(left_column)
        self._build_shoot_frame(left_column)
        self._build_monitor_frame(right_column)
        self._build_target_image_pad_frame(container)
        self._build_angle_frame(container)

        self.status_separator = ttk.Separator(self.root, orient=tk.HORIZONTAL)
        self.status_separator.pack(fill=tk.X, padx=12, pady=(0, 6))
        self.status_label = ttk.Label(self.root, textvariable=self.status_var)
        self.status_label.pack(anchor=tk.W, padx=12, pady=(0, 8))

    def _build_monitor_frame(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Monitor", padding=10)
        frame.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(frame, text="Item").grid(row=0, column=0, sticky=tk.W, padx=(0, 8))
        ttk.Label(frame, text="Left").grid(row=0, column=1, sticky=tk.W, padx=(0, 8))
        ttk.Label(frame, text="Right").grid(row=0, column=2, sticky=tk.W, padx=(0, 8))
        ttk.Label(frame, text="Type").grid(row=0, column=3, sticky=tk.W)

        rows = [
            ("remaining_disk", "/left/remaining_disk", "/right/remaining_disk", "Int8"),
            ("shoot_status", "/left/shoot_status", "/right/shoot_status", "Bool"),
            ("shoot_cmd", "/left/shoot_cmd", "/right/shoot_cmd", "Int32"),
            ("distance", "/left/distance", "/right/distance", "Int32"),
        ]
        for row_index, (label, left_topic, right_topic, type_name) in enumerate(rows, start=1):
            ttk.Label(frame, text=label).grid(
                row=row_index,
                column=0,
                sticky=tk.W,
                padx=(0, 8),
                pady=(6 if row_index > 1 else 4, 0),
            )
            ttk.Label(frame, textvariable=self.monitor_vars[left_topic], width=12).grid(
                row=row_index,
                column=1,
                sticky=tk.W,
                padx=(0, 8),
                pady=(6 if row_index > 1 else 4, 0),
            )
            ttk.Label(frame, textvariable=self.monitor_vars[right_topic], width=12).grid(
                row=row_index,
                column=2,
                sticky=tk.W,
                padx=(0, 8),
                pady=(6 if row_index > 1 else 4, 0),
            )
            ttk.Label(frame, text=type_name).grid(
                row=row_index, column=3, sticky=tk.W, pady=(6 if row_index > 1 else 4, 0)
            )

        summary_row = len(rows) + 1
        ttk.Separator(frame, orient=tk.HORIZONTAL).grid(
            row=summary_row, column=0, columnspan=4, sticky=tk.EW, pady=(8, 6)
        )

        detail_panel = ttk.Frame(frame)
        detail_panel.grid(row=summary_row + 1, column=0, columnspan=4, sticky=tk.EW, pady=(0, 4))

        joint_wrap = ttk.LabelFrame(detail_panel, text="/joint_states", padding=8)
        joint_wrap.grid(row=0, column=0, sticky=tk.NSEW, padx=(0, 8))
        can_tx_wrap = ttk.LabelFrame(detail_panel, text="/can/tx", padding=8)
        can_tx_wrap.grid(row=0, column=1, sticky=tk.NSEW)

        joint_panel = ttk.Frame(joint_wrap)
        joint_panel.pack(fill=tk.X, expand=True)

        ttk.Label(joint_panel, textvariable=self.joint_states_meta_var).grid(
            row=0, column=0, columnspan=3, sticky=tk.W, pady=(0, 4)
        )
        ttk.Label(joint_panel, text="name(idx)").grid(row=1, column=0, sticky=tk.W, padx=(0, 8))
        ttk.Label(joint_panel, text="pos").grid(row=1, column=1, sticky=tk.W, padx=(0, 8))
        ttk.Label(joint_panel, text="vel").grid(row=1, column=2, sticky=tk.W)

        for i in range(17):
            row = i + 2
            ttk.Label(joint_panel, text=str(i), width=8).grid(
                row=row, column=0, sticky=tk.W, padx=(0, 8), pady=(0, 2)
            )
            ttk.Label(joint_panel, textvariable=self.joint_state_pos_vars[i], width=12).grid(
                row=row, column=1, sticky=tk.W, padx=(0, 8), pady=(0, 2)
            )
            ttk.Label(joint_panel, textvariable=self.joint_state_vel_vars[i], width=12).grid(
                row=row, column=2, sticky=tk.W, pady=(0, 2)
            )

        joint_panel.columnconfigure(1, weight=1)
        joint_panel.columnconfigure(2, weight=1)

        can_tx_panel = ttk.Frame(can_tx_wrap)
        can_tx_panel.pack(fill=tk.X, expand=True)

        ttk.Label(can_tx_panel, textvariable=self.can_tx_frames_var).grid(
            row=0, column=0, columnspan=2, sticky=tk.W, pady=(0, 4)
        )
        ttk.Label(can_tx_panel, text="id").grid(row=1, column=0, sticky=tk.W, padx=(0, 8))
        ttk.Label(can_tx_panel, text="data").grid(row=1, column=1, sticky=tk.W)

        for can_id in range(17):
            row = can_id + 2
            ttk.Label(can_tx_panel, text=f"{can_id:02d}", width=8).grid(
                row=row, column=0, sticky=tk.W, padx=(0, 8), pady=(0, 2)
            )
            ttk.Label(can_tx_panel, textvariable=self.can_tx_id_vars[can_id], width=12).grid(
                row=row, column=1, sticky=tk.W, pady=(0, 2)
            )

        can_tx_panel.columnconfigure(1, weight=1)

        detail_panel.columnconfigure(0, weight=3)
        detail_panel.columnconfigure(1, weight=2)

        frame.columnconfigure(1, weight=1)
        frame.columnconfigure(2, weight=1)
        frame.columnconfigure(3, weight=1)

    def _build_target_image_pad_frame(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Target Image Position Pad", padding=10)
        frame.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(
            frame,
            text=(
                "Click pad to publish target_image_position as geometry_msgs/PointStamped "
                "(logical area 1280x720, origin at center)"
            ),
            justify=tk.LEFT,
        ).grid(row=0, column=0, columnspan=4, sticky=tk.W, pady=(0, 6))

        ttk.Label(frame, text="Target").grid(row=1, column=0, sticky=tk.W, padx=(0, 8))
        ttk.Radiobutton(
            frame,
            text="Left (/left/target_image_position)",
            value="left",
            variable=self.target_pad_side_var,
            command=self._update_target_pad_topic_label,
        ).grid(row=1, column=1, sticky=tk.W, padx=(0, 8))
        ttk.Radiobutton(
            frame,
            text="Right (/right/target_image_position)",
            value="right",
            variable=self.target_pad_side_var,
            command=self._update_target_pad_topic_label,
        ).grid(row=1, column=2, sticky=tk.W)

        ttk.Label(frame, textvariable=self.target_pad_topic_var).grid(
            row=2, column=0, columnspan=4, sticky=tk.W, pady=(0, 4)
        )
        ttk.Label(frame, textvariable=self.target_pad_coord_var).grid(
            row=3, column=0, columnspan=4, sticky=tk.W, pady=(0, 8)
        )

        canvas = tk.Canvas(
            frame,
            width=self.target_pad_display_w,
            height=self.target_pad_display_h,
            background="#f5f5f5",
            highlightthickness=1,
            highlightbackground="#808080",
        )
        canvas.grid(row=4, column=0, columnspan=4, sticky=tk.W)
        self.target_pad_canvas = canvas

        w = self.target_pad_display_w
        h = self.target_pad_display_h
        cx = w / 2
        cy = h / 2
        canvas.create_rectangle(1, 1, w - 1, h - 1, outline="#404040")
        canvas.create_line(cx, 0, cx, h, fill="#808080")
        canvas.create_line(0, cy, w, cy, fill="#808080")
        canvas.create_text(cx + 6, cy + 10, anchor=tk.NW, text="(0,0)", fill="#404040")
        self._draw_target_pad_marker(cx, cy)

        canvas.bind("<Button-1>", self._on_target_pad_press_or_drag)
        canvas.bind("<B1-Motion>", self._on_target_pad_press_or_drag)
        canvas.bind("<ButtonRelease-1>", self._on_target_pad_release)

        frame.columnconfigure(1, weight=1)
        frame.columnconfigure(2, weight=1)

    def _update_target_pad_topic_label(self) -> None:
        side = self.target_pad_side_var.get()
        if side == "right":
            self.target_pad_topic_var.set("/right/target_image_position")
            return
        self.target_pad_topic_var.set("/left/target_image_position")

    def _on_target_pad_press_or_drag(self, event) -> None:
        if self.target_pad_canvas is None:
            return
        self._cancel_target_pad_return()

        w = float(self.target_pad_display_w)
        h = float(self.target_pad_display_h)
        x_canvas = min(max(float(event.x), 0.0), w)
        y_canvas = min(max(float(event.y), 0.0), h)

        logical_x = (x_canvas - (w / 2.0)) * (1280.0 / w)
        logical_y = ((h / 2.0) - y_canvas) * (720.0 / h)
        logical_x = max(-640.0, min(640.0, logical_x))
        logical_y = max(-360.0, min(360.0, logical_y))

        self._set_target_pad_point(logical_x, logical_y, publish=True)

    def _on_target_pad_release(self, _event) -> None:
        self._start_target_pad_return()

    def _set_target_pad_point(self, logical_x: float, logical_y: float, publish: bool) -> None:
        logical_x = max(-640.0, min(640.0, float(logical_x)))
        logical_y = max(-360.0, min(360.0, float(logical_y)))
        self.target_pad_current_x = logical_x
        self.target_pad_current_y = logical_y

        x_canvas, y_canvas = self._target_pad_canvas_from_logical(logical_x, logical_y)
        self._draw_target_pad_marker(x_canvas, y_canvas)

        topic = (
            "/right/target_image_position"
            if self.target_pad_side_var.get() == "right"
            else "/left/target_image_position"
        )
        self.target_pad_coord_var.set(
            f"x={logical_x:.1f}, y={logical_y:.1f} (center origin, +x right, +y up)"
        )
        self.target_pad_topic_var.set(topic)

        if publish:
            self.node.publish_point_stamped(topic, logical_x, logical_y, 0.0)
            self._set_status(f"Published {topic}: x={logical_x:.1f}, y={logical_y:.1f}, z=0.0")

    def _target_pad_canvas_from_logical(
            self, logical_x: float, logical_y: float) -> tuple[float, float]:
        w = float(self.target_pad_display_w)
        h = float(self.target_pad_display_h)
        x_canvas = (logical_x / 1280.0) * w + (w / 2.0)
        y_canvas = (h / 2.0) - (logical_y / 720.0) * h
        return x_canvas, y_canvas

    def _cancel_target_pad_return(self) -> None:
        if self.target_pad_return_job is None:
            return
        try:
            self.root.after_cancel(self.target_pad_return_job)
        except Exception:
            pass
        self.target_pad_return_job = None

    def _start_target_pad_return(self) -> None:
        self._cancel_target_pad_return()
        self.target_pad_return_job = self.root.after(30, self._step_target_pad_return)

    def _step_target_pad_return(self) -> None:
        self.target_pad_return_job = None

        next_x = self.target_pad_current_x * 0.90
        next_y = self.target_pad_current_y * 0.90
        if abs(next_x) < 0.5 and abs(next_y) < 0.5:
            self._set_target_pad_point(0.0, 0.0, publish=True)
            return

        self._set_target_pad_point(next_x, next_y, publish=True)
        self.target_pad_return_job = self.root.after(30, self._step_target_pad_return)

    def _draw_target_pad_marker(self, x_canvas: float, y_canvas: float) -> None:
        if self.target_pad_canvas is None:
            return

        canvas = self.target_pad_canvas
        if self.target_pad_marker is not None:
            canvas.delete(self.target_pad_marker)

        r = 5
        self.target_pad_marker = canvas.create_oval(
            x_canvas - r,
            y_canvas - r,
            x_canvas + r,
            y_canvas + r,
            outline="#d22",
            width=2,
        )

    def _build_angle_frame(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Aimbot Test Angles", padding=10)
        frame.pack(fill=tk.X, pady=(0, 10))

        left_frame = ttk.LabelFrame(frame, text="Left")
        left_frame.grid(row=0, column=0, sticky=tk.NSEW, padx=(0, 8))
        right_frame = ttk.LabelFrame(frame, text="Right")
        right_frame.grid(row=0, column=1, sticky=tk.NSEW)

        self._build_angle_row(
            left_frame,
            row=0,
            topic="/left/test_yaw_angle",
            label="Yaw [rad]",
            slider_var=self.left_yaw_var,
            entry_var=self.left_yaw_entry_var,
            slider_min=-1.0,
            slider_max=1.0,
            spring_return=True,
            live_publish=True,
        )
        self._build_angle_row(
            left_frame,
            row=1,
            topic="/left/test_pitch_angle",
            label="Pitch [rad]",
            slider_var=self.left_pitch_var,
            entry_var=self.left_pitch_entry_var,
            slider_min=-1.0,
            slider_max=1.0,
            spring_return=True,
            live_publish=True,
        )
        ttk.Button(
            left_frame,
            text="Publish Left Angles",
            command=lambda: self._publish_angle_pair(
                "/left/test_yaw_angle",
                self.left_yaw_entry_var,
                "/left/test_pitch_angle",
                self.left_pitch_entry_var,
            ),
        ).grid(row=2, column=0, columnspan=5, sticky=tk.EW, padx=8, pady=(8, 8))

        self._build_angle_row(
            right_frame,
            row=0,
            topic="/right/test_yaw_angle",
            label="Yaw [rad]",
            slider_var=self.right_yaw_var,
            entry_var=self.right_yaw_entry_var,
            slider_min=-1.0,
            slider_max=1.0,
            spring_return=True,
            live_publish=True,
        )
        self._build_angle_row(
            right_frame,
            row=1,
            topic="/right/test_pitch_angle",
            label="Pitch [rad]",
            slider_var=self.right_pitch_var,
            entry_var=self.right_pitch_entry_var,
            slider_min=-1.0,
            slider_max=1.0,
            spring_return=True,
            live_publish=True,
        )
        ttk.Button(
            right_frame,
            text="Publish Right Angles",
            command=lambda: self._publish_angle_pair(
                "/right/test_yaw_angle",
                self.right_yaw_entry_var,
                "/right/test_pitch_angle",
                self.right_pitch_entry_var,
            ),
        ).grid(row=2, column=0, columnspan=5, sticky=tk.EW, padx=8, pady=(8, 8))

        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)
        frame.rowconfigure(0, weight=1)

    def _build_can_tx_frame(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="CAN TX (Single CAN in CANArray)", padding=10)
        frame.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(frame, text="/can/tx").grid(row=0, column=0, sticky=tk.W, padx=(0, 8), pady=4)
        ttk.Label(frame, text="id").grid(row=0, column=1, sticky=tk.E, padx=(0, 4), pady=4)
        ttk.Entry(frame, textvariable=self.can_tx_id_entry_var, width=8).grid(
            row=0, column=2, sticky=tk.W, padx=(0, 8), pady=4
        )
        ttk.Label(frame, text="data(float)").grid(
            row=0, column=3, sticky=tk.E, padx=(0, 4), pady=4
        )
        ttk.Entry(frame, textvariable=self.can_tx_data_entry_var, width=12).grid(
            row=0, column=4, sticky=tk.W, padx=(0, 8), pady=4
        )
        ttk.Button(frame, text="Publish", command=self._publish_can_tx_from_entries).grid(
            row=0, column=5, sticky=tk.EW, pady=4
        )

        ttk.Label(
            frame,
            text="Publishes CANArray{ array: [ CAN{ id, data:[value] } ] }",
        ).grid(row=1, column=0, columnspan=6, sticky=tk.W, pady=(4, 0))

        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(5, weight=0)

    def _build_angle_row(
        self,
        parent: ttk.LabelFrame,
        row: int,
        topic: str,
        label: str,
        slider_var: tk.DoubleVar,
        entry_var: tk.StringVar,
        slider_min: float = -3.14159265359,
        slider_max: float = 3.14159265359,
        spring_return: bool = False,
        live_publish: bool = False,
    ) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky=tk.W, padx=(0, 8), pady=4)

        def on_scale_change(value: str) -> None:
            float_value = float(value)
            entry_var.set(f"{float_value:.4f}")
            if live_publish:
                self.node.publish_float(topic, float_value)
                self._set_status(f"Published {topic}={float_value:.4f}")

        scale = ttk.Scale(
            parent,
            from_=slider_min,
            to=slider_max,
            variable=slider_var,
            command=on_scale_change,
        )
        scale.grid(row=row, column=1, sticky=tk.EW, padx=(0, 8), pady=4)

        entry = ttk.Entry(parent, textvariable=entry_var, width=12)
        entry.grid(row=row, column=2, sticky=tk.W, padx=(0, 8), pady=4)

        def apply_entry_to_slider() -> None:
            try:
                value = float(entry_var.get())
            except ValueError:
                self._set_status(f"Invalid float for {topic}: {entry_var.get()!r}")
                return
            value = max(slider_min, min(slider_max, value))
            entry_var.set(f"{value:.4f}")
            slider_var.set(value)

        ttk.Button(parent, text="Apply", command=apply_entry_to_slider).grid(
            row=row, column=3, sticky=tk.EW, padx=(0, 8), pady=4
        )
        ttk.Button(
            parent,
            text="Publish",
            command=lambda t=topic, v=entry_var: self._publish_float_from_entry(t, v),
        ).grid(row=row, column=4, sticky=tk.EW, pady=4)

        if spring_return:
            def on_scale_release(_event) -> None:
                slider_var.set(0.0)
                entry_var.set("0.0000")
                if live_publish:
                    self.node.publish_float(topic, 0.0)
                    self._set_status(f"Published {topic}=0.0000")

            scale.bind("<ButtonRelease-1>", on_scale_release)

        for col in range(5):
            parent.columnconfigure(col, weight=1 if col in (1, 2) else 0)

    def _build_state_frame(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="States", padding=10)
        frame.pack(fill=tk.X, pady=(0, 10))

        rows = [
            ("/test_mode", self.test_mode_var, "bool"),
            ("/manual_mode", self.manual_mode_var, "bool"),
            ("/left/disk_hold_state", self.left_disk_hold_state_var, "bool"),
            ("/right/disk_hold_state", self.right_disk_hold_state_var, "bool"),
            ("/system/emergency/hazard_status", self.hazard_status_var, "bool"),
        ]
        for row, (topic, var, value_kind) in enumerate(rows):
            ttk.Label(frame, text=topic).grid(row=row, column=0, sticky=tk.W, padx=(0, 8), pady=4)
            ttk.Checkbutton(
                frame,
                text="True / False",
                variable=var,
                command=lambda t=topic, v=var, k=value_kind: self._publish_state_value(
                    t, v.get(), k
                ),
            ).grid(row=row, column=1, sticky=tk.W, padx=(0, 8), pady=4)
            ttk.Button(
                frame,
                text="Publish",
                command=lambda t=topic, v=var, k=value_kind: self._publish_state_value(
                    t, v.get(), k
                ),
            ).grid(row=row, column=2, sticky=tk.EW, pady=4)

        reloading_row_base = len(rows)
        ttk.Separator(frame, orient=tk.HORIZONTAL).grid(
            row=reloading_row_base,
            column=0,
            columnspan=3,
            sticky=tk.EW,
            pady=(8, 6),
        )

        int8_rows = [
            ("/left/reloading", self.left_reloading_entry_var),
            ("/right/reloading", self.right_reloading_entry_var),
        ]
        for idx, (topic, entry_var) in enumerate(int8_rows, start=1):
            row = reloading_row_base + idx
            ttk.Label(frame, text=topic).grid(row=row, column=0, sticky=tk.W, padx=(0, 8), pady=4)
            entry = ttk.Entry(frame, textvariable=entry_var, width=10)
            entry.grid(row=row, column=1, sticky=tk.W, padx=(0, 8), pady=4)
            ttk.Button(
                frame,
                text="Publish",
                command=lambda t=topic, v=entry_var: self._publish_int8_from_entry(t, v),
            ).grid(row=row, column=2, sticky=tk.EW, pady=4)
            ttk.Label(frame, text="Int8").grid(row=row, column=3, sticky=tk.W, padx=(8, 0), pady=4)

        ttk.Button(frame, text="Publish All States", command=self._publish_all_states).grid(
            row=reloading_row_base + len(int8_rows) + 1,
            column=0,
            columnspan=4,
            sticky=tk.EW,
            pady=(8, 0),
        )

        frame.columnconfigure(1, weight=1)
        frame.columnconfigure(2, weight=0)

    def _build_shoot_frame(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Shooter Commands", padding=10)
        frame.pack(fill=tk.BOTH, expand=True)

        left_frame = ttk.LabelFrame(frame, text="Left")
        left_frame.grid(row=0, column=0, sticky=tk.NSEW, padx=(0, 8))
        right_frame = ttk.LabelFrame(frame, text="Right")
        right_frame.grid(row=0, column=1, sticky=tk.NSEW)

        self._build_shooter_side(
            left_frame,
            once_topic="/left_shoot_once",
            burst_topic="/left_shoot_burst",
            fullauto_topic="/left_shoot_fullauto",
            shoot_motor_topic="/left/shoot_motor",
            shoot_motor_slider_var=self.left_shoot_motor_var,
            shoot_motor_entry_var=self.left_shoot_motor_entry_var,
            fullauto_var=self.left_fullauto_var,
        )
        self._build_shooter_side(
            right_frame,
            once_topic="/right_shoot_once",
            burst_topic="/right_shoot_burst",
            fullauto_topic="/right_shoot_fullauto",
            shoot_motor_topic="/right/shoot_motor",
            shoot_motor_slider_var=self.right_shoot_motor_var,
            shoot_motor_entry_var=self.right_shoot_motor_entry_var,
            fullauto_var=self.right_fullauto_var,
        )

        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)
        frame.rowconfigure(0, weight=1)

    def _build_shooter_side(
        self,
        parent: ttk.LabelFrame,
        once_topic: str,
        burst_topic: str,
        fullauto_topic: str,
        shoot_motor_topic: str,
        shoot_motor_slider_var: tk.DoubleVar,
        shoot_motor_entry_var: tk.StringVar,
        fullauto_var: tk.BooleanVar,
    ) -> None:
        parent.configure(padding=10)

        ttk.Button(
            parent, text="Shoot Once", command=lambda t=once_topic: self._pulse_bool(t)
        ).pack(fill=tk.X, pady=(0, 8))
        ttk.Button(
            parent, text="Shoot Burst", command=lambda t=burst_topic: self._pulse_bool(t)
        ).pack(fill=tk.X, pady=(0, 8))
        ttk.Checkbutton(
            parent,
            text="Fullauto (toggle)",
            variable=fullauto_var,
            command=lambda t=fullauto_topic, v=fullauto_var: self._publish_bool(t, v.get()),
        ).pack(anchor=tk.W)

        motor_frame = ttk.LabelFrame(parent, text="shoot_motor (Float32)")
        motor_frame.pack(fill=tk.X, pady=(10, 0))

        ttk.Scale(
            motor_frame,
            from_=0.0,
            to=1.0,
            variable=shoot_motor_slider_var,
            command=lambda value, ev=shoot_motor_entry_var: ev.set(f"{float(value):.4f}"),
        ).grid(row=0, column=0, columnspan=3, sticky=tk.EW, padx=8, pady=(8, 6))

        ttk.Entry(motor_frame, textvariable=shoot_motor_entry_var, width=10).grid(
            row=1, column=0, sticky=tk.W, padx=8, pady=(0, 8)
        )
        ttk.Button(
            motor_frame,
            text="Apply",
            command=lambda v=shoot_motor_entry_var, s=shoot_motor_slider_var, t=shoot_motor_topic:
            self._apply_entry_to_slider(v, s, t),
        ).grid(row=1, column=1, sticky=tk.EW, padx=(0, 6), pady=(0, 8))
        ttk.Button(
            motor_frame,
            text="Publish",
            command=lambda t=shoot_motor_topic, v=shoot_motor_entry_var:
            self._publish_float_from_entry(t, v),
        ).grid(row=1, column=2, sticky=tk.EW, padx=(0, 8), pady=(0, 8))
        ttk.Button(
            motor_frame,
            text="Stop(0)",
            command=lambda t=shoot_motor_topic, v=shoot_motor_entry_var, s=shoot_motor_slider_var:
            self._publish_shoot_motor_zero(t, v, s),
        ).grid(row=2, column=0, columnspan=3, sticky=tk.EW, padx=8, pady=(0, 8))

        motor_frame.columnconfigure(0, weight=1)
        motor_frame.columnconfigure(1, weight=1)
        motor_frame.columnconfigure(2, weight=1)

    def _publish_float_from_entry(self, topic: str, entry_var: tk.StringVar) -> None:
        try:
            value = float(entry_var.get())
        except ValueError:
            self._set_status(f"Invalid float for {topic}: {entry_var.get()!r}")
            return
        if topic in (
            "/left/test_yaw_angle",
            "/left/test_pitch_angle",
            "/right/test_yaw_angle",
            "/right/test_pitch_angle",
        ):
            value = max(-1.0, min(1.0, value))
            entry_var.set(f"{value:.4f}")
        if topic in ("/left/shoot_motor", "/right/shoot_motor"):
            value = max(0.0, min(1.0, value))
            entry_var.set(f"{value:.4f}")
        self.node.publish_float(topic, value)
        self._set_status(f"Published {topic}={value:.4f}")

    def _publish_angle_pair(
        self,
        yaw_topic: str,
        yaw_entry_var: tk.StringVar,
        pitch_topic: str,
        pitch_entry_var: tk.StringVar,
    ) -> None:
        self._publish_float_from_entry(yaw_topic, yaw_entry_var)
        self._publish_float_from_entry(pitch_topic, pitch_entry_var)

    def _publish_can_tx_from_entries(self) -> None:
        try:
            can_id = int(self.can_tx_id_entry_var.get(), 10)
        except ValueError:
            self._set_status(f"Invalid CAN id: {self.can_tx_id_entry_var.get()!r}")
            return
        try:
            data = float(self.can_tx_data_entry_var.get())
        except ValueError:
            self._set_status(f"Invalid CAN data: {self.can_tx_data_entry_var.get()!r}")
            return

        if can_id < 0 or can_id > 255:
            self._set_status(f"CAN id out of range (0-255): {can_id}")
            return

        self.node.publish_can_single(can_id, data)
        self._set_status(f"Published /can/tx id={can_id}, data={data:.4f}")

    def _publish_shoot_motor_zero(
        self, topic: str, entry_var: tk.StringVar, slider_var: tk.DoubleVar
    ) -> None:
        slider_var.set(0.0)
        entry_var.set("0.0")
        self._publish_float_from_entry(topic, entry_var)

    def _apply_entry_to_slider(
        self, entry_var: tk.StringVar, slider_var: tk.DoubleVar, topic: str
    ) -> None:
        try:
            value = float(entry_var.get())
        except ValueError:
            self._set_status(f"Invalid float for {topic}: {entry_var.get()!r}")
            return
        if topic in ("/left/shoot_motor", "/right/shoot_motor"):
            value = max(0.0, min(1.0, value))
            entry_var.set(f"{value:.4f}")
        slider_var.set(value)

    def _publish_bool(self, topic: str, value: bool) -> None:
        self.node.publish_bool(topic, value)
        self._set_status(f"Published {topic}={value}")

    def _publish_int8_from_entry(self, topic: str, entry_var: tk.StringVar) -> None:
        try:
            value = int(entry_var.get(), 10)
        except ValueError:
            self._set_status(f"Invalid Int8 for {topic}: {entry_var.get()!r}")
            return

        value = max(-128, min(127, value))
        entry_var.set(str(value))
        self.node.publish_int8(topic, value)
        self._set_status(f"Published {topic}={value}")

    def _publish_state_value(self, topic: str, checked: bool, value_kind: str) -> None:
        self._publish_bool(topic, checked)

    def _publish_all_states(self) -> None:
        self._publish_bool("/test_mode", self.test_mode_var.get())
        self._publish_bool("/manual_mode", self.manual_mode_var.get())
        self.node.publish_bool("/left/disk_hold_state", self.left_disk_hold_state_var.get())
        self.node.publish_bool("/right/disk_hold_state", self.right_disk_hold_state_var.get())
        self._publish_bool("/system/emergency/hazard_status", self.hazard_status_var.get())
        self._publish_int8_from_entry("/left/reloading", self.left_reloading_entry_var)
        self._publish_int8_from_entry("/right/reloading", self.right_reloading_entry_var)

    def _pulse_bool(self, topic: str) -> None:
        self.node.publish_bool(topic, True)
        self._set_status(f"Pulse publish {topic}=True")

    def _set_status(self, text: str) -> None:
        self.status_var.set(text)

    def _on_monitor_value(self, topic: str, value) -> None:
        if isinstance(value, bool):
            text = "True" if value else "False"
        else:
            text = str(value)
        self.monitor_vars[topic].set(text)

    def _on_joint_states_grid(self, pos_values: dict, vel_values: dict, meta_text: str) -> None:
        self.joint_states_meta_var.set(meta_text)
        for i in range(17):
            self.joint_state_pos_vars[i].set(pos_values.get(i, "--"))
            self.joint_state_vel_vars[i].set(vel_values.get(i, "--"))

    def _on_can_tx_grid(self, updates: dict, frame_text: str, extra_text: str) -> None:
        meta_text = frame_text if not extra_text else f"{frame_text} | extra: {extra_text}"
        self.can_tx_frames_var.set(meta_text)
        for can_id, value_text in updates.items():
            if can_id in self.can_tx_id_vars:
                self.can_tx_id_vars[can_id].set(value_text)
        self.can_tx_extra_var.set(f"extra: {extra_text}" if extra_text else "")

    def _schedule_spin(self) -> None:
        if rclpy.ok():
            # Drain a small batch each tick so high-rate monitor topics stay responsive.
            for _ in range(4):
                rclpy.spin_once(self.node, timeout_sec=0.0)
            self.root.after(5, self._schedule_spin)

    def _fit_window_to_content(self) -> None:
        self.root.update_idletasks()

        desired_w = 1920
        desired_h = 1080

        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()
        max_w, max_h = self.root.maxsize()
        avail_w = max_w if max_w > 1 else screen_w
        avail_h = max_h if max_h > 1 else screen_h

        target_w = min(desired_w, avail_w)
        target_h = min(desired_h, avail_h)

        min_w = min(760, target_w)
        min_h = min(520, target_h)
        self.root.minsize(min_w, min_h)

        pos_x = max((screen_w - target_w) // 2, 0)
        pos_y = max((screen_h - target_h) // 2, 0)
        self.root.geometry(f"{target_w}x{target_h}+{pos_x}+{pos_y}")
        self.root.update_idletasks()

    def _on_scroll_frame_configure(self, _event) -> None:
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _on_canvas_configure(self, event) -> None:
        self.canvas.itemconfigure(self.canvas_window_id, width=event.width)

    def _on_mousewheel(self, event) -> None:
        event_num = getattr(event, "num", None)
        event_delta = getattr(event, "delta", 0)

        if event_num == 4:
            self.canvas.yview_scroll(-1, "units")
            return
        if event_num == 5:
            self.canvas.yview_scroll(1, "units")
            return
        if event_delta:
            self.canvas.yview_scroll(int(-event_delta / 120), "units")


def main() -> None:
    rclpy.init()
    node = DebugTopicPublisher()
    root = tk.Tk()
    gui = DebugGui(root, node)

    def on_close() -> None:
        try:
            if gui.left_fullauto_var.get():
                node.publish_bool("/left_shoot_fullauto", False)
            if gui.right_fullauto_var.get():
                node.publish_bool("/right_shoot_fullauto", False)
        except Exception:
            pass
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)

    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
