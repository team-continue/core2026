#!/usr/bin/env python3

import re
import tkinter as tk
import tkinter.font as tkfont
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String


@dataclass(frozen=True)
class BehaviorStyle:
    background: str
    foreground: str


@dataclass(frozen=True)
class ViewModel:
    background: str
    foreground: str
    title: str
    body_lines: tuple[str, ...]
    footer: str
    emergency: bool


DEFAULT_STYLE = BehaviorStyle(background="#173a63", foreground="#ffffff")
DEFAULT_BEHAVIOR_TOPIC = "/behavior_system/state_name"
BEHAVIOR_STYLES = {
    "ATTACK": BehaviorStyle(background="#c78100", foreground="#ffffff"),
    "MANUAL": BehaviorStyle(background="#5f6773", foreground="#ffffff"),
    "AUTO_SELECTED": BehaviorStyle(background="#1f8a4c", foreground="#ffffff"),
    "AUTO_WAYPOINT": BehaviorStyle(background="#2cacc9", foreground="#ffffff"),
    "AUTO_IDLE": BehaviorStyle(background="#24415f", foreground="#ffffff"),
}
EMERGENCY_STYLE = BehaviorStyle(background="#b30d0d", foreground="#ffffff")
DEFAULT_HAZARD_STATUS_TOPIC = "/system/emergency/hazard_status"
DEFAULT_HAZARD_STATUS_COMPAT_TOPIC = "/system/emergency/hazard_state"
POLL_INTERVAL_MS = 100

HAZARD_REASON_ALIASES = {
    "EMERGENCY SWITCH TRIGGERED": "E-STOP",
    "MICROCONTROLLER DIAGNOSTIC FAILURE": "MCU FAILURE",
    "RECEIVER DIAGNOSTIC FAILURE": "RX FAILURE",
    "DESTROY COMMAND RECEIVED": "DESTROYED",
    "SOFTWARE EMERGENCY ACTIVE": "SW EMERGENCY",
    "SYSTEM NORMAL": "",
}


def normalize_behavior_label(value: str) -> str:
    value = value.strip().upper()
    value = re.sub(r"\s+", "_", value)
    value = re.sub(r"[^A-Z0-9_+-]", "", value)
    return value


def normalize_reason_label(value: str) -> str:
    value = value.strip().upper()
    value = re.sub(r"\s+", " ", value)
    return HAZARD_REASON_ALIASES.get(value, value)


class StatusDisplayNode(Node):
    def __init__(self) -> None:
        super().__init__("status_display_gui")

        self.declare_parameter("behavior_topic", DEFAULT_BEHAVIOR_TOPIC)
        self.declare_parameter("hazard_status_topic", DEFAULT_HAZARD_STATUS_TOPIC)
        self.declare_parameter(
            "hazard_status_compat_topic", DEFAULT_HAZARD_STATUS_COMPAT_TOPIC
        )
        self.declare_parameter("hazard_state_topic", "")
        self.declare_parameter("hazard_state_compat_topic", "")
        self.declare_parameter("hazard_label_topic", "/system/emergency/hazard_label")
        self.declare_parameter("fullscreen", True)
        self.declare_parameter("screen_index", 0)
        self.declare_parameter("window_title", "ROS Status Display")

        self.behavior_topic = self.get_parameter("behavior_topic").value
        self.hazard_status_topic = self.get_parameter("hazard_status_topic").value
        self.hazard_status_compat_topic = self.get_parameter("hazard_status_compat_topic").value
        self.hazard_label_topic = self.get_parameter("hazard_label_topic").value
        self.fullscreen = bool(self.get_parameter("fullscreen").value)
        self.screen_index = int(self.get_parameter("screen_index").value)
        self.window_title = self.get_parameter("window_title").value
        parameter_overrides = getattr(self, "_parameter_overrides", {})
        hazard_status_topic_overridden = "hazard_status_topic" in parameter_overrides
        hazard_status_compat_topic_overridden = (
            "hazard_status_compat_topic" in parameter_overrides
        )
        legacy_hazard_status_topic_overridden = "hazard_state_topic" in parameter_overrides
        legacy_hazard_status_compat_topic_overridden = (
            "hazard_state_compat_topic" in parameter_overrides
        )
        legacy_hazard_status_topic = self.get_parameter("hazard_state_topic").value
        legacy_hazard_status_compat_topic = self.get_parameter("hazard_state_compat_topic").value

        if legacy_hazard_status_topic_overridden and legacy_hazard_status_topic:
            if not hazard_status_topic_overridden:
                self.hazard_status_topic = legacy_hazard_status_topic
                self.get_logger().warn(
                    "parameter 'hazard_state_topic' is deprecated; use 'hazard_status_topic'"
                )
            elif self.hazard_status_topic != legacy_hazard_status_topic:
                self.get_logger().warn(
                    "both 'hazard_status_topic' and deprecated 'hazard_state_topic' are set; "
                    "using 'hazard_status_topic'"
                )
        if legacy_hazard_status_compat_topic_overridden and legacy_hazard_status_compat_topic:
            if not hazard_status_compat_topic_overridden:
                self.hazard_status_compat_topic = legacy_hazard_status_compat_topic
                self.get_logger().warn(
                    "parameter 'hazard_state_compat_topic' is deprecated; "
                    "use 'hazard_status_compat_topic'"
                )
            elif self.hazard_status_compat_topic != legacy_hazard_status_compat_topic:
                self.get_logger().warn(
                    "both 'hazard_status_compat_topic' and deprecated "
                    "'hazard_state_compat_topic' are set; using 'hazard_status_compat_topic'"
                )

        self._behavior = ""
        self._hazard_active = True
        self._hazard_state_received = False
        self._hazard_label = ""
        self._dirty = True
        self._subscriptions = []
        latched_hazard_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        live_hazard_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._subscriptions.append(
            self.create_subscription(String, self.behavior_topic, self._behavior_callback, 10)
        )
        self._subscribe_hazard_bool(
            self.hazard_status_topic, latched_hazard_qos, live_hazard_qos
        )
        if (
            self.hazard_status_compat_topic
            and self.hazard_status_compat_topic != self.hazard_status_topic
        ):
            self._subscribe_hazard_bool(
                self.hazard_status_compat_topic, latched_hazard_qos, live_hazard_qos
            )
        self._subscribe_hazard_label(self.hazard_label_topic, latched_hazard_qos, live_hazard_qos)

        self.get_logger().info(
            "status_display_gui started: "
            f"behavior_topic={self.behavior_topic}, "
            f"hazard_status_topic={self.hazard_status_topic}, "
            f"hazard_status_compat_topic={self.hazard_status_compat_topic}, "
            f"hazard_label_topic={self.hazard_label_topic}, "
            f"fullscreen={'true' if self.fullscreen else 'false'}, "
            f"screen_index={self.screen_index}"
        )
        if self.screen_index != 0:
            self.get_logger().warn(
                f"screen_index={self.screen_index} requested, "
                "but the current implementation uses the primary display only"
            )

    def _behavior_callback(self, msg: String) -> None:
        if msg.data == self._behavior:
            return
        self._behavior = msg.data
        self._dirty = True

    def _subscribe_hazard_bool(
        self,
        topic: str,
        latched_qos: QoSProfile,
        live_qos: QoSProfile,
    ) -> None:
        if not topic:
            return
        self._subscriptions.append(
            self.create_subscription(Bool, topic, self._hazard_state_callback, latched_qos)
        )
        self._subscriptions.append(
            self.create_subscription(Bool, topic, self._hazard_state_callback, live_qos)
        )

    def _subscribe_hazard_label(
        self,
        topic: str,
        latched_qos: QoSProfile,
        live_qos: QoSProfile,
    ) -> None:
        if not topic:
            return
        self._subscriptions.append(
            self.create_subscription(String, topic, self._hazard_label_callback, latched_qos)
        )
        self._subscriptions.append(
            self.create_subscription(String, topic, self._hazard_label_callback, live_qos)
        )

    def _hazard_state_callback(self, msg: Bool) -> None:
        first_message = not self._hazard_state_received
        self._hazard_state_received = True
        if bool(msg.data) == self._hazard_active and not first_message:
            return
        self._hazard_active = bool(msg.data)
        self._dirty = True

    def _hazard_label_callback(self, msg: String) -> None:
        if msg.data == self._hazard_label:
            return
        self._hazard_label = msg.data
        self._dirty = True

    def take_dirty(self) -> bool:
        dirty = self._dirty
        self._dirty = False
        return dirty

    def build_view_model(self) -> ViewModel:
        if not self._hazard_state_received:
            return ViewModel(
                background=EMERGENCY_STYLE.background,
                foreground=EMERGENCY_STYLE.foreground,
                title="STATUS UNKNOWN",
                body_lines=("WAITING FOR", "HAZARD STATUS"),
                footer="FAIL-SAFE",
                emergency=True,
            )
        if self._hazard_active:
            reasons = self._format_hazard_lines()
            return ViewModel(
                background=EMERGENCY_STYLE.background,
                foreground=EMERGENCY_STYLE.foreground,
                title="EMERGENCY",
                body_lines=tuple(reasons),
                footer="HAZARD ACTIVE",
                emergency=True,
            )

        behavior = normalize_behavior_label(self._behavior)
        if not behavior:
            behavior = "WAITING"
        style = BEHAVIOR_STYLES.get(behavior, DEFAULT_STYLE)
        return ViewModel(
            background=style.background,
            foreground=style.foreground,
            title="",
            body_lines=(behavior,),
            footer="NORMAL",
            emergency=False,
        )

    def _format_hazard_lines(self) -> list[str]:
        parts = [part.strip() for part in self._hazard_label.split(",")]
        labels: list[str] = []
        seen = set()
        for part in parts:
            if not part:
                continue
            label = normalize_reason_label(part)
            if not label or label in seen:
                continue
            seen.add(label)
            labels.append(label)

        if not labels:
            return ["HAZARD ACTIVE"]
        if len(labels) <= 2:
            return labels
        return [labels[0], labels[1], f"+{len(labels) - 2} MORE"]


class StatusDisplayWindow:
    def __init__(self, root: tk.Tk, node: StatusDisplayNode) -> None:
        self.root = root
        self.node = node
        self.current_model = node.build_view_model()

        self.root.title(self.node.window_title)
        self.root.configure(bg=self.current_model.background)
        self.root.bind("<Escape>", self._handle_escape)
        self.root.bind("<Configure>", self._handle_resize)
        self.root.configure(cursor="none")

        if self.node.fullscreen:
            try:
                self.root.attributes("-fullscreen", True)
            except tk.TclError as exc:
                self.node.get_logger().warn(
                    f"fullscreen setup failed ({exc}); falling back to windowed mode"
                )
                self.root.geometry("1280x720+0+0")
        else:
            self.root.geometry("1280x720+0+0")

        self.title_font = tkfont.Font(family="Helvetica", weight="bold")
        self.body_font = tkfont.Font(family="Helvetica", weight="bold")
        self.footer_font = tkfont.Font(family="Helvetica", weight="bold")

        self.title_label = tk.Label(
            root,
            text="",
            font=self.title_font,
            bd=0,
            justify="center",
        )
        self.body_label = tk.Label(
            root,
            text="",
            font=self.body_font,
            bd=0,
            justify="center",
        )
        self.footer_label = tk.Label(
            root,
            text="",
            font=self.footer_font,
            bd=0,
            justify="center",
        )

        self.title_label.place(relx=0.5, rely=0.16, anchor="center")
        self.body_label.place(relx=0.5, rely=0.50, anchor="center")
        self.footer_label.place(relx=0.5, rely=0.92, anchor="center")

        self.apply_model(self.current_model)
        self.root.after(POLL_INTERVAL_MS, self._poll_ros)

    def _handle_escape(self, _event) -> None:
        self.root.destroy()

    def _handle_resize(self, _event) -> None:
        self._update_fonts()
        self._apply_wraplength()

    def _poll_ros(self) -> None:
        if not rclpy.ok():
            return

        rclpy.spin_once(self.node, timeout_sec=0.0)
        if self.node.take_dirty():
            self.apply_model(self.node.build_view_model())

        if self.root.winfo_exists():
            self.root.after(POLL_INTERVAL_MS, self._poll_ros)

    def apply_model(self, model: ViewModel) -> None:
        self.current_model = model
        widgets = (self.root, self.title_label, self.body_label, self.footer_label)
        for widget in widgets:
            widget.configure(bg=model.background)
        for label in (self.title_label, self.body_label, self.footer_label):
            label.configure(fg=model.foreground)

        self.title_label.configure(text=model.title)
        self.body_label.configure(text="\n".join(model.body_lines))
        self.footer_label.configure(text=model.footer)

        self._update_fonts()
        self._apply_wraplength()

    def _apply_wraplength(self) -> None:
        width = max(self.root.winfo_width(), 1280)
        self.body_label.configure(wraplength=int(width * 0.86))

    def _update_fonts(self) -> None:
        height = max(self.root.winfo_height(), 720)
        width = max(self.root.winfo_width(), 1280)
        if self.current_model.emergency:
            title_size = max(72, min(150, int(height * 0.18)))
            body_size = max(40, min(72, int(height * 0.08)))
            footer_size = max(28, min(52, int(height * 0.055)))
        else:
            title_size = max(1, int(height * 0.01))
            body_size = max(120, min(170, int(height * 0.19)))
            footer_size = max(28, min(52, int(height * 0.055)))

        self.title_font.configure(size=title_size)
        self.body_font.configure(size=self._fit_body_font_size(body_size, width))
        self.footer_font.configure(size=footer_size)

    def _fit_body_font_size(self, max_size: int, width: int) -> int:
        lines = tuple(line for line in self.current_model.body_lines if line)
        if not lines:
            return max_size

        available_width = int(width * 0.86)
        min_size = 40 if self.current_model.emergency else 80
        size = max_size
        while size > min_size:
            self.body_font.configure(size=size)
            if max(self.body_font.measure(line) for line in lines) <= available_width:
                return size
            size -= 2
        return min_size


def main() -> None:
    rclpy.init()
    node = StatusDisplayNode()
    root = tk.Tk()
    app = StatusDisplayWindow(root, node)

    try:
        root.mainloop()
    finally:
        try:
            if app.root.winfo_exists():
                app.root.destroy()
        except tk.TclError:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
