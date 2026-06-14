from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QCheckBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSlider,
    QVBoxLayout,
    QWidget,
)
from PySide6.QtGui import QKeySequence, QShortcut

from .parameter_state import (
    ControllerParameters,
    ParameterState,
    SharedControlParameters,
    WorkspaceLimits,
)


class ParameterGui(QMainWindow):
    def __init__(self, on_apply):
        super().__init__()
        self.on_apply = on_apply
        self.setWindowTitle("Haptic Controller Parameters")
        self.setMinimumSize(720, 720)

        root = QWidget()
        root.setObjectName("contentRoot")
        root.setMaximumWidth(920)
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(24, 24, 24, 24)
        root_layout.setSpacing(16)
        root_layout.addWidget(self._build_header())
        root_layout.addWidget(self._build_controller_group())
        root_layout.addWidget(self._build_workspace_group())
        root_layout.addWidget(self._build_shared_control_group())
        root_layout.addLayout(self._build_actions())
        root_layout.addStretch()

        scroll_area = QScrollArea()
        scroll_area.setObjectName("scrollArea")
        scroll_area.setWidgetResizable(True)
        scroll_area.setAlignment(Qt.AlignHCenter | Qt.AlignTop)
        scroll_area.setWidget(root)
        self.setCentralWidget(scroll_area)
        QShortcut(QKeySequence.Quit, self, activated=self.close)

        self._set_state(ParameterState())
        self._update_shared_control_enabled()
        self._apply_style()

    def _build_header(self):
        header = QLabel(
            "Tune haptic teleoperation parameters before sending them to the "
            "controller node."
        )
        header.setWordWrap(True)
        header.setAlignment(Qt.AlignHCenter)
        header.setObjectName("headerText")
        return header

    def _build_controller_group(self):
        group = QGroupBox("Controller Parameters")
        layout = self._group_layout(group)

        self.zeta = ParameterSlider(0.0, 5.0, 0.01, 3)
        self.mass = ParameterSlider(0.1, 100.0, 0.1, 3)
        self.damping = ParameterSlider(0.0, 500.0, 1.0, 3)
        self.haptic_damping = ParameterSlider(0.0, 100.0, 0.1, 3)
        self.k_orient = ParameterSlider(0.0, 200.0, 1.0, 3)

        layout.addWidget(self._slider_row("zeta", self.zeta))
        layout.addWidget(self._slider_row("mass", self.mass))
        layout.addWidget(self._slider_row("damping", self.damping))
        layout.addWidget(self._slider_row("haptic damping", self.haptic_damping))
        layout.addWidget(self._slider_row("k_orient", self.k_orient))
        return group

    def _build_workspace_group(self):
        group = QGroupBox("Workspace Limits")
        layout = self._group_layout(group)

        self.x_min = ParameterSlider(-2.0, 2.0, 0.01, 3)
        self.x_max = ParameterSlider(-2.0, 2.0, 0.01, 3)
        self.y_min = ParameterSlider(-2.0, 2.0, 0.01, 3)
        self.y_max = ParameterSlider(-2.0, 2.0, 0.01, 3)
        self.z_min = ParameterSlider(-2.0, 2.0, 0.01, 3)
        self.z_max = ParameterSlider(-2.0, 2.0, 0.01, 3)

        layout.addWidget(self._slider_row("x_min", self.x_min))
        layout.addWidget(self._slider_row("x_max", self.x_max))
        layout.addWidget(self._slider_row("y_min", self.y_min))
        layout.addWidget(self._slider_row("y_max", self.y_max))
        layout.addWidget(self._slider_row("z_min", self.z_min))
        layout.addWidget(self._slider_row("z_max", self.z_max))
        return group

    def _build_shared_control_group(self):
        group = QGroupBox("Shared Control")
        layout = self._group_layout(group)

        self.shared_enabled = QCheckBox("Enable shared control")
        self.shared_enabled.stateChanged.connect(self._update_shared_control_enabled)

        self.grasp_assist = ParameterSlider(0.0, 1.0, 0.01, 2)
        self.target_pursuance_assist = ParameterSlider(0.0, 1.0, 0.01, 2)
        self.desired_trajectory_track = ParameterSlider(0.0, 1.0, 0.01, 2)

        layout.addWidget(self.shared_enabled)
        layout.addWidget(self._slider_row("grasp assist", self.grasp_assist))
        layout.addWidget(
            self._slider_row("target pursuance assist", self.target_pursuance_assist)
        )
        layout.addWidget(
            self._slider_row("desired trajectory track", self.desired_trajectory_track)
        )
        return group

    def _build_actions(self):
        layout = QHBoxLayout()
        layout.setContentsMargins(0, 4, 0, 0)
        layout.setSpacing(8)
        layout.addStretch()

        reset_button = QPushButton("Reset Defaults")
        reset_button.clicked.connect(self._reset_defaults)
        layout.addWidget(reset_button)

        quit_button = QPushButton("Quit")
        quit_button.clicked.connect(self.close)
        layout.addWidget(quit_button)

        apply_button = QPushButton("Apply")
        apply_button.clicked.connect(self._apply)
        apply_button.setDefault(True)
        layout.addWidget(apply_button)
        return layout

    def _group_layout(self, group):
        layout = QVBoxLayout(group)
        layout.setContentsMargins(18, 24, 18, 18)
        layout.setSpacing(14)
        return layout

    def _slider_row(self, name, slider):
        row = QWidget()
        row.setObjectName("sliderRow")
        layout = QHBoxLayout(row)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(14)

        label = QLabel(name)
        label.setObjectName("paramLabel")
        label.setFixedWidth(170)
        label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        layout.addWidget(label)
        layout.addWidget(slider, 1)
        return row

    def _reset_defaults(self):
        self._set_state(ParameterState())
        self._update_shared_control_enabled()

    def _apply(self):
        try:
            payload = self._read_state().to_payload()
        except ValueError as error:
            QMessageBox.warning(self, "Invalid Parameters", str(error))
            return

        self.on_apply(payload)

    def _read_state(self):
        return ParameterState(
            controller=ControllerParameters(
                zeta=self.zeta.value(),
                mass=self.mass.value(),
                damping=self.damping.value(),
                haptic_damping=self.haptic_damping.value(),
                k_orient=self.k_orient.value(),
            ),
            workspace_limits=WorkspaceLimits(
                x_min=self.x_min.value(),
                x_max=self.x_max.value(),
                y_min=self.y_min.value(),
                y_max=self.y_max.value(),
                z_min=self.z_min.value(),
                z_max=self.z_max.value(),
            ),
            shared_control=SharedControlParameters(
                enabled=self.shared_enabled.isChecked(),
                grasp_assist=self.grasp_assist.value(),
                target_pursuance_assist=self.target_pursuance_assist.value(),
                desired_trajectory_track=self.desired_trajectory_track.value(),
            ),
        )

    def _set_state(self, state):
        self.zeta.setValue(state.controller.zeta)
        self.mass.setValue(state.controller.mass)
        self.damping.setValue(state.controller.damping)
        self.haptic_damping.setValue(state.controller.haptic_damping)
        self.k_orient.setValue(state.controller.k_orient)

        self.x_min.setValue(state.workspace_limits.x_min)
        self.x_max.setValue(state.workspace_limits.x_max)
        self.y_min.setValue(state.workspace_limits.y_min)
        self.y_max.setValue(state.workspace_limits.y_max)
        self.z_min.setValue(state.workspace_limits.z_min)
        self.z_max.setValue(state.workspace_limits.z_max)

        self.shared_enabled.setChecked(state.shared_control.enabled)
        self.grasp_assist.setValue(state.shared_control.grasp_assist)
        self.target_pursuance_assist.setValue(
            state.shared_control.target_pursuance_assist
        )
        self.desired_trajectory_track.setValue(
            state.shared_control.desired_trajectory_track
        )

    def _update_shared_control_enabled(self):
        enabled = self.shared_enabled.isChecked()
        self.grasp_assist.setEnabled(enabled)
        self.target_pursuance_assist.setEnabled(enabled)
        self.desired_trajectory_track.setEnabled(enabled)

    def _apply_style(self):
        self.setStyleSheet(
            """
            QMainWindow {
                background: #111827;
            }
            QScrollArea#scrollArea {
                background: #111827;
                border: none;
            }
            QWidget#contentRoot {
                background: #111827;
            }
            QLabel#headerText {
                color: #9ca3af;
                font-size: 13px;
            }
            QLabel#paramLabel {
                color: #d1d5db;
                font-weight: 600;
            }
            QGroupBox {
                background: #1f2937;
                border: 1px solid #374151;
                border-radius: 8px;
                color: #f9fafb;
                font-weight: 600;
                margin-top: 12px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 16px;
                padding: 0 6px;
                color: #f9fafb;
                background: #111827;
            }
            QCheckBox {
                color: #d1d5db;
                spacing: 8px;
                font-weight: 600;
            }
            QCheckBox::indicator {
                width: 16px;
                height: 16px;
            }
            QCheckBox::indicator:unchecked {
                border: 1px solid #6b7280;
                border-radius: 3px;
                background: #111827;
            }
            QCheckBox::indicator:checked {
                border: 1px solid #60a5fa;
                border-radius: 3px;
                background: #3b82f6;
            }
            QPushButton {
                background: #2563eb;
                border: 1px solid #3b82f6;
                border-radius: 6px;
                color: #ffffff;
                font-weight: 600;
                padding: 8px 16px;
            }
            QPushButton:hover {
                background: #1d4ed8;
            }
            QPushButton:pressed {
                background: #1e40af;
            }
            QSlider::groove:horizontal {
                background: #374151;
                border-radius: 4px;
                height: 8px;
            }
            QSlider::sub-page:horizontal {
                background: #60a5fa;
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: #f9fafb;
                border: 2px solid #60a5fa;
                border-radius: 8px;
                margin: -5px 0;
                width: 16px;
            }
            QSlider::handle:horizontal:disabled {
                background: #6b7280;
                border-color: #4b5563;
            }
            QSlider::groove:horizontal:disabled,
            QSlider::sub-page:horizontal:disabled {
                background: #2b3544;
            }
            QLabel#valueLabel {
                color: #f9fafb;
                font-family: Menlo, Consolas, monospace;
                font-weight: 700;
                background: #111827;
                border: 1px solid #374151;
                border-radius: 5px;
                padding: 5px 8px;
            }
            """
        )


class ParameterSlider(QWidget):
    def __init__(self, minimum, maximum, step, decimals):
        super().__init__()
        self.minimum = minimum
        self.maximum = maximum
        self.step = step
        self.decimals = decimals
        self.steps = round((maximum - minimum) / step)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, self.steps)
        self.slider.setSingleStep(1)
        self.slider.valueChanged.connect(self._update_label)

        self.label = QLabel()
        self.label.setObjectName("valueLabel")
        self.label.setFixedWidth(84)
        self.label.setAlignment(Qt.AlignCenter)

        layout.addWidget(self.slider)
        layout.addWidget(self.label)
        self._update_label()

    def value(self):
        value = self.minimum + (self.slider.value() * self.step)
        value = min(max(value, self.minimum), self.maximum)
        return round(value, self.decimals)

    def setValue(self, value):
        step_value = round((value - self.minimum) / self.step)
        self.slider.setValue(min(max(step_value, 0), self.steps))
        self._update_label()

    def setEnabled(self, enabled):
        super().setEnabled(enabled)
        self.slider.setEnabled(enabled)
        self.label.setEnabled(enabled)

    def _update_label(self):
        self.label.setText(f"{self.value():.{self.decimals}f}")
