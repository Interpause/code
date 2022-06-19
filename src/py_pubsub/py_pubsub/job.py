from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import (
    ParameterDescriptor,
    FloatingPointRange,
    SetParametersResult
    )

@dataclass(eq=False)
class NiceNode(Node):
    """Extension of Node with some common properties."""

    node_name: str = "undefined"
    """name of node"""
    max_fps: int = 30
    """max rate of node"""

    def __post_init__(self):
        super(NiceNode, self).__init__(self.node_name)

        self._log = self.get_logger()

@dataclass
class JobCfg:
    """Default config for Job."""

    max_rate: int = 30
    """Max rate in Hertz for job."""


@dataclass
class Job(ABC):
    """Base class of common properties for Jobs."""

    node: Node = field(default_factory=Node)
    """Node to attach Job to."""
    ini_cfg: JobCfg = field(default_factory=JobCfg)
    """Initial/default config for Job."""

    @abstractmethod
    def attach_params(self, node: Node, params: JobCfg):
        """Attaches Job config to Node as parameters."""

        node.declare_parameter('max_rate', float(params.max_rate))
        node.set_descriptor('max_rate', ParameterDescriptor(
            description='Max rate in Hertz for job.',
            type=Parameter.Type.DOUBLE.value,
            floating_point_range=[FloatingPointRange(
                from_value=0.0,
                to_value=32767.0,
                step=0.0
            )]
        ))

    @abstractmethod
    def attach_behaviour(self, node: Node, params: JobCfg):
        """Attaches Job behaviours like pub-sub, services, etc to Node."""
        pass

    @abstractmethod
    def on_params_change(self, node: Node, params: dict):
        """Handle change in parameters.

        Args:
            node (Node): ROS Node
            params (dict): Map of parameter name to new value

        Returns:
            Union[Tuple[bool, str], bool]: Whether the change is successful.
        """
        return True, 'NIL'
    
    def update_attrs_from_params(self):
        for name, param in self.node._parameters.items():
            setattr(self, name, param.value)

    def params_to_cfg(self, params: list[Parameter]):
        """Convert list of Parameter to dict."""
        return {p.name:p.value for p in params}

    def _param_change_cb(self, params: list[Parameter]):
        # ros2 only includes the parameters that were changed.

        res = self.on_params_change(self.node, self.params_to_cfg(params))
        if isinstance(res, tuple):
            success, msg = res
            return SetParametersResult(
                successful=success,
                reason=msg
            )
        return SetParametersResult(successful=res)

    def __post_init__(self):
        """Attaches Job to Node."""

        self.log = self.node.get_logger()

        self.attach_params(self.node, self.ini_cfg)
        self.update_attrs_from_params()
        self.attach_behaviour(self.node, self.ini_cfg)
        self.node.add_on_set_parameters_callback(self._param_change_cb)
