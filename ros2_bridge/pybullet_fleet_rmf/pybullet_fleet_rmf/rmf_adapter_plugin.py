"""BridgePlugin wrapper for running the RMF fleet adapter in-process."""

from __future__ import annotations

from pybullet_fleet_ros.bridge_plugin import BridgePlugin

from pybullet_fleet_rmf.fleet_adapter import RmfAdapterRuntime, start_adapter_runtime


class RmfAdapterBridgePlugin(BridgePlugin):
    """Start EasyFullControl from inside ``bridge_node``.

    This is the Plugin + Bridge deployment path: RMF commands use the direct
    Python fleet client while the ROS bridge can still expose fleet/per-robot
    interfaces for visualization, debugging, and non-RMF control.
    """

    def __init__(
        self,
        node,
        sim_core,
        config_file: str,
        nav_graph: str = "",
        client_mode: str = "python_fleet",
        use_sim_time: bool = True,
        server_uri: str = "",
    ) -> None:
        super().__init__(node, sim_core)
        self.config_file = config_file
        self.nav_graph = nav_graph
        self.client_mode = client_mode
        self.use_sim_time = bool(use_sim_time)
        self.server_uri = server_uri
        self.runtime: RmfAdapterRuntime | None = None

    def on_init(self) -> None:
        """Start the RMF adapter after the simulation core is ready."""
        self.runtime = start_adapter_runtime(
            node=self.node,
            config_path=self.config_file,
            nav_graph_path=self.nav_graph,
            use_sim_time=self.use_sim_time,
            client_mode=self.client_mode,
            sim_core=self.sim_core,
            server_uri=self.server_uri,
        )
        if self.runtime is None:
            raise RuntimeError("failed to start in-process RMF adapter")
