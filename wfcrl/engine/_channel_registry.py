"""Output channel/variable registry for FAST.Farm and FLORIS simulators.

Provides ChannelSpec and ChannelRegistry for managing which output
channels/variables are collected during simulation.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional


@dataclass
class ChannelSpec:
    """Metadata for a single output channel (FAST.Farm) or variable (FLORIS).

    Attributes
    ----------
    name : str
        Channel/variable name (e.g. "GenPwr" for FAST.Farm, "turbine_powers" for FLORIS).
    module : str
        FAST.Farm module that owns this channel (e.g. "ServoDyn", "ElastoDyn").
        Empty string for FLORIS variables.
    description : str
        Human-readable description.
    output_key : str
        Key mapping to the SimulationOutput dataclass field.
    unit : str
        Physical unit string.
    group : str
        Logical group for preset selection (e.g. "performance", "loads", "wind").
    api_method : str
        FLORIS API method name. Empty for FAST.Farm channels.
    """

    name: str
    module: str = ""
    description: str = ""
    output_key: str = ""
    unit: str = ""
    group: str = ""
    api_method: str = ""


class ChannelRegistry:
    """Registry for output channel/variable definitions.

    Loads definitions from a YAML reference file or built-in defaults,
    and resolves user selections (channel list or preset name).

    Usage
    -----
    registry = ChannelRegistry(builtin_channels)
    specs = registry.resolve(preset="standard")
    by_module = registry.group_by_module(specs)  # FAST.Farm only
    """

    def __init__(self, channels: List[dict]):
        self._specs: List[ChannelSpec] = [ChannelSpec(**c) for c in channels]
        self._by_name: Dict[str, ChannelSpec] = {s.name: s for s in self._specs}
        self._by_group: Dict[str, List[ChannelSpec]] = {}
        for s in self._specs:
            if s.group:
                self._by_group.setdefault(s.group, []).append(s)

    def resolve(
        self,
        channels: Optional[List[str]] = None,
        preset: Optional[str] = None,
    ) -> List[ChannelSpec]:
        """Resolve user channel selection.

        Priority: channels > preset > "standard" default.

        Parameters
        ----------
        channels : list of str or None
            Explicit channel name list.
        preset : str or None
            Preset group name (e.g. "minimal", "standard", "loads", "full").

        Returns
        -------
        list of ChannelSpec
        """
        if channels is not None:
            result = []
            for name in channels:
                if name in self._by_name:
                    result.append(self._by_name[name])
            return result

        preset = preset or "standard"
        if preset in _BUILTIN_PRESETS:
            return [self._by_name[n] for n in _BUILTIN_PRESETS.get(preset, []) if n in self._by_name]
        if preset in self._by_group:
            return list(self._by_group[preset])

        return [s for s in self._specs if s.group == "performance"]

    def group_by_module(
        self, specs: List[ChannelSpec]
    ) -> Dict[str, List[ChannelSpec]]:
        """Group channel specs by FAST.Farm module name.

        Parameters
        ----------
        specs : list of ChannelSpec

        Returns
        -------
        dict: module_name -> list of ChannelSpec
        """
        result: Dict[str, List[ChannelSpec]] = {}
        for s in specs:
            if s.module:
                result.setdefault(s.module, []).append(s)
        return result

    def find_by_output_key(self, output_key: str) -> Optional[ChannelSpec]:
        """Find a channel by its output_key mapping."""
        for s in self._specs:
            if s.output_key == output_key:
                return s
        return None

    @property
    def default_preset(self) -> str:
        return "standard"

    @property
    def all_channels(self) -> List[ChannelSpec]:
        return list(self._specs)

    def get(self, name: str) -> ChannelSpec:
        if name not in self._by_name:
            raise KeyError(f"Channel '{name}' not found")
        return self._by_name[name]


# ---- Built-in preset definitions ----

_BUILTIN_PRESETS: Dict[str, List[str]] = {
    "minimal": ["GenPwr", "YawPzn", "BldPitch1"],
    "standard": [
        "GenPwr", "GenTq", "RotSpeed", "YawPzn", "BldPitch1",
        "RootMIP1", "RootMOoP1", "RootMzb1",
        "Wind1VelX", "Wind1VelY", "Wind1VelZ",
    ],
    "loads": [
        "GenPwr", "GenTq", "RotSpeed", "YawPzn", "BldPitch1",
        "RootMIP1", "RootMOoP1", "RootMzb1",
        "Wind1VelX", "Wind1VelY", "Wind1VelZ",
    ],
}
