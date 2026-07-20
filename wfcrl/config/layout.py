"""
风场布局数据结构与注册表
========================
定义 FarmLayout（纯布局数据）和 LayoutRegistry（注册表）。
布局数据可从内置定义加载，或从 YAML 文件加载（未来支持）。

用法
----
# 从内置注册表加载
registry = LayoutRegistry.from_builtin()
layout = registry.get("HornsRev1")
print(layout.num_turbines, layout.xcoords, layout.ycoords)

# 直接构造（非 RL 场景）
from wfcrl.config.layout import FarmLayout
layout = FarmLayout(
    name="my_farm", num_turbines=3,
    xcoords=[0.0, 504.0, 1008.0],
    ycoords=[0.0, 0.0, 0.0],
)
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Union


# =========================================================================
# FarmLayout — 纯风场布局数据
# =========================================================================

@dataclass
class FarmLayout:
    """
    纯风场布局数据 — 不依赖任何仿真器或 RL 配置。

    可用于 FAST.Farm、FLORIS、或任何其他仿真器。

    Attributes
    ----------
    name : str
        布局名称（如 "HornsRev1", "DafengH1"）。
    num_turbines : int
        风机数量。
    xcoords : List[float]
        风机 X 坐标 (m)。
    ycoords : List[float]
        风机 Y 坐标 (m)。
    turbine_type : str
        风机型号 (如 "nrel_5MW", "goldwind_8_5mw")，默认 "nrel_5MW"。
    metadata : dict
        额外元数据，如 {"source": "Horns Rev 1", "reference": "..."}。
    """
    name: str
    num_turbines: int
    xcoords: List[float]
    ycoords: List[float]
    turbine_type: str = "nrel_5MW"
    metadata: dict = field(default_factory=dict)

    def __post_init__(self):
        if len(self.xcoords) != len(self.ycoords):
            raise ValueError("xcoords and ycoords must have same length")
        if len(self.xcoords) != self.num_turbines:
            self.num_turbines = len(self.xcoords)

    def to_dict(self) -> dict:
        """转为字典，适用于序列化或传递给仿真配置构造。"""
        return {
            "name": self.name,
            "num_turbines": self.num_turbines,
            "xcoords": list(self.xcoords),
            "ycoords": list(self.ycoords),
            "turbine_type": self.turbine_type,
            "metadata": dict(self.metadata),
        }

    @classmethod
    def from_dict(cls, d: dict) -> "FarmLayout":
        """从字典构建 FarmLayout。"""
        return cls(
            name=d["name"],
            num_turbines=d.get("num_turbines", len(d["xcoords"])),
            xcoords=list(d["xcoords"]),
            ycoords=list(d["ycoords"]),
            turbine_type=d.get("turbine_type", "nrel_5MW"),
            metadata=d.get("metadata", {}),
        )


# =========================================================================
# LayoutRegistry — 风场布局注册表
# =========================================================================

class LayoutRegistry:
    """
    风场布局注册表 — 从内置定义或 YAML 文件加载布局。

    用法
    ----
    # 内置布局
    registry = LayoutRegistry.from_builtin()
    layout = registry.get("DafengH1")
    for name in registry.list_names():
        print(name)

    # YAML 目录（未来支持）
    # registry = LayoutRegistry.from_yaml_directory("FarmInputs/layouts/")
    """

    def __init__(self):
        self._layouts: Dict[str, FarmLayout] = {}

    def register(self, layout: FarmLayout) -> None:
        """注册一个布局。同名布局会覆盖。"""
        self._layouts[layout.name] = layout

    def get(self, name: str) -> FarmLayout:
        """
        按名称获取布局。

        Raises
        ------
        KeyError
            未找到时抛出。
        """
        if name not in self._layouts:
            raise KeyError(
                f"Unknown layout: '{name}'. "
                f"Available: {', '.join(sorted(self._layouts.keys()))}"
            )
        return self._layouts[name]

    def list_names(self) -> List[str]:
        """返回所有已注册布局的名称列表。"""
        return list(self._layouts.keys())

    def __contains__(self, name: str) -> bool:
        return name in self._layouts

    def __len__(self) -> int:
        return len(self._layouts)

    # ---- 工厂方法 ----

    @classmethod
    def from_builtin(cls) -> "LayoutRegistry":
        """加载内置布局集合（从硬编码定义）。"""
        registry = cls()

        # === 3 turbines row ===
        registry.register(FarmLayout(
            name="3T",
            num_turbines=3,
            xcoords=[0.0, 504.0, 1008.0],
            ycoords=[0.0, 0.0, 0.0],
            metadata={"source": "builtin"},
        ))

        # === 6 turbines (2 x 3) ===
        registry.register(FarmLayout(
            name="6T",
            num_turbines=6,
            xcoords=[0.0, 504.0, 1008.0, 0.0, 504.0, 1008.0],
            ycoords=[-252.0, -252.0, -252.0, 252.0, 252.0, 252.0],
            metadata={"source": "builtin"},
        ))

        # === 16 turbines ===
        # Bizon Monroc, C., Bušić, A., Dubuc, D., & Zhu, J.
        # "Actor critic agents for wind farm control", 2023
        x16 = [
            891.5, 891.5, 891.5, 2088.6, 2089.1, 2088.2, 3285.7, 3285.3,
            3285.3, 3285.3, 3285.7, 4482.4, 4482.4, 4482.4, 4482.8, 4481.9,
        ]
        x16 = [xc - min(x16) for xc in x16]
        y16 = [
            5169.3, 3743.2, 2317.2, 5168.7, 3743.5, 2318.0, 6594.1, 5169.4,
            3743.4, 2317.3, 892.2, 6594.9, 5168.8, 3742.8, 2317.6, 892.1,
        ]
        y16 = [yc - (max(y16) + min(y16)) / 2 for yc in y16]
        registry.register(FarmLayout(
            name="16T",
            num_turbines=16,
            xcoords=x16,
            ycoords=y16,
            metadata={"source": "Bizon Monroc et al. 2023"},
        ))

        # === 32 turbines ===
        x32 = [
            891.5, 891.5, 891.5, 2088.6, 2089.1, 2088.2, 3285.7, 3285.3,
            3285.3, 3285.3, 3285.7, 4482.4, 4482.4, 4482.4, 4482.8, 4481.9,
            5679.5, 5679.9, 5679.1, 5679.0, 5679.5, 6876.2, 6876.2, 6876.1,
            6876.6, 6876.6, 8073.2, 8073.2, 8073.7, 8072.8, 8073.3, 9270.8,
        ]
        x32 = [xc - min(x32) for xc in x32]
        y32 = [
            5169.3, 3743.2, 2317.2, 5168.7, 3743.5, 2318.0, 6594.1, 5169.4,
            3743.4, 2317.3, 892.2, 6594.9, 5168.8, 3742.8, 2317.6, 892.1,
            6594.2, 5169.1, 3743.5, 2317.5, 892.3, 6595.0, 5169.0, 3742.9,
            2317.7, 891.7, 6594.4, 5168.3, 3743.2, 2317.6, 892.4, 6594.6,
        ]
        y32 = [yc - (max(y32) + min(y32)) / 2 for yc in y32]
        registry.register(FarmLayout(
            name="32T",
            num_turbines=32,
            xcoords=x32,
            ycoords=y32,
            metadata={"source": "Bizon Monroc et al. 2023"},
        ))

        # === TCRWP (Total Control Reference Wind Power Plant) ===
        # https://farmconners.readthedocs.io/en/latest/provided_data_sets.html
        x_tcrwp = [(i // 4) * 126 * 4 + int(i % 2 == 0) * 126 * 2 for i in range(32)]
        x_tcrwp = [xc - min(x_tcrwp) for xc in x_tcrwp]
        y_tcrwp = [-300 + (i % 4) * 126 * 4 for i in range(32)]
        y_tcrwp = [yc - (max(y_tcrwp) + min(y_tcrwp)) / 2 for yc in y_tcrwp]
        registry.register(FarmLayout(
            name="TCRWP",
            num_turbines=32,
            xcoords=x_tcrwp,
            ycoords=y_tcrwp,
            metadata={"source": "Total Control Reference Wind Power Plant"},
        ))

        # === Ablaincourt ===
        # T. Duc, O. Coupiac, N. Girard, G. Giebel, and T. Göçmen,
        # "Local turbulence parameterization improves the Jensen wake model"
        registry.register(FarmLayout(
            name="Ablaincourt",
            num_turbines=7,
            xcoords=[484.8, 797.1, 1038.8, 1377.6, 1716.9, 2057.3, 2400.0],
            ycoords=[274.0, 251.0, 66.9, -22.7, -112.5, -195.3, -259.0],
            metadata={"source": "Duc et al."},
        ))

        # === Horns Rev 1 ===
        x_hr1 = [
            978.66429328, 1046.94882783, 1114.77605221,
            1182.63478828, 1250.7064236, 1318.35226044, 1386.42389576,
            1454.70843032, 1536.12485835, 1604.86349156, 1673.35516693,
            1741.84684231, 1810.55337238, 1879.01294462, 1947.93432938,
            2016.4581079, 2095.73045319, 2164.43698326, 2232.96076178,
            2301.66729185, 2369.94411253, 2438.8654973, 2507.35717267,
            2576.06370274, 2657.48013077, 2725.88455072, 2794.28897067,
            2862.26554207, 2930.638141, 2998.82863667, 3067.01913235,
            3135.66929759, 3214.94164288, 3283.37788384, 3351.5365585,
            3419.94097845, 3488.3453984, 3556.32196979, 3624.72638974,
            3693.13080969, 3774.54629069, 3842.83000239, 3911.04995735,
            3979.54778283, 4047.79961616, 4115.83733572, 4184.48551824,
            4252.73735157, 4334.1537796, 4402.77309866, 4470.68734397,
            4539.09283338, 4607.22090834, 4675.59460536, 4743.96830237,
            4812.34199938, 4891.61434467, 4960.35297788, 5028.6619017,
            5097.36843177, 5166.28981654, 5234.56663722, 5302.99410631,
            5371.94759422, 5455.50810499, 5524.09563166, 5591.79604732,
            5660.20153672, 5728.54344134, 5796.7033087, 5865.10879811,
            5933.69632477, 6012.9696171, 6081.19193378, 6149.56453272,
            6217.60474614, 6286.00916609, 6354.13601973, 6423.00010925,
            6491.15878391,
        ]
        x_hr1 = [xc - min(x_hr1) for xc in x_hr1]
        y_hr1 = [
            5447.12743106, 4888.84439828, 4334.30025777,
            3779.49848277, 3222.95607889, 2669.89493278, 2113.3525289,
            1555.06949612, 5445.55762914, 4888.83332877, 4334.10917624,
            3779.38502372, 3222.9207314, 2668.45658693, 2110.25215482,
            1555.26799424, 5445.75536963, 4889.29107731, 4334.30691673,
            3777.84262442, 3224.85861169, 2666.65417958, 2111.93002705,
            1555.46573474, 5445.95386775, 4889.4522232, 4332.95057866,
            3779.92968031, 3223.68691446, 2668.92564301, 2114.16437157,
            1555.66347523, 5446.15085062, 4889.39032738, 4334.88793462,
            3778.38629008, 3221.88464553, 2668.86374719, 2112.36210264,
            1555.8604581, 5444.58180632, 4889.56895881, 4335.07432872,
            3778.32115554, 3223.56741674, 2670.55400361, 2112.57872218,
            1557.82498337, 5448.31311639, 4889.80853138, 4337.04266434,
            3780.2784761, 3225.7722123, 2669.26678789, 2112.76136349,
            1556.25593908, 5446.74331447, 4890.0190141, 4336.77499331,
            3780.31070099, 3222.10626889, 2669.12225615, 2114.91811974,
            1556.45367957, 5446.94257021, 4888.69674904, 4337.67127877,
            3780.90709053, 3224.66042996, 2669.89540232, 2113.13121408,
            1554.88539291, 5447.13955308, 4892.11940294, 4335.87663709,
            3782.33798136, 3225.83633681, 2671.59282275, 2111.35155331,
            1556.84916056,
        ]
        y_hr1 = [yc - (max(y_hr1) + min(y_hr1)) / 2 for yc in y_hr1]
        registry.register(FarmLayout(
            name="HornsRev1",
            num_turbines=80,
            xcoords=x_hr1,
            ycoords=y_hr1,
            metadata={"source": "Horns Rev 1 offshore wind farm"},
        ))

        # === Horns Rev 2 ===
        x_hr2 = [
            3586.690071, 3038.283676, 2502.705501, 1957.5061600000001,
            1421.927985, 873.52159, 341.150469, 3548.2054120000003,
            3003.0060719999997, 2457.8067309999997, 1922.228556, 1380.236271,
            831.829876, 286.630535, 3551.412467, 3022.2484010000003,
            2473.842006, 1928.642666, 1383.443326, 838.2439860000001, 293.044645,
            3609.139456, 3063.9401159999998, 2531.568995, 1986.369655,
            1444.37737, 908.7991939999999, 363.599854, 3708.5581589999997,
            3163.358819, 2627.780643, 2101.823633, 1563.038403, 1017.839062,
            488.674997, 3849.668576, 3310.8833459999996, 2778.5122260000003,
            2249.34816, 1720.184095, 1191.020029, 671.477129, 4019.6424879999995,
            3496.892533, 2977.349632, 2454.599676, 1938.263831, 1428.3420950000002,
            905.592139, 4240.929279, 3731.007544, 3237.1210819999997,
            2727.199347, 2214.0705559999997, 1704.14882, 1197.434139,
            4513.5289490000005, 4013.228378, 3516.134862, 3028.662511,
            2534.77605, 2037.682534, 1550.210183, 4824.613279, 4340.347983,
            3856.082686, 3378.2315, 2903.587368, 2432.1502920000003,
            1954.299105, 5170.975213000001, 4702.745191, 4253.757498999999,
            3788.734532, 3333.332731, 2868.309764, 2403.2867969999998,
            5562.235916, 5119.662334, 4673.881697, 4231.308115, 3798.355697,
            3355.782115, 2910.0014779999997, 5988.774223, 5565.442971,
            5138.904663, 4725.194576, 4305.070378, 3881.739126, 3455.200818,
        ]
        y_hr2 = [
            1990.3730449999998, 1945.447252, 1897.3124750000002,
            1839.550742, 1794.62495, 1749.6991580000001, 1701.56438,
            2667.468913, 2657.841957, 2648.2150020000004, 2622.543121,
            2616.12515, 2612.916165, 2600.080225, 3376.6546329999996,
            3383.072603, 3411.95347, 3440.834336, 3456.8792620000004,
            3488.969114, 3492.178099, 4056.959487, 4105.094264, 4172.482952,
            4239.871641, 4288.006418, 4355.395106, 4403.5298840000005,
            4740.473325, 4846.369836, 4933.012435000001, 5016.446049,
            5125.551544, 5212.194144, 5311.672684, 5427.196149, 5552.34657,
            5674.288006000001, 5809.065383, 5934.2158039999995,
            6056.1572400000005, 6178.0986760000005, 6088.247092, 6245.487365,
            6412.354593, 6582.430805999999, 6742.880064, 6906.538307,
            7063.77858, 6755.716005, 6945.046128999999, 7140.794224,
            7317.288407999999, 7519.454473, 7708.784597, 7907.741677,
            7391.095066, 7609.306057000001, 7837.144002999999, 8074.608905,
            8286.401925, 8520.657842, 8751.704773, 7994.384276, 8270.357,
            8520.657842, 8787.00361, 9034.095467000001, 9274.769354,
            9550.742078000001, 8597.673486, 8889.691135, 9168.872844,
            9451.263538, 9743.281186999999, 10028.88087, 10314.48055,
            9181.708784999999, 9483.353389, 9804.251905, 10131.56839,
            10436.421980000001, 10763.73847, 11084.636980000001,
            9720.818291, 10064.1797, 10407.54112, 10750.90253, 11094.263939999999,
            11453.67028, 11800.240670000001,
        ]
        y_hr2 = [yc - (max(y_hr2) + min(y_hr2)) / 2 for yc in y_hr2]
        registry.register(FarmLayout(
            name="HornsRev2",
            num_turbines=len(x_hr2),
            xcoords=x_hr2,
            ycoords=y_hr2,
            metadata={"source": "Horns Rev 2 offshore wind farm"},
        ))

        # === Ormonde ===
        x_orm = [
            4586.23169601483, 4207.909022918064, 3822.671339711839,
            3439.185657229091, 3061.6924881018426, 2676.475469958829,
            2295.5609596374716, 1915.5069508804402, 3868.77479147359,
            3479.8198869796765, 3094.605331044565, 2707.6286696999778,
            2315.2792370310417, 1931.7403314094568, 1544.76367006487,
            3525.49397590361, 3133.125437405249, 2740.608840334446,
            2350.7386279644306, 1957.3645790887745, 1564.1385887855513,
            1172.5534728058212, 786.11306765524, 2804.60426320667,
            2408.246264201987, 2009.1108340703724, 1611.699404782204,
            1214.816882330398, 819.5326086704703, 422.23540315107,
        ]
        y_orm = [
            1672.9962825278799, 2045.790730432492, 2425.399143133105,
            2803.2811561519584, 3175.258221181207, 3554.846270786033,
            3930.194682795655, 4304.69516728624, 1349.4126394052,
            1718.7113456882025, 2084.458725409041, 2451.879160901989,
            2824.400849718425, 3188.557259674342, 3555.9776951672898,
            650.60966542751, 1010.7905655699709, 1371.1073784186349,
            1728.9949006129034, 2090.098824895329, 2451.066836471561,
            2810.5285815333623, 3165.26765799256, 327.02602230483,
            677.2402547409571, 1029.9085713956556, 1381.0535943774057,
            1731.7312855475302, 2080.9967951159333, 2432.04089219331,
        ]
        registry.register(FarmLayout(
            name="Ormonde",
            num_turbines=len(x_orm),
            xcoords=x_orm,
            ycoords=y_orm,
            metadata={"source": "Ormonde offshore wind farm"},
        ))

        # === WMR ===
        x_wmr = [
            3378.3280896, 4347.000730399999, 5287.63824580002,
            6245.1108866000095, 7196.42677700001, 8167.2210428,
            3900.5954734, 4871.9464891999805, 5812.30562940001,
            7721.61578559999, 8675.33167620002, 4425.78448199999,
            5359.743622199991, 8214.491293600011, 9173.842309599999,
            4924.781614799999, 5879.854255599999, 8733.28030220001,
            9684.317817600011, 5464.9273735999905, 6427.2000144,
            7378.794280199991, 9287.30443639999, 10234.620326,
            5977.03800720001, 6904.0755223999895, 7895.70491359999,
            9810.09344499998, 10763.809336, 6505.9486406, 7463.42128139998,
            8404.5804216, 9378.609812800001, 10340.325703999999, 11288.47672,
        ]
        y_wmr = [
            6514.8637897997005, 7132.538836799939, 7762.2424016000205,
            8388.74596620019, 9021.40187620011, 9628.95309560009,
            5724.1606004003, 6347.21651039973, 6971.5962475998,
            8218.97485920013, 8849.23076920029, 4993.07617259966,
            5589.1227016002795, 7463.8048780002, 8094.06078799964,
            4167.17298319965, 4786.1718573998105, 6654.45403379988,
            7282.833771200049, 3326.01275800027, 3950.6686680000203,
            4584.64840519987, 5819.22701679977, 6468.93058160029,
            2533.2427767999598, 3143.1939961997, 3786.74521579973,
            5030.92382740013, 5661.1797374003, 1733.46341459983,
            2366.39549720015, 2997.97523460024, 3614.85028140031,
            4249.10619139984, 4870.53358340008,
        ]
        registry.register(FarmLayout(
            name="WMR",
            num_turbines=len(x_wmr),
            xcoords=x_wmr,
            ycoords=y_wmr,
            metadata={"source": "WMR wind farm"},
        ))

        # === Dafeng H1 (Goldwind 8.5MW) ===
        x_df = [
            6.206, 939.84, 1955.816, 2995.663, 4002.738, 5434.417,
            6387.486, 0.0, 17.343, 267.999, 1128.129, 2282.239,
            3436.35, 4590.461, 5744.571, 6612.689, 6754.058, 518.655,
            1544.285, 2569.916, 3595.546, 4621.176, 5646.806, 7051.284,
        ]
        y_df = [
            2162.469, 2158.897, 2158.263, 2156.617, 2156.044, 2153.971,
            2153.41, 1103.544, 44.08, -1015.383, 45.825, 47.57,
            49.314, 51.059, 52.804, 753.797, -644.7, -2074.846,
            -2069.661, -2064.475, -2059.289, -2054.103, -2048.917, -2162.469,
        ]
        registry.register(FarmLayout(
            name="DafengH1",
            num_turbines=24,
            xcoords=x_df,
            ycoords=y_df,
            turbine_type="goldwind_8_5mw",
            metadata={"source": "Dafeng H1 Project, Goldwind 8.5MW"},
        ))

        # === Generic row layouts (dynamic) ===
        # 这些按需生成：注册特殊的 callable 名称，在 get() 中延迟构造。
        registry._row_cache: Dict[str, FarmLayout] = {}

        return registry

    def get_row_layout(self, num_turbines: int) -> FarmLayout:
        """每行 N 台风机、Y=0 的通用行布局（间距 4D = 504m）。"""
        name = f"Row{num_turbines}T"
        if name not in self._layouts:
            self._layouts[name] = FarmLayout(
                name=name,
                num_turbines=num_turbines,
                xcoords=[i * 504.0 for i in range(num_turbines)],
                ycoords=[0.0 for _ in range(num_turbines)],
                metadata={"source": "generic row layout, 4D spacing"},
            )
        return self._layouts[name]

    @classmethod
    def from_yaml_directory(cls, path: str) -> "LayoutRegistry":
        """
        从 YAML 文件目录加载布局（每文件一个布局）。
        文件名（不含扩展名）用作布局名称。

        期待 YAML 格式：
        ```yaml
        num_turbines: 80
        turbine_type: nrel_5MW
        xcoords: [978.66, 1046.95, ...]
        ycoords: [5447.13, 4888.84, ...]
        metadata:
          source: "Horns Rev 1"
        ```

        Parameters
        ----------
        path : str
            包含 YAML 布局文件的目录路径。

        Returns
        -------
        LayoutRegistry
        """
        import yaml

        registry = cls()
        dir_path = Path(path)
        if not dir_path.is_dir():
            raise NotADirectoryError(f"Layout directory not found: {path}")

        for yaml_file in sorted(dir_path.glob("*.yaml")):
            with open(yaml_file, "r") as f:
                data = yaml.safe_load(f)
            name = data.get("name", yaml_file.stem)
            layout = FarmLayout.from_dict({**data, "name": name})
            registry.register(layout)

        return registry


# =========================================================================
# 快捷函数
# =========================================================================

# 全局内置注册表单例
_BUILTIN_REGISTRY: Optional[LayoutRegistry] = None


def get_builtin_registry() -> LayoutRegistry:
    """获取（或惰性创建）全局内置布局注册表。"""
    global _BUILTIN_REGISTRY
    if _BUILTIN_REGISTRY is None:
        _BUILTIN_REGISTRY = LayoutRegistry.from_builtin()
    return _BUILTIN_REGISTRY


def list_builtin_layouts() -> List[str]:
    """列出所有内置布局名称。"""
    return get_builtin_registry().list_names()


def get_layout(name: str) -> FarmLayout:
    """
    从内置注册表获取布局。

    便捷函数，等价于 ``LayoutRegistry.from_builtin().get(name)``。

    Raises
    ------
    KeyError
        未找到时抛出。
    """
    return get_builtin_registry().get(name)
