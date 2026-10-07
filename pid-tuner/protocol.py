"""Protocol for the firmware: 29 base CSV integers + 14 raw reflectance = 43."""
import math
import re

CHANNELS = [
    ("偏差", "", .01), ("目标角速度", "°/s", .01),
    ("实际角速度", "°/s", .01), ("角速度误差", "°/s", .01),
    ("差速命令", "", 1), ("目标速度", "脉冲/采样", 1),
    ("左轮速度", "脉冲/采样", 1), ("右轮速度", "脉冲/采样", 1),
    ("基础 PWM 命令", "", 1), ("左轮 PWM 命令", "", 1),
    ("右轮 PWM 命令", "", 1), ("运行标志", "", 1),
    ("累计角度", "°", .1), ("转弯计数", "", 1),
    ("校准模式", "", 1),
] + [(f"传感器 {i+1}", "1=白 / 0=黑", 1) for i in range(14)] + \
  [(f"反射率 {i+1}", "0..10 低=黑", .01) for i in range(14)]

PARAMS = [
    dict(name="KP_L", label="比例 P", group="速度环 · 两轮共用", kind="float", default=25),
    dict(name="KI_L", label="积分 I", group="速度环 · 两轮共用", kind="float", default=.5),
    dict(name="KD_L", label="微分 D", group="速度环 · 两轮共用", kind="float", default=0),
    dict(name="P_A", label="比例 P", group="角速度环", kind="float", default=15),
    dict(name="I_A", label="积分 I", group="角速度环", kind="float", default=.5),
    dict(name="P_tobeA", label="比例 P", group="方向外环", kind="float", default=200),
    dict(name="D_tobeA", label="微分 D", group="方向外环", kind="float", default=0),
    dict(name="base_speed", label="巡航速度", group="速度与限幅", kind="int", default=40),
    dict(name="pid_limit", label="PID 增量上限", group="速度与限幅", kind="int", default=1000),
    dict(name="pwm_limit", label="PWM 命令上限", group="速度与限幅", kind="int", default=9300),
    dict(name="dead_zone", label="电机死区补偿", group="速度与限幅", kind="int", default=700),
    dict(name="duty_limit", label="差速命令上限", group="速度与限幅", kind="int", default=18000),
]
SENSOR_WEIGHT_DEFAULTS = [6.5,5,4,3,2,1,.1,.1,1,2,3,4,5,6.5]
PARAMS += [dict(name=f'weight{i+1}', label=f'S{i+1:02d}', group='14 路传感器权重',
                kind='float', default=value, sensor=i, min=0, max=100)
           for i,value in enumerate(SENSOR_WEIGHT_DEFAULTS)]
# Existing firmware exposes 20 stored slots but the disabled lookup scans five.
TURN_TABLE_INFO = dict(slots=20, lookup_slots=5, switch_parameter='turn_enable',
    boot_enabled=False, status_source='Read turn_enable from vehicle; absent on legacy firmware.')
TURN_DEFAULTS = [(7,-75,100),(8,-25,100),(11,20,600),(14,20,10)]+[(0,0,0)]*16
PARAMS += [dict(name='entry_ms',label='进弯确认时间',group='转向表',kind='int',
                default=10,min=1,max=60000,turn_table=True)]
for _slot, (_turn,_angle,_exit) in enumerate(TURN_DEFAULTS,1):
    for _prefix,_kind,_value,_min,_max in (
        ('turn','int',_turn,0,1000000),('angle','float',_angle,-180,180),('exit','int',_exit,0,60000)):
        PARAMS.append(dict(name=f'{_prefix}{_slot}', label=_prefix, group='转向表',
            kind=_kind,default=_value,min=_min,max=_max,turn_table=True,slot=_slot))
PARAM_MAP = {p['name'].lower(): p for p in PARAMS}
PARAM_MAP['turn_enable'] = dict(name='turn_enable',kind='int',group='转向表开关',min=0,max=1)
NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
ACK = re.compile(r"^(?:[-+0-9,]+)?\[OK\]\s+(\w+)\s*=\s*(" + NUMBER + r")\s*$")
VALUE = re.compile(r"^(\w+)\s*=\s*(" + NUMBER + r")\s*(?://.*)?$")
QUERY_NAMES = [p['name'] for p in PARAM_MAP.values()] + ['KP_R','KI_R','KD_R','run','speed','max_time']
# The car's legacy TX can leave numeric CSV bytes before a parameter reply.
# Recover only exact known parameter names, not arbitrary suffixes.
KNOWN_VALUE = re.compile(r"^(?:[-+0-9,]+)?(" + '|'.join(map(re.escape,QUERY_NAMES)) + r")\s*=\s*(" + NUMBER + r")\s*(?://.*)?$")

def parse_line(line):
    line = line.strip()
    if len(line) > 512:
        return dict(type="message", text="忽略超长串口行")
    fields = line.split(',')
    if len(fields) == 43 and all(re.fullmatch(r"[-+]?\d+", v.strip()) for v in fields):
        raw = list(map(int, fields))
        if any(abs(v) > 2147483648 for v in raw):
            return dict(type="message", text="忽略超出 int32 范围的数据帧")
        return dict(type="frame", raw=raw, values=[v*c[2] for v,c in zip(raw,CHANNELS)])
    match = ACK.fullmatch(line)
    if match:
        value = float(match[2])
        if math.isfinite(value):
            return dict(type="ack", name=match[1], value=value)
    match = KNOWN_VALUE.fullmatch(line) or VALUE.fullmatch(line)
    if match:
        value = float(match[2])
        if math.isfinite(value):
            return dict(type="param", name=match[1], value=value)
    if line:
        return dict(type="message", text=line)
    return None

class Decoder:
    def __init__(self):
        self.buffer = bytearray()
    def feed(self, data):
        self.buffer.extend(data)
        result = []
        while b'\n' in self.buffer:
            line, _, rest = self.buffer.partition(b'\n')
            self.buffer = bytearray(rest)
            # UART replies can interrupt a telemetry line. Never accept a partial frame.
            item = parse_line(line.decode('ascii', 'replace'))
            if item:
                result.append(item)
        if len(self.buffer) > 8192:
            self.buffer.clear()
            result.append(dict(type="message", text="接收行过长，已重新同步"))
        return result

def validate_parameter(name, value):
    spec = PARAM_MAP.get(str(name).lower())
    if not spec:
        raise ValueError("未知参数；这里只允许修改界面列出的参数")
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError("参数必须是有效数字") from None
    if not math.isfinite(number) or abs(number) > 1000000:
        raise ValueError("参数必须有限且绝对值不超过 1000000")
    if spec['kind'] == 'int' and not number.is_integer():
        raise ValueError("这个参数必须是整数")
    if 'sensor' in spec and not 0 <= number <= 100:
        raise ValueError("传感器权重幅值必须在 0 到 100 之间，方向由左右位置决定")
    if spec['name']=='turn_enable' and number not in (0,1):
        raise ValueError('转向表开关只允许 0（关闭）或 1（开启）')
    if spec.get('turn_table') and not spec['min'] <= number <= spec['max']:
        raise ValueError(f"{spec['name']} 必须在 {spec['min']} 到 {spec['max']} 之间")
    if spec['group'] == '速度与限幅' and number < 0:
        raise ValueError("速度和限幅参数不能为负数")
    number = int(number) if spec['kind'] == 'int' else number
    return spec['name'], number, f"{spec['name']}={number:.8g}\n"
