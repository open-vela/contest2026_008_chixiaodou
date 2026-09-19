# openvela 桌面充电站

基于 **STM32H750B-DK** 与 **openvela** 的一体化焊接工作站：把多口快充、功率计量与智能烙铁整合到一块主控上，并以 AI 作为统一交互入口。

队伍：**赤小豆**（编号 008）　选题：**AI 硬件产品创新**

---

## 一、作品简介

在电子工程师、硬件爱好者与维修从业者的典型工位上，往往同时存在两类设备：为手机、平板、笔记本供电的多口充电器，以及电烙铁等焊接工具。当前使用中存在四个具体痛点：

1. **充电设备分散、状态不可见** —— 需要多个充电头配多根数据线，桌面凌乱；传统充电器只能以指示灯粗略表示"正在充电"，无法得知每个接口实时的电压、电流、功率与协商到的快充协议。
2. **焊接工具缺乏状态管理** —— 电烙铁长时间通电空烧存在安全隐患，用户离开工位后常常忘记断电，而传统焊台没有主动提醒机制。
3. **两类设备缺乏统一的交互入口** —— 用户需要在多个设备之间分别操作，无法用统一的自然语言入口同时查询与控制。
4. **焊接过程中双手被占用，调节温度不便** —— 实际焊接时通常一手持烙铁、一手持元件或焊丝，此时若想调整烙铁温度，必须先放下工具去操作面板。

本作品把上述两类设备整合到一块主控上。硬件以 STM32H750B-DK 为核心，外接 TCA9548A 八通道 I2C 多路复用器、SW3538 快充模块、INA226 功率监视器、MAX6675 热电偶转换模块与 JBC245 烙铁加热驱动电路，通过 Arduino 排针互连。

软件实现 LVGL 三页触控界面、快充寄存器驱动、功率计量与 PID 恒温控制；**自研软件 I2C 传输层**承载 TCA9548A，解决了硬件 I2C 已被触摸屏占用、且快充芯片地址固定（0x3C）无法共存两项约束。接入 openvela `ai_agent` 与云端大模型实现自然语言交互，并沉淀自定义 Skill 1 个。

**关键实测指标：** 固件 966700 B，占板载 2 MB Flash 的 46.10 %；AI 单轮对话端到端响应 2170 ms；公网连通性丢包 0 %、RTT 平均 55 ms；运行期空闲堆 706496 B。

---

## 二、选题方向

**AI 硬件产品创新**

作品的核心不在于把 AI 当作一个附加的语音助手，而是让 AI 成为设备的**统一交互入口**：充电状态的查询、焊接工艺参数的询问都通过自然语言完成，不再需要在多个设备与多级菜单之间切换。

---

## 三、目录结构

```text
app/charging_station/        # 本作品全部代码
├── charging_station_main.c  # 应用入口与调度（372 行）
├── charging_station_ui.c    # LVGL 三页界面与全部可变状态（1457 行）
├── sw3538.c                 # SW3538 快充芯片寄存器驱动（388 行）
├── ina226.c                 # INA226 功率计量驱动（424 行）
├── iron.c                   # 热电偶采样、PID 恒温控制与软件 PWM（565 行）
├── cs_i2c_sw.c              # 自研软件 I2C 传输层（241 行）
├── *.h                      # 对应头文件（共 818 行）
├── skills/
│   └── soldering-session.md # 自定义 Skill：焊接会话（94 行）
├── ui_preview.html          # 界面 1:1 HTML 还原预览
├── charging_station_logo.h  # 界面 logo 位图
└── Kconfig / Make.defs / Makefile / CMakeLists.txt   # 构建集成

logs/3181619934/             # AI Coding 对话日志，10 场会话
技术报告.md                   # 技术报告正文
contest2026_008_chixiaodou.xml  # 本仓 repo manifest
openvela.xml                 # 上游 openvela 全量 manifest
```

应用核心代码为 **6 个 .c 源文件、共 3447 行**（不含 openvela 上游既有代码）。

### 各源文件职责

| 文件 | 职责 |
| --- | --- |
| `charging_station_main.c` | 应用入口与调度。初始化 TCA9548A 上的 SW3538 / INA226 探测；主循环 100 ms 一拍，铁 PID 每拍执行、快充通道轮转读取（每 4 拍一轮）、INA226 每 5 拍读取、每 5 s 重探缺席设备 |
| `charging_station_ui.c` | LVGL 9.1 三页界面与**全部可变状态**。界面是用户可设置状态（目标温度、加热开关、PID 参数）的唯一持有者，主循环每 100 ms 反向读取后下发驱动层，构成单向数据流 |
| `sw3538.c` | 快充芯片寄存器驱动。写保护解锁（Reg0x10 依次写 0x20→0x40→0x80）、ADC 通道选择、12 位数据拼接、快充协议解析、NTC 温度换算、热插拔重探 |
| `ina226.c` | 功率监视器驱动。整数校准（CAL 公式）、缺席重探、读取失败降级 |
| `iron.c` | 热电偶位翻转 SPI 读取（MAX6675）、位置式 PID（条件积分抗饱和 + 微分斜率限幅）、20 档软件 PWM、休眠检测与三层失效保护 |
| `cs_i2c_sw.c` | 自研软件 I2C 传输层。复用 NuttX `i2c_bitbang` 引擎，把"选择 TCA9548A 通道"与"访问从机"封装进**同一把互斥锁**，保证事务在正确通道上完成 |

### 硬件构成

| 器件 | 型号 / 规格 | 数量 |
| --- | --- | --- |
| 主控 | STM32H750B-DK（Cortex-M7，2 MB Flash，8 MB SDRAM，480×272 LTDC 屏，板载 LAN8740A 以太网 PHY） | 1 |
| I2C 多路复用器 | TCA9548A，8 通道，地址 0x70 | 1 |
| 快充模块 | SW3538（地址固定 0x3C），**设计 4 路、实装 3 路** | 3 |
| 功率监视器 | INA226（0x40 整机输入 / 0x46 加热丝） | 2 |
| 热电偶转换 | MAX6675，12 位、分辨率 0.25 ℃、量程 0~+1024 ℃ | 1 |
| 烙铁 | JBC245 手柄与烙铁架 | 1 |
| 加热驱动 | NPN + MOSFET 模块 | 1 |

TCA9548A 通道分配：CH0~CH2 接 SW3538（CH3 因与板载网口结构干涉空出）、CH4/CH5 接 INA226。每路快充含 1×Type-C + 1×Type-A。

板级互连使用 **Arduino 排针**，引脚分配见 [`技术报告.md`](技术报告.md) 第 3.4.3 节（硬件设计与适配）。

---

## 四、运行方式

### 1. 拉取与编译

```bash
# 1) 拉取 openvela 全量源码 + 本仓
repo init -u https://github.com/open-vela/contest2026_008_chixiaodou \
          -b dev-ai-contest-2026 -m contest2026_008_chixiaodou.xml
repo sync -c -j8

# 2) 在 openvela 工作区根目录编译（即本仓的上一级）
cd ..
./build.sh stm32h750b-dk:lvgl -j8

# 需要改配置时
./build.sh stm32h750b-dk:lvgl menuconfig
```

本作品使用的 board config 是 `nuttx/boards/arm/stm32h7/stm32h750b-dk/configs/lvgl`。仓内 `app/charging_station` 已由 manifest 的 `<linkfile>` 软链到编译树的 `packages/demos/contest2026_008_charging_station`，**无需手动拷贝，上游仓库零改动**。

编译产物在 `nuttx/` 根目录：`nuttx.hex`（烧录用）、`nuttx.bin`、`nuttx`（ELF）。

### 2. ⚠️ 依赖公共仓改动，请先读这一节

本作品的板级与驱动改动按大赛要求放在公共仓，**不在本仓内**。这些改动**尚未合入上游 `dev-ai-contest-2026`**，已整理为独立分支提交至对应公共仓，等待组委会 review：

| 公共仓 | 分支 / 提交 | 文件数 | 改动内容 | 未合入的后果 |
| --- | --- | --- | --- | --- |
| `open-vela/nuttx` | `contest2026-008-ethernet-phy-poll`<br>`8b1602cf33d` | 6 | LAN8740A PHY 链路轮询；SDRAM bank2 与显存地址重叠修复；板级 `defconfig`；`src/Makefile` 与 `src/etc/` 启动脚本；ft5x06 触摸框架迁移 | 触摸与以太网失效、堆遍历崩溃、**应用不会被启动** |
| `open-vela/nuttx-apps` | `contest2026-008-netinit-carrier-poll`<br>`d0c56f46e` | 4 | 运行时 IPv4 策略 API（`netinit_set_ipv4_config()`）；`CONFIG_NETINIT_CARRIER_POLL` 载波轮询 | 网线热插拔后地址策略不会自动恢复 |
| `open-vela/packages_ai_agent` | `contest2026-008-chunked-body-and-monotonic-clock`<br>`6e2f65a` | 3 | 按块长遍历的 chunked 完成判定；改用 `CLOCK_MONOTONIC` 计时 | **AI 每轮对话被看门狗判超时（61910 ms），无法对话** |

**在这些改动合入上游之前，仅执行 `repo sync` 得到的 nuttx 树编译出的固件不会启动本作品** —— 上游 `configs/lvgl/defconfig` 中未启用 `CONFIG_LVX_USE_DEMO_CONTEST2026_008_CHARGING_STATION`，且 `boards/arm/stm32h7/stm32h750b-dk/src/etc/`（启动脚本所在目录）在上游不存在。

评委如需复现，请待上述 PR 合入后重新 `repo sync`。本作品的完整构建与上板过程、原始串口记录，均保留在 `logs/` 的会话日志中。

### 3. 烧录与运行

```bash
# 擦除 + 烧写 + 复位（ST-Link）
STM32_Programmer_CLI -c port=SWD mode=UR reset=HWrst freq=4000 -e all -w nuttx/nuttx.hex -v

# 串口控制台
minicom -D /dev/ttyACM0 -b 115200
```

**运行无需手工敲命令**：板级 `rcS` 启动脚本会自动拉起 `charging_station`。`rc.sysinit` 会挂载 procfs / tmpfs / devfs，并把 `/data` 挂在 RAM 盘上（本板无可持久化可写文件系统，AI Agent 的配置与会话状态因此掉电即失）。

### 4. 界面预览

`app/charging_station/ui_preview.html` 是界面的 1:1 HTML 还原（坐标、颜色、字号均按实现值），无需硬件即可在浏览器中查看界面布局。

---

## 五、AI Coding 使用说明

本作品全程使用 **Claude Code**（终端 CLI，含其 Skill、子代理与任务编排机制）辅助开发。

**使用规模**（统计口径为本项目开发会话，覆盖 25 天、累计 4909 次模型调用）：

| 项目 | 数量 |
| --- | --- |
| Token 使用总量 | 约 20.4 亿（输入约 1.96 亿 + 输出约 343 万，另计缓存读取约 18.4 亿） |
| AI Coding 代码占比 | 约 90 %（以自研源码为分母，不含 openvela 上游既有代码） |
| 文件写入 / 代码编辑 | 82 次文件写入、518 次代码编辑，覆盖 21 个自研文件 |
| 调用 Skill | 10 次（openvela-build 5 次，kconfig-tweak、nuttx-driver-development、submit-pr、contest-log-collector、update-config 各 1 次） |
| 新增沉淀 Skill | 1 个（`skills/soldering-session.md`） |

**协作方式：** 需求拆解与方案设计阶段用 AI 做技术选型对比（例如否决端侧推理模型的定量依据）；编码阶段由 AI 生成三个自研驱动（sw3538 / ina226 / iron）并同步补齐边界检查、失效保护与降级逻辑；调试阶段用 AI 解读串口日志、分析寄存器与引脚复用冲突；文档阶段由 AI 汇总数据成文。

**沉淀的 Skill** —— `skills/soldering-session.md`：焊接作业的偏好记忆与定时巡检，声明 `read_file` / `cron_add` / `cron_list` / `cron_remove` / `get_current_time` 五个工具。该 Skill **刻意不包含驱动加热器的命令**，温度只能通过触摸屏设置，AI 对执行器只读不写。

**对上游的反馈** —— 向 openvela 上游提交了 ft5x06 触摸驱动迁移修复（`open-vela/nuttx` PR #374），另整理出显存/堆地址规划、HTTP 流式解析、计时基准三项改进建议。

> 完整的对话日志见 [`logs/3181619934/`](logs/3181619934/)，共 10 场会话，覆盖从环境搭建、联网适配、驱动开发到报告撰写的全过程。

---

## 六、已知不足

以下内容如实标注，不作粉饰。

**开发板在验证阶段硬件调试中损坏**（属硬件故障，与本作品固件无关）。受此影响，以下测试项**未能完成**：

- 长时间连续运行（内存与帧率波动）
- 温控稳态精度
- 功率计量误差
- 多路快充的满载并发

功能测试表中，以下 4 项目前标注为**「未验证」**——驱动与逻辑均已实现，仅因开发板损坏而未完成上板验证：**多路快充状态读取、功率计量、烙铁温度闭环、休眠检测**。

其余在功能测试表中判定为**「通过」**的项目：系统启动与界面显示、触摸交互、DHCP 获取地址、以太网公网连通性、DNS 域名解析、网线热插拔、HTTPS/TLS 链路、AI 对话。

**其他限制：**

- **Skill 运行时加载未启用** —— 本版固件未划分可写用户分区，`/data` 无法作为可写目录挂载，Skill 的定义、工具链与框架支持均已就位，端到端效果留待具备可写存储后验证。该限制来自文件系统布局，与存储容量无关。
- **多媒体方向尚未涉及** —— 痛点 ④（焊接时双手被占用）当前以触摸屏提供温度调节入口，用户仍需放下工具操作；以语音直接调温是更彻底的解法，列为后续演进方向。
- **快充实测数据的适用口径** —— 报告中 SW3538 的电压 / 电流实测值（约 20.0 V、Type-A 口 3295 mA、协议识别为 PD）系**在单芯片直连 I2C 的早期调试阶段**测得；经 TCA9548A 的多路读取因开发板损坏未完成上板验证。

**后续工作：** 更换硬件后补齐上述量化测试项；为开发板扩展 SPI Flash 或 SD 卡等可写存储，启用 Skill 与长期记忆的持久化；扩展语音控制（语音唤醒 + TTS 播报，统一使用大赛指定唤醒词）；利用空闲的 Arduino 排针接入环境感知（温湿度、电流互感器等）。
