# Loongson 2K300/301 Smart Car 7.8 Baseline

这是基于龙芯 2K300/301 智能车工程整理出的 7.8 基线版本。当前版本重点不是堆叠多套算法，而是保留 7.8 主扫线链路，在视觉路径层接管环岛和十字，最后仍然输出统一的 `ImageStatus.Det_True` 给电机差速控制。

## 当前状态

- 主线视觉：二值化、扫线、边界处理、中心线生成、滤波、动态前瞻偏差。
- 特殊元素：环岛和十字只作为视觉路径接管，不直接改 PID、基速或差速参数。
- 电机控制：继续消费统一后的 `ImageStatus.Det_True - ImageStatus.MiddleLine`。
- 硬件修正：右轮前进方向已按当前车况反转。
- 最新基线提交：`d4136a2 整合环岛十字路径接管并修正右轮方向`。

## 目录结构

```text
.
├── Loongson_2K300_301_LIB/
│   ├── libraries/app/
│   │   ├── image.cpp      # 视觉主流程、环岛/十字识别与路径接管
│   │   ├── image.hpp      # 图像状态、道路类型、全局接口
│   │   └── motor.cpp      # 电机差速、速度 PID、方向极性
│   ├── main/
│   │   ├── build.sh       # 编译、上传、可选运行脚本
│   │   └── main.cpp
│   ├── example/
│   ├── libraries/
│   └── tools/
├── Doc/
├── MD_Image/
└── README.md
```

## 视觉算法思路

当前推荐方案是：

```text
二值化
-> 边界扫线
-> 元素识别
-> 环岛/十字路径接管
-> RouteFilter
-> GetDet
-> 电机差速控制
```

正常道路仍然走 7.8 原来的扫线主链路。环岛和十字不新增独立中心线输出，也不形成另一套视觉算法。

### 环岛接管

环岛识别依赖现有的 `LeftCirque` / `RightCirque` 状态和环岛阶段机。

接管函数：

- `Element_Handle_Left_Rings()`
- `Element_Handle_Right_Rings()`

它们负责补边界、生成适合绕环的中心线，并写回同一份 `ImageDeal[]`。后续 `RouteFilter()` 和 `GetDet()` 不需要知道这是普通道路还是环岛，只处理已经统一过的路径。

### 十字接管

十字识别使用 `Cross` / `Cross_ture` 状态，触发条件比较保守：底部需要有稳定边界，前方出现连续无边、大白区或有效边界明显减少时才进入十字状态。

接管函数：

- `Element_Judgment_Cross()`
- `Element_Handle_Cross()`

十字接管会在大白区内补连续中心线，避免 `Det_True` 因丢边而跳变。

## 电机控制思路

电机侧不区分正常道路、环岛、十字三套控制，只看统一后的视觉偏差：

```cpp
turn_error = ImageStatus.Det_True - ImageStatus.MiddleLine;
```

随后由 `Motor_Diff_Pid1()` 计算差速：

```cpp
Diff_SpeedL_expect = current_base_speed + turn_output;
Diff_SpeedR_expect = current_base_speed - turn_output;
```

再由左右轮速度 PID 输出 PWM。这样特殊元素只负责把路画对，电机只负责追这条路，调试时不会出现多套 PID 互相影响。

## 编译

工程推荐在 WSL 或 Linux 环境中编译。进入 `main` 目录后执行：

```bash
cd "Loongson_2K300_301_LIB/main"
bash ./build.sh
```

编译成功后，可执行文件位于：

```text
Loongson_2K300_301_LIB/main/build/main
```

## 上传到开发板

只编译并上传，不自动运行：

```bash
bash ./build.sh <开发板IP>
```

例如：

```bash
bash ./build.sh 192.168.31.187
```

脚本会把程序上传到：

```text
/home/root/main
```

如果需要上传后直接运行，再显式加 `-r`：

```bash
bash ./build.sh <开发板IP> -r
```

默认建议先只上传，不自动运行，方便上车前确认状态。

## 当前关键改动

### 1. 环岛/十字路径接管

`ImageProcess()` 的视觉链路已调整为：

```cpp
Element_Test();
DrawExtensionLine();
Element_Handle();
RouteFilter();
GetDet();
```

`Element_Handle()` 当前只开放环岛和十字接管，避免其它复杂元素继续膨胀。

### 2. 单一偏差输出

不新增“正常 Det / 环岛 Det / 十字 Det”三套结果。所有路径接管最终都写回 `ImageDeal[]`，再由 `GetDet()` 输出唯一的 `ImageStatus.Det_True`。

### 3. 右轮方向修正

当前车况下右轮方向已反转：

```cpp
constexpr bool kRightForwardDir = true;
```

位置：

```text
Loongson_2K300_301_LIB/libraries/app/motor.cpp
```

## 调试建议

- 路径问题先看 `ImageDeal[].Center` 和 `ImageStatus.Det_True`，不要先改电机 PID。
- 环岛和十字如果跑偏，优先调整视觉接管的补线条件和中心线连续性。
- 电机侧保持一套差速逻辑，避免正常、环岛、十字各自维护一套速度策略。
- 上传测试时默认不加 `-r`，确认板端环境后再运行。

## 仓库地址

```text
https://github.com/Bianka5441/Loongson_2K300_301_LIB_7_8
```
