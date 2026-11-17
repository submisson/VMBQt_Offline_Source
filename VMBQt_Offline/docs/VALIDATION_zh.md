# 本次交付的验证记录

验证日期：2026-10-26。

## 环境

| 项目 | 本次环境 |
|---|---|
| 系统 | Ubuntu 24.04.3 LTS，Linux x86_64 |
| Python | 3.12.14 |
| Qt / PySide6 | 6.8.3 |
| NumPy | 2.2.6 |
| pyproj | 3.7.2 |
| PyYAML | 6.0.3 |
| pytest | 8.4.2 |
| 可选互操作库 | lanelet2 1.2.2、mgrs 1.5.0 |
| GUI 后端 | Qt offscreen；使用 QTest 实际鼠标/按键事件与窗口渲染 |

## 已执行

完整自动化测试 **41 passed，无跳过**。可重复命令：

```bash
python -m pip install -r requirements-dev.txt
python -m pip install lanelet2==1.2.2 mgrs==1.5.0
QT_QPA_PLATFORM=offscreen python -m pytest -q
```

普通安装没有两个可选库时，相关互操作项目会跳过。查看实际输出，不能把 SKIPPED 当通过。

测试覆盖的主要行为：

- 真实 Qt 鼠标点击画车道，Enter 完成，撤销与重做。
- 属性修改共享节点，2D/3D、透视、点云与底图绘制。
- 正交/透视视点下的坐标投影与反投影一致性。
- 共享边界、连接端点、拆分与合并、拓扑路径、Bézier 转弯连接。
- 失败事务回滚、已分配 ID 状态恢复、旧验证签名失效。
- 交通灯外框/灯泡/规则成员与复制 ID 重映射。
- right_of_way 的循环引用不会导致无限遍历；移动一条车道不沿路权引用移动另一条道路。
- 64 位范围大整数 ID、各 OSM 类型相同 ID、未知标签、XML 扩展保留。
- 缺失引用报告、删除引用保护、地图 ID 合并与比较。
- PCD ASCII、binary、binary_compressed、LZF 重叠回引、长度/越界检查、RGB、NaN、显示采样。
- CSV 附加列、停靠点和工程相对资源路径往返。
- LocalCartesianUTM / TransverseMercator 正反投影；MGRS 方格检查；拒绝静默高程换算。
- 使用真正的 Lanelet2 `loadRobust` 读取输出；在日本、挪威和斯瓦尔巴三个投影原点下，普通道路、邻道和中心线解析无错误，节点坐标与原模型差异小于 1e-5m。
- 使用独立 MGRS / GEOTRANS 实现比对东京、伦敦、纽约、悉尼、北京、挪威、斯瓦尔巴及南半球近赤道点的方格内坐标，与 1 米精度 MGRS 字符串的截断值一致。

此外完成：

1. `compileall` 检查所有 Python 模块。
2. `bash -n` 检查安装/启动脚本语法。
3. 构建 `vmbqt_offline-0.1.0-py3-none-any.whl`；它是本程序包，不含 Qt 等第三方依赖。
4. 打开合成演示工程，加载实际 PCD，渲染并查看 2D/3D 窗口截图，检查中文控件与布局。
5. 演示地图内置检查：0 ERROR、2 WARNING。提示对象是没有前驱/后继的独立邻道和横道，不作为真实道路已通过验证的证据。

## 未验证 / 明确不在本次完成范围

- Ubuntu 22.04 / Python 3.10 的实际桌面安装；依赖版本已按 Python 版本分支配置，但没有实机结果。
- X11/Wayland 中的全套人工鼠标交互、不同显卡、HiDPI、多屏幕与长期稳定性。
- 多 GB/数十亿点或超大 OSM 地图的性能、内存与压力测试。
- ROS2 环境下的外部 Autoware 验证器启动；目标版本需求集的全量检查。
- Autoware 扩展规则的加载执行、Planning Simulator、真实车辆或道路运行。
- FMS、账号、地图云端版本、在线卫星图、联机 Plan Route / Plan Path。
- 与当前 VMB 登录态版本逐功能、逐异常和逐像素的对等验收。

因此这里能确认的是：**提供了可运行、通过所列测试的本地编辑器工程；不能据此宣称已经完成网页版全部功能或生产级 Autoware 验收。**
