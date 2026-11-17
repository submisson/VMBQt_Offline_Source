# 公开来源与兼容性约定

查询日期：2026-10-26。以下是用于确定格式、工具范围和部署方式的第一方公开资料。它们是参考资料，不是本工程依赖文件；本包没有网页源码或 TIER IV 的授权服务端实现。

| 来源 | 用途 |
|---|---|
| [Autoware：Creating a vector map](https://docs.autoware.org/main/tutorials/integrating-autoware/creating-maps/creating-vector-map/) | VMB 是地图创建工具、需要账号、核心地图元素 |
| [Autoware：Creating a Lanelet](https://docs.autoware.org/1.8.0/tutorials/integrating-autoware/creating-maps/creating-vector-map/lanelet2/) | 点云底图、宽度、连接、限速和导出流程 |
| [VMB：画面構成](https://docs.web.auto/user-manuals/vector-map-builder/screen-layout) | 2D/3D、图元编辑、文件、轨迹、比较、FMS 与联机规划的功能范围 |
| [VMB：Introduction](https://docs.web.auto/user-manuals/vector-map-builder/introduction) | node/way/relation、坐标标签、信号灯/灯泡的结构 |
| [Autoware Lanelet2 扩展格式](https://github.com/autowarefoundation/autoware_lanelet2_extension/blob/main/autoware_lanelet2_extension/docs/lanelet2_format_extension.md) | 交通灯、横道、检测区域、路权和其他扩展规则 |
| [Autoware Map Projection Loader](https://github.com/autowarefoundation/autoware_core/blob/main/map/autoware_map_projection_loader/README.md) | map_projector_info.yaml、投影类型、原点、高程基准 |
| [Autoware Lanelet2 Map Validator](https://github.com/tier4/autoware_lanelet2_map_validator) | 外部验证入口和命令格式 |
| [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) | 基础模型和实际 OSM 读取互操作 |
| [Qt for Python：Getting started](https://doc.qt.io/qtforpython-6/quickstart.html) | PySide6 官方绑定和 wheel 安装 |
| [Qt Linux requirements](https://doc.qt.io/qt-6/linux-requirements.html) | Ubuntu 图形系统库 |

本项目采取以下明确约定：

1. `local_x`、`local_y` 和 `ele` 是编辑器的坐标输入；如果没有本地 XY，使用用户显式指定的地理投影。不会根据第一个点猜原点。
2. 写出时总是写本地 XY/ele，并按声明投影计算 lat/lon；Local 写 lat/lon=0。真实使用的 Autoware 加载器必须采用匹配的 MapProjectorInfo；通用 OSM 编辑器并不一定按本地标签加载。
3. OSM ID 用 Python 整数保存，没有经过 JavaScript double；同一整数在 node/way/relation 中是不同身份。新增对象使用当前模型未占用的正整数。
4. 共享节点通过 ID 引用，几何修改可影响所有拥有该节点的 Way。规则对其他 Lanelet 的引用不被当成几何所有权：移动/复制一条道路不会沿 right_of_way 引用递归移动/复制另一条道路；复制后的规则仍可引用未选中的原车道。
5. 未知标签、OSM 关系与扩展 XML 不自动删除；XML 格式/注释/顺序不是逐字节保真。复制/合并只重映射标准引用和已知 `traffic_light_id` / `intersection_area` 标签，不能识别所有自定义标签里隐含的 ID。使用私有扩展时要检查这些字段。
6. 默认模板只生成可编辑的初始几何和关联关系，不证明交通规则适合真实道路、ODD 或特定 Autoware 发布版本。
7. 内置检查和 pip Lanelet2 基础读取通过，不意味着 Autoware 扩展模块已经验证。高级规则需要目标版本的扩展库和官方需求集检查。
8. PCD 显示样本与原始点云明确分开。PNG 是视图；`.vmbqt` 保存引用；地图目录导出复制原始点云。没有把采样显示数据当完整测绘数据导出。
9. 软件 3D 点云有逐像素深度，矢量始终叠加显示；不声称有完整场景深度遮挡、碰撞检测、光照、GPU LOD 或车辆运动学规划。
10. 本工程不调用 TIER IV 云服务、不接收账号密码、不包含 FMS 模拟成功接口，也没有假装存在的 ROS2 规划请求。

兼容性最终应由目标 Autoware 版本、投影配置、官方验证器和 Planning Simulator 实际验收确定。
