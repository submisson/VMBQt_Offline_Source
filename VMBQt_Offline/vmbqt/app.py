"""Qt desktop shell. No browser, credentials, remote API, or ROS dependency."""
from __future__ import annotations

import copy
import difflib
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import traceback
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from PySide6.QtCore import Qt, Signal, QThread, QTimer, QStandardPaths, QProcess
from PySide6.QtGui import QAction, QActionGroup, QKeySequence, QImage
from PySide6.QtWidgets import (QApplication, QMainWindow, QDockWidget, QWidget,
    QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QLineEdit, QComboBox,
    QCheckBox, QFormLayout, QTabWidget, QTreeWidget, QTreeWidgetItem,
    QAbstractItemView, QToolBar, QFileDialog, QMessageBox, QPlainTextEdit,
    QDialog, QDialogButtonBox, QSplitter, QListWidget, QScrollArea)

from . import __version__, geometry as g, operations as op
from .canvas import Canvas
from .dialogs import (number, form_dialog, text_dialog, make_table, set_rows,
                       table_rows, table_buttons, TagDelegate)
from .model import Document, History, Member, StopPoint
from .projection import ProjectionConfig, Projector
from .pointcloud import read_pcd, height_at
from .exchange import read_csv, write_csv, save_project, load_project
from .validation import validate, compare
from .storage import atomic_text


def apply_theme(app):
    from PySide6.QtGui import QPalette,QColor
    app.setStyle("Fusion")
    palette=QPalette()
    for role,color in [(QPalette.ColorRole.Window,"#202b37"),(QPalette.ColorRole.WindowText,"#e0e8f0"),
        (QPalette.ColorRole.Base,"#17222e"),(QPalette.ColorRole.AlternateBase,"#243342"),
        (QPalette.ColorRole.Text,"#e0e8f0"),(QPalette.ColorRole.Button,"#2d3b4a"),
        (QPalette.ColorRole.ButtonText,"#e0e8f0"),(QPalette.ColorRole.Highlight,"#2a6f92"),
        (QPalette.ColorRole.HighlightedText,"#ffffff"),(QPalette.ColorRole.ToolTipBase,"#27394a"),
        (QPalette.ColorRole.ToolTipText,"#f5f7fa")]:palette.setColor(role,QColor(color))
    app.setPalette(palette)


class CloudLoader(QThread):
    loaded=Signal(object)
    failed=Signal(str)
    def __init__(self,path,parent=None):
        super().__init__(parent);self.path=path
    def run(self):
        try:self.loaded.emit(read_pcd(self.path))
        except Exception as exc:self.failed.emit(f"{self.path}\n{exc}")


class MainWindow(QMainWindow):
    def __init__(self,testing=False):
        super().__init__()
        apply_theme(QApplication.instance())
        self.testing=testing
        self.document=Document();self.history=History(self.document)
        self.project_path=None;self.asset_dirty=False
        self.metadata={};self.metadata_source=None
        self.loaders=[];self._pending_clouds=[];self.external_process=None
        self.resize(1480,920);self.setMinimumSize(1080,700)
        self.canvas=Canvas(self.document);self.setCentralWidget(self.canvas)
        self.canvas.selectionChanged.connect(self.selection_changed)
        self.canvas.drawn.connect(self.on_drawn);self.canvas.moved.connect(self.on_moved)
        self.cursor_label=QLabel();self.statusBar().addPermanentWidget(self.cursor_label)
        self.canvas.cursorInfo.connect(self.cursor_label.setText)
        self._make_tree();self._make_inspector();self._make_bottom();self._make_menus();self._make_tools()
        self.setStyleSheet("""
            QMainWindow { background: #17212c; }
            QDockWidget { font-weight: 600; }
            QToolBar { spacing: 5px; padding: 5px; }
            QToolButton { padding: 6px; }
            QLineEdit, QDoubleSpinBox, QComboBox { min-height: 25px; }
            QPushButton { padding: 5px 9px; }
            QTabWidget::pane { border: 1px solid #424c56; }
        """)
        self.autosave_timer=QTimer(self);self.autosave_timer.setInterval(60000)
        self.autosave_timer.timeout.connect(self.autosave)
        self.recovery_path=Path(QStandardPaths.writableLocation(QStandardPaths.StandardLocation.CacheLocation))/"recovery.vmbqt"
        if not testing:self.autosave_timer.start()
        self.refresh()
        self.log("VMBQt 离线地图编辑器 0.1 · Qt 6 / PySide6 · 坐标单位 m，速度 km/h，角度 rad")

    def log(self,text):
        self.log_edit.appendPlainText(str(text));self.statusBar().showMessage(str(text).splitlines()[0],7000)

    def report_error(self,exc):
        self.log(f"错误：{exc}")
        if not self.testing:QMessageBox.warning(self,"操作未完成",str(exc))

    def edit(self,label,operation,select=None):
        try:
            result=self.history.edit(label,operation)
            if select:self.canvas.select(select(result),False)
            self.refresh();self.log(label);return result
        except Exception as exc:
            self.refresh();self.report_error(exc);return None

    def refresh(self):
        self.canvas.document=self.document;self.canvas.refresh();self.refresh_tree()
        self.selection_changed(self.canvas.selection,from_tree=True)
        star=" *" if self.history.dirty or self.asset_dirty else ""
        self.setWindowTitle(f"{self.document.name}{star} — VMBQt · Offline Lanelet2 Editor")
        if hasattr(self,"undo_action"):
            self.undo_action.setEnabled(bool(self.history.undo_items))
            self.redo_action.setEnabled(bool(self.history.redo_items))
        self.count_label.setText(f"{len(self.document.nodes)} 点  ·  {len(self.document.ways)} 线  ·  {len(self.document.lanelets())} 车道")

    def _action(self,menu,text,callback,shortcut=None,checkable=False,checked=False):
        action=QAction(text,self);action.setCheckable(checkable);action.setChecked(checked)
        if shortcut:action.setShortcut(QKeySequence(shortcut))
        action.triggered.connect(lambda checked=False: self._safe(lambda:callback(checked) if checkable else callback()))
        menu.addAction(action);return action

    def _safe(self,callback):
        try:return callback()
        except Exception as exc:self.report_error(exc)

    def _make_menus(self):
        f=self.menuBar().addMenu("文件")
        self._action(f,"新建工程",self.new_project,"Ctrl+N")
        self._action(f,"打开工程 .vmbqt…",self.open_project,"Ctrl+O")
        self._action(f,"保存工程…",self.save,"Ctrl+S")
        self._action(f,"工程另存为…",lambda:self.save(True),"Ctrl+Shift+S")
        f.addSeparator()
        self._action(f,"导入 Lanelet2 OSM（替换当前地图）…",self.import_osm)
        self._action(f,"合并另一张 OSM（重分配 ID）…",self.merge_osm)
        self._action(f,"导入 PCD 文件…",self.import_clouds)
        self._action(f,"导入 PCD 目录…",self.import_cloud_directory)
        self._action(f,"导入/管理点云 metadata YAML…",self.manage_metadata)
        self._action(f,"导入轨迹 CSV…",lambda:self.import_csv("trajectory"))
        self._action(f,"导入停靠点 CSV…",lambda:self.import_csv("stop"))
        self._action(f,"导入平面底图…",self.import_image)
        f.addSeparator()
        self._action(f,"导出 Lanelet2 OSM…",self.export_osm,"Ctrl+E")
        self._action(f,"导出轨迹 CSV…",lambda:self.export_csv("trajectory"))
        self._action(f,"导出停靠点 CSV…",lambda:self.export_csv("stop"))
        self._action(f,"导出当前视图 PNG…",self.export_image)
        self._action(f,"导出 Autoware 地图目录…",self.export_bundle)
        f.addSeparator();self._action(f,"退出",self.close,"Ctrl+Q")
        e=self.menuBar().addMenu("编辑")
        self.undo_action=self._action(e,"撤销",self.undo,"Ctrl+Z")
        self.redo_action=self._action(e,"重做",self.redo,"Ctrl+Shift+Z")
        self._action(e,"删除选中对象（保护引用）",self.delete_selection,"Delete")
        self._action(e,"复制并平移…",self.duplicate_selection,"Ctrl+D")
        self._action(e,"平移/旋转/缩放…",self.transform_selection,"Ctrl+T")
        self._action(e,"合并两个点（保留先选 ID）",self.merge_points)
        self._action(e,"清理无标签孤立点",lambda:self.edit("清理孤立点",lambda:op.clean_orphans(self.document)))
        self._action(e,"按 ID 查找…",self.find_object,"Ctrl+F")
        self._action(e,"批量设置标签…",self.batch_tags)
        self._action(e,"编辑轨迹点表…",self.edit_trajectory)
        self._action(e,"编辑停靠点表…",self.edit_stops)
        v=self.menuBar().addMenu("视图")
        self._action(v,"适应全部",self.canvas.fit_all,"F")
        self._action(v,"聚焦选中对象",self.canvas.focus_refs,"Space")
        self._action(v,"跳转 XY / 经纬度…",self.goto_coordinate)
        self._action(v,"重置旋转",self.reset_rotation)
        self.ids_action=self._action(v,"显示 ID",lambda c:self.set_view("show_ids",c),"I",True)
        self.directions_action=self._action(v,"显示方向箭头",lambda c:self.set_view("show_directions",c),None,True,True)
        self.grid_action=self._action(v,"显示网格",lambda c:self.set_view("show_grid",c),None,True,True)
        self._action(v,"背景与图层颜色…",self.edit_colors)
        self._action(v,"导入显示设置 JSON…",self.import_settings)
        self._action(v,"导出显示设置 JSON…",self.export_settings)
        lane=self.menuBar().addMenu("车道/几何")
        for text,callback in [
            ("两条边界创建车道（先左后右）",self.lane_from_bounds),
            ("左侧增加共享边界车道",lambda:self.adjacent("left")),
            ("右侧增加共享边界车道",lambda:self.adjacent("right")),
            ("更改车道宽度…",self.change_width),
            ("生成/更新中心线",self.centerlines),
            ("反转车道方向",self.reverse_lanes),
            ("连接两车道端点…",self.connect_lanes),
            ("首尾合并两条车道",self.join_lanes),
            ("按比例拆分车道…",self.split_lane),
            ("创建两车道间的转弯连接",self.connector),
            ("在选中 Polygon 内批量创建交叉口连接…",self.intersection_dialog),
            ("用轨迹生成车道",self.lane_from_trajectory),
            ("选中车道本地拓扑寻路",self.plan_route),
            ("清除路径预览",lambda:self.clear_route()),
            ("合并两条独立线",self.join_ways),
            ("偏移复制折线…",self.offset_way),
            ("折线重采样/平滑…",self.smooth_way),
            ("线/多边形互转",self.toggle_polygon),
            ("所选闭合边界创建 Area",self.make_area),
            ("创建通用关系（JSON）…",self.new_relation),
            ("贴合点云高度…",self.fit_height)]:self._action(lane,text,callback)
        rules=self.menuBar().addMenu("交通规则")
        self._action(rules,"给选中车道添加规则…",self.rule_dialog)
        self._action(rules,"两车道创建人行横道规则",self.crosswalk_rule)
        self._action(rules,"两车道设置路权（先优先、后让行）",self.right_of_way)
        self._action(rules,"允许边界换道",lambda:self.set_tag_selected("lane_change","yes","way"))
        self._action(rules,"禁止边界换道",lambda:self.set_tag_selected("lane_change","no","way"))
        self._action(rules,"边界设为虚线",lambda:self.set_tag_selected("subtype","dashed","way"))
        self._action(rules,"边界设为实线",lambda:self.set_tag_selected("subtype","solid","way"))
        m=self.menuBar().addMenu("地图")
        self._action(m,"设置投影/地图信息…",self.projection_dialog)
        self._action(m,"导入 map_projector_info.yaml…",self.import_projection)
        self._action(m,"导出 map_projector_info.yaml…",self.export_projection)
        self._action(m,"编辑 MapMetaInfo…",self.map_meta)
        self._action(m,"比较另一张地图/应用差异…",self.compare_maps)
        self._action(m,"运行内置检查",self.run_checks,"F7")
        self._action(m,"外部 Autoware 验证器…",self.external_validator)
        h=self.menuBar().addMenu("帮助")
        self._action(h,"操作与已知差异",self.show_help)
        self._action(h,"关于",lambda:QMessageBox.about(self,"VMBQt",f"VMBQt {__version__}\nQt 6 / PySide6 原生离线编辑器\n原创实现，与 TIER IV 无关联。\n不是网页版全部功能的等价复刻。"))

    def _make_tools(self):
        toolbar=QToolBar("绘制",self);toolbar.setMovable(False);self.addToolBar(toolbar)
        self.tool_group=QActionGroup(self);self.tool_actions={}
        tools=[("select","选择 [V]"),("lane","车道"),("crosswalk","横道"),("line","折线"),
               ("polygon","多边形"),("point","点"),("rectangle","矩形"),("circle","圆"),
               ("area","Area"),("measure","测距"),("stop","停靠点")]
        for tool,label in tools:
            a=QAction(label,self);a.setCheckable(True);a.setData(tool);self.tool_group.addAction(a)
            if tool=="select":a.setChecked(True);a.setShortcut(QKeySequence("V"))
            a.triggered.connect(lambda checked=False,t=tool:self.canvas.set_tool(t))
            toolbar.addAction(a);self.tool_actions[tool]=a
        self.addToolBarBreak()
        second=QToolBar("规则与视点",self);second.setMovable(False);self.addToolBar(second)
        for tool,label in [("stopline","停止线+标志"),("trafficlight","信号灯+停止线"),
                           ("detection_area","检测区域"),("no_stopping_area","禁停区域"),
                           ("no_parking_area","禁泊区域"),("speed_bump","减速带"),
                           ("insert","插点"),("split","拆线"),("mirror","镜像复制")]:
            a=QAction(label,self);a.setCheckable(True);self.tool_group.addAction(a)
            a.triggered.connect(lambda checked=False,t=tool:self.canvas.set_tool(t));second.addAction(a);self.tool_actions[tool]=a
        third=QToolBar("视图设置",self);third.setMovable(False);self.addToolBarBreak();self.addToolBar(third)
        for text,value in [("2D",False),("3D",True)]:
            button=QPushButton(text);button.clicked.connect(lambda checked=False,v=value:self.set_3d(v));third.addWidget(button)
        self.perspective=QCheckBox("透视");self.perspective.toggled.connect(lambda c:self.set_view("perspective",c,camera=True));third.addWidget(self.perspective)
        third.addSeparator();third.addWidget(QLabel("选择类型 "))
        self.pick_combo=QComboBox()
        for text,value in [("全部","all"),("点","node"),("线/多边形","way"),("车道","lane"),("Area","area"),("停靠点","stop")]:self.pick_combo.addItem(text,value)
        self.pick_combo.currentIndexChanged.connect(lambda:self.set_view("pick_kind",self.pick_combo.currentData()));third.addWidget(self.pick_combo)
        self.snap_check=QCheckBox("吸附节点");self.snap_check.setChecked(True);self.snap_check.toggled.connect(lambda c:self.set_view("snap_enabled",c));third.addWidget(self.snap_check)
        self.cloud_snap_check=QCheckBox("采点云 Z");self.cloud_snap_check.toggled.connect(lambda c:self.set_view("cloud_snap",c));third.addWidget(self.cloud_snap_check)
        third.addWidget(QLabel("编辑 Z "));self.edit_z=number(0,-1e6,1e6,3);self.edit_z.setMaximumWidth(110)
        self.edit_z.valueChanged.connect(lambda v:self.set_view("edit_z",v));third.addWidget(self.edit_z)
        third.addWidget(QLabel("车道宽 "));self.width_spin=number(3.5,.2,50,2);self.width_spin.setMaximumWidth(85);third.addWidget(self.width_spin)
        third.addWidget(QLabel("速度 "));self.speed_spin=number(30,.1,160,1);self.speed_spin.setMaximumWidth(85);third.addWidget(self.speed_spin)
        self.canvas.toolChanged.connect(lambda name:self.tool_actions[name].setChecked(True) if name in self.tool_actions else None)

    def _make_tree(self):
        dock=QDockWidget("项目 / 对象",self);dock.setObjectName("project")
        box=QWidget();layout=QVBoxLayout(box)
        self.filter_edit=QLineEdit();self.filter_edit.setPlaceholderText("按 ID、名称或类型过滤")
        self.filter_edit.textChanged.connect(self.filter_tree);layout.addWidget(self.filter_edit)
        self.tree=QTreeWidget();self.tree.setHeaderLabels(["对象","type / subtype"])
        self.tree.setSelectionMode(QAbstractItemView.SelectionMode.ExtendedSelection)
        self.tree.itemSelectionChanged.connect(self.tree_selection)
        self.tree.itemDoubleClicked.connect(lambda item,col:self.canvas.focus_refs([item.data(0,Qt.ItemDataRole.UserRole)]) if item.data(0,Qt.ItemDataRole.UserRole) else None)
        layout.addWidget(self.tree)
        self.count_label=QLabel();self.count_label.setWordWrap(True);layout.addWidget(self.count_label)
        dock.setWidget(box);self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea,dock);dock.setMinimumWidth(245)

    def refresh_tree(self):
        self.tree.blockSignals(True);self.tree.clear();self._tree_refs={}
        groups={}
        for name in ("车道 Lanelet","线 / 多边形 Way","规则 RegulatoryElement","区域 Area","其他关系","点 Point","停靠点 StopPoint"):
            groups[name]=QTreeWidgetItem(self.tree,[name]);groups[name].setExpanded(name!="点 Point")
        for kind,objects in (("relation",self.document.relations),("way",self.document.ways),
                             ("node",self.document.nodes),("stop",self.document.stops)):
            for oid,obj in objects.items():
                tags=getattr(obj,"tags",{})
                group=("车道 Lanelet" if tags.get("type")=="lanelet" else "规则 RegulatoryElement"
                       if tags.get("type")=="regulatory_element" else "区域 Area"
                       if tags.get("type")=="multipolygon" else "其他关系") if kind=="relation" else \
                      {"way":"线 / 多边形 Way","node":"点 Point","stop":"停靠点 StopPoint"}[kind]
                item=QTreeWidgetItem(groups[group],[f"{oid}  {getattr(obj,'name',tags.get('name',''))}",
                                         f"{tags.get('type','')} / {tags.get('subtype','')}"])
                ref=(kind,oid);item.setData(0,Qt.ItemDataRole.UserRole,ref);self._tree_refs[ref]=item
                item.setSelected(ref in self.canvas.selection)
        for group in groups.values():group.setText(0,f"{group.text(0)} ({group.childCount()})")
        self.tree.blockSignals(False);self.filter_tree()

    def filter_tree(self):
        query=self.filter_edit.text().lower().strip()
        for item in self._tree_refs.values():item.setHidden(query not in (item.text(0)+" "+item.text(1)).lower())

    def tree_selection(self):
        refs=[tuple(i.data(0,Qt.ItemDataRole.UserRole)) for i in self.tree.selectedItems() if i.data(0,Qt.ItemDataRole.UserRole)]
        ordered=[r for r in self.canvas.selection if r in refs]+[r for r in refs if r not in self.canvas.selection]
        self.canvas.select(ordered)

    def _make_inspector(self):
        dock=QDockWidget("编辑 / 显示",self);dock.setObjectName("inspector")
        self.right_tabs=QTabWidget();dock.setWidget(self.right_tabs)
        inspector=QWidget();layout=QVBoxLayout(inspector)
        self.selection_label=QLabel("未选择对象");self.selection_label.setWordWrap(True);layout.addWidget(self.selection_label)
        self.coords_box=QWidget();form=QFormLayout(self.coords_box);self.coords={}
        for k in ("x","y","z","yaw"):
            self.coords[k]=number(0,-1e9,1e9,6);form.addRow(k,self.coords[k])
        self.stop_name=QLineEdit();self.stop_lane=QLineEdit();form.addRow("停靠点名称",self.stop_name);form.addRow("lanelet_id",self.stop_lane)
        layout.addWidget(self.coords_box)
        self.way_nodes=QLineEdit();self.way_nodes.setPlaceholderText("顺序节点 ID，以逗号分隔")
        layout.addWidget(self.way_nodes)
        self.tag_table=make_table(["Tag key","value"]);self.tag_table.setItemDelegate(TagDelegate(self.tag_table))
        layout.addWidget(self.tag_table);layout.addLayout(table_buttons(self.tag_table))
        self.member_table=make_table(["type","ref","role"]);layout.addWidget(self.member_table)
        self.member_buttons=QWidget();mb=QVBoxLayout(self.member_buttons);mb.setContentsMargins(0,0,0,0);mb.addLayout(table_buttons(self.member_table));layout.addWidget(self.member_buttons)
        apply=QPushButton("应用属性修改");apply.clicked.connect(self.apply_inspector);layout.addWidget(apply)
        self.parents_list=QListWidget();self.parents_list.setMaximumHeight(100)
        self.parents_list.itemDoubleClicked.connect(lambda i:self.canvas.select([tuple(i.data(Qt.ItemDataRole.UserRole))]))
        layout.addWidget(QLabel("被以下对象引用（双击跳转）"));layout.addWidget(self.parents_list)
        self.right_tabs.addTab(inspector,"属性")
        self._make_view_settings()
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea,dock);dock.setMinimumWidth(310)

    def _make_view_settings(self):
        widget=QWidget();layout=QVBoxLayout(widget);form=QFormLayout()
        self.layer_controls={}
        for key,label in [("node","点"),("way","线 / Polygon"),("lane","Lanelet"),("area","Area"),
                          ("stop","停靠点"),("trajectory","轨迹"),("cloud","点云"),("image","底图")]:
            check=QCheckBox(label);check.setChecked(True)
            check.toggled.connect(lambda value,k=key:self.set_layer(k,value));layout.addWidget(check)
            self.layer_controls[key]=check
        self.view_controls={}
        choices=[("cloud_opacity","点云透明度",.8,0.,1.),("map_opacity","面填充透明度",.32,0.,1.),
                 ("z_min","点云最低 Z",-10000,-1e9,1e9),("z_max","点云最高 Z",10000,-1e9,1e9),
                 ("intensity_max","强度最大值",255,.001,1e9),("snap_grid","网格吸附（0 关闭）",0.,0.,1000.),
                 ("max_display","点云显示上限",180000.,1000.,1000000.),("cloud_size","点大小 px",1.,1.,3.)]
        for key,label,default,low,high in choices:
            control=number(default,low,high,0 if key in ("max_display","cloud_size") else 3)
            control.valueChanged.connect(lambda value,k=key:self.set_view(k,int(value) if k in ("max_display","cloud_size") else value))
            form.addRow(label,control);self.view_controls[key]=control
        color=QComboBox();color.addItems(["intensity","height","rgb"])
        color.currentTextChanged.connect(lambda v:self.set_view("cloud_color",v));form.addRow("点云着色",color)
        layout.addLayout(form);layout.addWidget(QLabel("已加载点云（显示为采样数据，原文件不变）"))
        self.cloud_list=QListWidget();self.cloud_list.itemChanged.connect(self.cloud_visibility)
        layout.addWidget(self.cloud_list)
        remove=QPushButton("移除选中点云");remove.clicked.connect(self.remove_cloud);layout.addWidget(remove)
        layout.addStretch()
        scroll=QScrollArea();scroll.setWidgetResizable(True);scroll.setWidget(widget)
        self.right_tabs.addTab(scroll,"图层 / 点云")

    def _make_bottom(self):
        dock=QDockWidget("检查 / 日志",self);dock.setObjectName("checks")
        tabs=QTabWidget();self.issue_table=make_table(["等级","对象","问题"])
        self.issue_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.issue_table.cellDoubleClicked.connect(self.focus_issue)
        self.log_edit=QPlainTextEdit();self.log_edit.setReadOnly(True);self.log_edit.setMaximumBlockCount(2000)
        tabs.addTab(self.issue_table,"检查结果（F7 / 双击定位）");tabs.addTab(self.log_edit,"日志")
        dock.setWidget(tabs);self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea,dock)
        dock.setMinimumHeight(140);self.resizeDocks([dock],[160],Qt.Orientation.Vertical)

    def selection_changed(self,refs,from_tree=False):
        refs=list(map(tuple,refs));self.selection_label.setText("未选择对象" if not refs else
                         f"已选 {len(refs)} 个："+", ".join(f"{k}/{i}" for k,i in refs[:5]))
        if not from_tree:
            self.tree.blockSignals(True)
            for ref,item in self._tree_refs.items():item.setSelected(ref in refs)
            self.tree.blockSignals(False)
        obj=self.document.get(refs[0]) if len(refs)==1 else None
        kind=refs[0][0] if obj else None
        self.coords_box.setVisible(kind in ("node","stop"));self.way_nodes.setVisible(kind=="way")
        self.member_table.setVisible(kind=="relation");self.member_buttons.setVisible(kind=="relation")
        self.tag_table.setEnabled(kind in ("node","way","relation"));self.parents_list.clear()
        tags=dict(getattr(obj,"tags",{}))
        if kind=="node":
            for k in ("local_x","local_y","ele"):tags.pop(k,None)
        set_rows(self.tag_table,sorted(tags.items()))
        if kind in ("node","stop"):
            for k in self.coords:self.coords[k].setValue(float(getattr(obj,k,0)))
            self.coords["yaw"].setEnabled(kind=="stop")
            self.stop_name.setEnabled(kind=="stop");self.stop_lane.setEnabled(kind=="stop")
            self.stop_name.setText(getattr(obj,"name",""));self.stop_lane.setText(str(getattr(obj,"lanelet_id",0)))
        if kind=="way":self.way_nodes.setText(",".join(map(str,obj.nodes)))
        if kind=="relation":set_rows(self.member_table,[(m.type,m.ref,m.role) for m in obj.members])
        if obj:
            from PySide6.QtWidgets import QListWidgetItem
            for ref in self.document.parents(refs[0]):
                item=QListWidgetItem(f"{ref[0]}/{ref[1]}");item.setData(Qt.ItemDataRole.UserRole,ref);self.parents_list.addItem(item)

    def apply_inspector(self):
        refs=self.canvas.selection
        if len(refs)!=1:return self.report_error("属性编辑需单选；批量修改请使用编辑菜单")
        ref=refs[0]
        def operation():
            obj=self.document.get(ref);kind=ref[0]
            if kind in ("node","way","relation"):
                rows=table_rows(self.tag_table);tags={}
                for k,v in rows:
                    if not k:continue
                    if k in tags:raise ValueError(f"标签重复：{k}")
                    tags[k]=v
                obj.tags=tags
            if kind in ("node","stop"):
                for k in ("x","y","z"):setattr(obj,k,self.coords[k].value())
                if kind=="stop":
                    obj.yaw=self.coords["yaw"].value();obj.name=self.stop_name.text()
                    obj.lanelet_id=int(self.stop_lane.text() or "0")
            if kind=="way":
                ids=[int(i.strip()) for i in self.way_nodes.text().replace("，",",").split(",") if i.strip()]
                if any(i not in self.document.nodes for i in ids):raise ValueError("引用了不存在的节点")
                obj.nodes=ids
            if kind=="relation":
                members=[]
                for typ,oid,role in table_rows(self.member_table):
                    if not typ and not oid:continue
                    if typ not in ("node","way","relation"):raise ValueError("成员 type 为 node/way/relation")
                    oid=int(oid)
                    if self.document.get((typ,oid)) is None:raise ValueError(f"成员不存在：{typ}/{oid}")
                    members.append(Member(typ,oid,role))
                obj.members=members
        self.edit("修改对象属性",operation)

    def on_moved(self,refs,delta):
        self.edit("移动选中几何",lambda:op.transform(self.document,refs,*map(float,delta)))

    def selected(self,kind=None,count=None,lanes=False):
        refs=[r for r in self.canvas.selection if (not kind or r[0]==kind) and
              (not lanes or r[0]=="relation" and self.document.relations[r[1]].tags.get("type")=="lanelet")]
        if count is not None and len(refs)!=count:raise ValueError(f"请按操作顺序选择 {count} 个{'车道' if lanes else kind or '对象'}")
        if not refs:raise ValueError("请先选择对象")
        return refs

    def on_drawn(self,tool,draft):
        # Collapse duplicate double-click positions; snapped ids are reused by raw ways.
        clean=[]
        for xyz,nid in draft:
            if not clean or np.linalg.norm(np.array(xyz)-clean[-1][0])>1e-6:clean.append((np.array(xyz),nid))
        if not clean:return
        points=np.array([p for p,_ in clean]);d=self.document
        try:
            lane_ids=[r[1] for r in self.selected(lanes=True)] if tool in (
                "stopline","trafficlight","detection_area","no_stopping_area","no_parking_area","speed_bump","stop") else []
            if tool=="measure":
                self.log(f"距离：XY {g.length(points,True):.4f} m；3D {g.length(points):.4f} m");return
            def operation():
                if tool=="point":return "node",d.add_node(points[0])
                if tool=="stop":
                    lid=lane_ids[0];center=op.lane_center(d,lid)
                    _,i,_,foot=g.nearest_segment(center,points[0]);t=center[i+1]-center[i]
                    oid=d.new_id();d.stops[oid]=StopPoint(oid,*map(float,foot),math.atan2(t[1],t[0]),f"stop_{oid}",lid)
                    return "stop",oid
                if tool in ("stopline","trafficlight"):
                    return "relation",op.stop_line_at(d,lane_ids[0],points[0],"traffic_sign" if tool=="stopline" else "traffic_light")
                if tool in ("insert","split"):
                    wid=self.selected("way",1)[0][1];nid=op.insert_point(d,wid,points[0])
                    return ("way",op.split_way(d,wid,nid)) if tool=="split" else ("node",nid)
                if tool=="mirror":
                    if len(points)<2:raise ValueError("镜像轴需要两点")
                    refs=op.clone(d,self.selected());op.transform(d,refs,mirror=points[:2]);return refs[0]
                if tool in ("lane","crosswalk"):
                    return "relation",op.create_lane(d,points,self.width_spin.value(),self.speed_spin.value(),"crosswalk" if tool=="crosswalk" else "road")
                if tool=="circle":
                    if len(points)<2:raise ValueError("圆需要圆心和半径点")
                    radius=np.linalg.norm((points[1]-points[0])[:2])
                    if radius<.01:raise ValueError("半径过小")
                    a=np.linspace(0,2*math.pi,65)[:-1]
                    coordinates=points[0]+np.column_stack([radius*np.cos(a),radius*np.sin(a),np.zeros_like(a)])
                    ids=[d.add_node(p) for p in coordinates];ids.append(ids[0])
                    return "way",d.add_way(ids,op.LINE_TAGS,ids=True)
                if tool=="rectangle":
                    if len(points)<2:raise ValueError("矩形需要两个对角点")
                    a,b=points[:2];coordinates=[a,[b[0],a[1],a[2]],[b[0],b[1],a[2]],[a[0],b[1],a[2]]]
                    ids=[d.add_node(p) for p in coordinates]
                else:ids=[nid if nid in d.nodes else d.add_node(p) for p,nid in clean]
                polygon=tool in ("polygon","area","rectangle","detection_area","no_stopping_area","no_parking_area","speed_bump")
                if len(set(ids))<(3 if polygon else 2):raise ValueError("多边形至少需要三点，折线至少需要两点")
                if polygon and ids[0]!=ids[-1]:ids.append(ids[0])
                tags={"type":tool if tool in ("detection_area","no_stopping_area","no_parking_area","speed_bump") else "parking_lot","area":"yes"} if polygon else dict(op.LINE_TAGS)
                wid=d.add_way(ids,tags,ids=True)
                if tool=="area":
                    d.ways[wid].tags=dict(op.LINE_TAGS)
                    return "relation",d.add_relation([("way",wid,"outer")],{"type":"multipolygon","subtype":"parking"})
                if tool in ("detection_area","no_stopping_area","no_parking_area","speed_bump"):
                    members=[("way",wid,"refers")]
                    if tool in ("detection_area","no_stopping_area"):
                        stop=self.plain_stopline(lane_ids[0],points[0]);members.append(("way",stop,"ref_line"))
                    return "relation",op.add_rule(d,lane_ids,tool,members)
                return "way",wid
            self.edit(f"绘制 {tool}",operation,select=lambda ref:[ref])
        except Exception as exc:self.report_error(exc)

    def plain_stopline(self,lid,point):
        d=self.document;center=op.lane_center(d,lid);_,i,_,foot=g.nearest_segment(center,point)
        t=center[i+1]-center[i];t/=max(np.linalg.norm(t[:2]),1e-12)
        a,b=op.bound_nodes(d,lid);width=np.linalg.norm(d.nodes[a[-1]].xyz-d.nodes[b[-1]].xyz)
        normal=np.array([-t[1],t[0],0.])*width/2
        return d.add_way([foot+normal,foot-normal],{"type":"stop_line"})

    def undo(self):
        label=self.history.undo();self.refresh()
        if label:self.log(f"撤销：{label}")

    def redo(self):
        label=self.history.redo();self.refresh()
        if label:self.log(f"重做：{label}")

    def delete_selection(self):
        refs=self.selected();self.edit("删除对象",lambda:op.delete_objects(self.document,refs))

    def transform_selection(self,duplicate=False):
        refs=self.selected()
        values=form_dialog(self,"复制并变换" if duplicate else "变换选中几何",[
            ("dx","X 平移 / m",3. if duplicate else 0.),("dy","Y 平移 / m",0.),("dz","Z 平移 / m",0.),
            ("angle","绕选中点集中心旋转 / deg",0.),("scale","XY 缩放比例",1.)],
            "共享节点只变换一次；引用这些节点的其他对象也会随之改变。")
        if values:
            def operation():
                targets=op.clone(self.document,refs) if duplicate else refs
                op.transform(self.document,targets,values["dx"],values["dy"],values["dz"],math.radians(values["angle"]),values["scale"])
                return targets
            self.edit("复制并变换" if duplicate else "变换选中对象",operation,lambda r:r)

    def duplicate_selection(self):self.transform_selection(True)

    def merge_points(self):
        refs=self.selected("node",2);self.edit("合并节点",lambda:op.merge_nodes(self.document,refs[0][1],refs[1][1]))

    def find_object(self):
        v=form_dialog(self,"按 ID 查找",[("kind","OSM 类型",["relation","way","node","stop"]),("id","ID","")])
        if v:
            ref=(v["kind"],int(v["id"]))
            if self.document.get(ref) is None:raise ValueError("ID 不存在")
            self.canvas.select([ref]);self.canvas.focus_refs()

    def batch_tags(self):
        refs=self.selected();v=form_dialog(self,"批量标签",[("key","key","speed_limit"),("value","value（空值删除该 key）","30")])
        if v:
            if not v["key"]:raise ValueError("key 不能为空")
            self.set_tag_selected(v["key"],v["value"])

    def set_tag_selected(self,key,value,kind=None):
        refs=self.selected(kind)
        def operation():
            for ref in refs:
                obj=self.document.get(ref)
                if not hasattr(obj,"tags"):continue
                if value:obj.tags[key]=value
                else:obj.tags.pop(key,None)
        self.edit(f"设置标签 {key}={value}",operation)

    def lane_from_bounds(self):
        refs=self.selected("way",2)
        self.edit("从左右边界创建车道",lambda:op.lane_from_bounds(self.document,refs[0][1],refs[1][1],self.speed_spin.value()),lambda i:[("relation",i)])

    def adjacent(self,side):
        lid=self.selected(lanes=True,count=1)[0][1]
        self.edit("增加共享边界车道",lambda:op.add_adjacent(self.document,lid,side,self.width_spin.value()),lambda i:[("relation",i)])

    def change_width(self):
        lid=self.selected(lanes=True,count=1)[0][1]
        v=form_dialog(self,"更改车道宽度",[("width","宽度 / m",self.width_spin.value())])
        if v:self.edit("调整车道宽度",lambda:op.change_width(self.document,lid,v["width"]))

    def centerlines(self):
        refs=self.selected(lanes=True)
        self.edit("生成中心线",lambda:[op.generate_centerline(self.document,i) for _,i in refs])

    def reverse_lanes(self):
        ids=[i for _,i in self.selected(lanes=True)];self.edit("反转车道方向",lambda:op.reverse_lanes(self.document,ids))

    def connect_lanes(self):
        refs=self.selected(lanes=True,count=2)
        v=form_dialog(self,"连接车道端点",[("tolerance","允许合并的最大距离 / m",2.)],"先选前车道，再选后车道。后车道端点将移动到前车道端点，使用相同 ID。")
        if v:self.edit("连接车道共享端点",lambda:op.connect_lanes(self.document,refs[0][1],refs[1][1],v["tolerance"]))

    def join_lanes(self):
        refs=self.selected(lanes=True,count=2)
        self.edit("合并车道",lambda:op.join_lanes(self.document,refs[0][1],refs[1][1]),lambda i:[("relation",i)])

    def split_lane(self):
        lid=self.selected(lanes=True,count=1)[0][1]
        v=form_dialog(self,"拆分车道",[("fraction","按边界弧长拆分的比例",.5)],"交通规则转移到后一段；共享边界或被规则引用的车道会阻止拆分。")
        if v:self.edit("拆分车道",lambda:op.split_lane(self.document,lid,v["fraction"]),lambda i:[("relation",lid),("relation",i)])

    def connector(self):
        refs=self.selected(lanes=True,count=2)
        self.edit("创建三次 Bézier 连接车道",lambda:op.create_connector(self.document,refs[0][1],refs[1][1]),lambda i:[("relation",i)])

    def intersection_dialog(self):
        wid=self.selected("way",1)[0][1]
        polygon=self.document.points(wid)
        if self.document.ways[wid].tags.get("area")!="yes" or len(polygon)<3:
            raise ValueError("先绘制并选择一个覆盖交叉口的 Polygon")
        incoming=[];outgoing=[]
        for lane in self.document.lanelets():
            if lane.tags.get("subtype")!="road":continue
            center=op.lane_center(self.document,lane.id)
            if g.contains(polygon,center[-1]):incoming.append(lane.id)
            if g.contains(polygon,center[0]):outgoing.append(lane.id)
        pairs=[(a,b) for a in incoming for b in outgoing if a!=b]
        if not pairs:raise ValueError("区域内没有可连接的入口末端与出口首端；可改用选择两车道创建连接")
        if len(pairs)>100:raise ValueError("候选连接超过 100 对，请缩小 Polygon 范围")
        dialog=QDialog(self);dialog.setWindowTitle("批量交叉口连接（显式选择允许的转向）");dialog.resize(620,500)
        layout=QVBoxLayout(dialog)
        label=QLabel("使用三次 Bézier 插值，并共享前后车道端点。这里只生成几何；路权、横道和信号需另外配置。")
        label.setWordWrap(True);layout.addWidget(label)
        table=make_table(["创建","入口 Lanelet","出口 Lanelet"])
        from PySide6.QtWidgets import QTableWidgetItem
        table.setRowCount(len(pairs))
        for row,(a,b) in enumerate(pairs):
            item=QTableWidgetItem();item.setFlags(Qt.ItemFlag.ItemIsEnabled|Qt.ItemFlag.ItemIsUserCheckable)
            item.setCheckState(Qt.CheckState.Unchecked);table.setItem(row,0,item)
            table.setItem(row,1,QTableWidgetItem(str(a)));table.setItem(row,2,QTableWidgetItem(str(b)))
        layout.addWidget(table);form=QFormLayout();spacing=number(1.,.2,10,2)
        form.addRow("采样间距 / m",spacing);layout.addLayout(form)
        buttons=QDialogButtonBox(QDialogButtonBox.StandardButton.Ok|QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(dialog.accept);buttons.rejected.connect(dialog.reject);layout.addWidget(buttons)
        if dialog.exec()!=QDialog.DialogCode.Accepted:return
        chosen=[pair for row,pair in enumerate(pairs) if table.item(row,0).checkState()==Qt.CheckState.Checked]
        if chosen:self.edit("创建交叉口连接",lambda:[op.create_connector(self.document,a,b,spacing.value()) for a,b in chosen],lambda ids:[("relation",i) for i in ids])

    def lane_from_trajectory(self):
        if len(self.document.trajectory)<2:raise ValueError("先导入至少两点的轨迹 CSV")
        points=np.array([[p.x,p.y,p.z] for p in self.document.trajectory])
        v=form_dialog(self,"轨迹生成车道",[("spacing","重采样点间距 / m",1.),("start","起始行（从 1）",1.),("end","终止行",float(len(points)))])
        if v:
            subset=points[max(0,int(v["start"])-1):int(v["end"])]
            p=g.sample(subset,max(2,int(g.length(subset)/max(.1,v["spacing"]))+1))
            self.edit("轨迹生成车道",lambda:op.create_lane(self.document,p,self.width_spin.value(),self.speed_spin.value()),lambda i:[("relation",i)])

    def plan_route(self):
        refs=self.selected(lanes=True,count=2)
        self.canvas.route_ids=op.route(self.document,refs[0][1],refs[1][1]);self.canvas.update()
        self.log("本地共享端点拓扑路径："+" → ".join(map(str,self.canvas.route_ids)))

    def clear_route(self):self.canvas.route_ids=[];self.canvas.update()

    def join_ways(self):
        refs=self.selected("way",2);self.edit("合并折线",lambda:op.join_ways(self.document,refs[0][1],refs[1][1]))

    def offset_way(self):
        wid=self.selected("way",1)[0][1];v=form_dialog(self,"偏移复制",[("distance","左侧为正 / m",3.5)])
        if v:self.edit("偏移复制折线",lambda:self.document.add_way(g.offset(self.document.points(wid),v["distance"]),self.document.ways[wid].tags),lambda i:[("way",i)])

    def smooth_way(self):
        wid=self.selected("way",1)[0][1]
        v=form_dialog(self,"重采样 / 平滑",[("count","输出节点数量",50.),("smooth","平滑",["否","是（Chaikin 两轮）"])],"保留首尾节点；内部节点将替换。与其他对象共享内部节点时会阻止操作。")
        if v:
            def operation():
                d=self.document;w=d.ways[wid]
                if w.tags.get("area")=="yes" or w.nodes[0]==w.nodes[-1]:raise ValueError("当前平滑工具只处理开放折线")
                if any(any(p!=("way",wid) for p in d.parents(("node",i))) for i in w.nodes[1:-1]):raise ValueError("内部节点与其他对象共享")
                points=d.points(wid)
                if v["smooth"].startswith("是"):points=g.smooth(points)
                points=g.sample(points,max(2,min(10000,int(v["count"]))))
                w.nodes=[w.nodes[0]]+[d.add_node(p) for p in points[1:-1]]+[w.nodes[-1]]
            self.edit("平滑 / 重采样折线",operation)

    def toggle_polygon(self):
        wid=self.selected("way",1)[0][1]
        def operation():
            d=self.document;w=d.ways[wid]
            if d.parents(("way",wid)):raise ValueError("对象已被引用，请先调整关系")
            if w.tags.get("area")=="yes":
                w.tags.pop("area",None);w.tags.update(op.LINE_TAGS)
            else:
                if len(set(w.nodes))<3:raise ValueError("Polygon 至少需要三个不同点")
                if w.nodes[0]!=w.nodes[-1]:w.nodes.append(w.nodes[0])
                w.tags.update(area="yes",type="parking_lot")
        self.edit("切换线 / Polygon",operation)

    def make_area(self):
        refs=self.selected("way")
        def operation():
            # Rings can consist of one closed way or multiple connected ways.
            ids=[list(self.document.ways[i].nodes) for _,i in refs]
            pending=list(ids);chain=pending.pop(0)
            while pending:
                found=False
                for j,p in enumerate(pending):
                    if chain[-1]==p[-1]:p=p[::-1]
                    if chain[-1]==p[0]:chain+=p[1:];pending.pop(j);found=True;break
                if not found:raise ValueError("边界必须按共享端点连接为闭合环")
            if chain[0]!=chain[-1]:raise ValueError("Area 外边界尚未闭合")
            return self.document.add_relation([("way",i,"outer") for _,i in refs],{"type":"multipolygon","subtype":"parking"})
        self.edit("创建 Area",operation,lambda i:[("relation",i)])

    def new_relation(self):
        text=text_dialog(self,"创建通用关系",json.dumps({"tags":{"type":"multipolygon","subtype":"parking"},"members":[{"type":k,"ref":i,"role":"outer"} for k,i in self.canvas.selection]},ensure_ascii=False,indent=2),"成员可使用 outer / inner 等 role；可表达带孔 Area 和其他 OSM 关系。")
        if text:
            data=json.loads(text)
            def operation():
                members=[Member(**m) for m in data["members"]]
                for m in members:
                    if m.type not in ("node","way","relation") or self.document.get((m.type,m.ref)) is None:raise ValueError("关系成员不存在")
                return self.document.add_relation(members,data["tags"])
            self.edit("创建通用关系",operation,lambda i:[("relation",i)])

    def fit_height(self):
        refs=self.selected();ids=self.document.geometry_nodes(refs)
        if not self.canvas.clouds:raise ValueError("请先加载点云")
        v=form_dialog(self,"贴合点云 Z",[("radius","XY 搜索半径 / m",1.)],"使用当前已采样、可见点云中的 XY 最近点。超出半径的节点保持原值。")
        if v:
            def operation():
                changed=0
                for i in ids:
                    n=self.document.nodes[i];z=height_at(self.canvas.clouds,n.xyz,v["radius"])
                    if z is not None:n.z=z;changed+=1
                self.log(f"贴合 {changed}/{len(ids)} 个节点")
            self.edit("贴合点云高度",operation)

    def rule_dialog(self):
        lanes=[i for _,i in self.selected(lanes=True)]
        selected_ways=[i for k,i in self.canvas.selection if k=="way"]
        v=form_dialog(self,"规则模板",[("subtype","subtype",["traffic_light","traffic_sign","detection_area","speed_bump","crosswalk","right_of_way","no_stopping_area","no_parking_area","bus_stop_area","virtual_traffic_light","road_marking","all_way_stop","speed_limit"])])
        if not v:return
        members=[{"type":"way","ref":i,"role":"ref_line" if self.document.ways[i].tags.get("type")=="stop_line" else "refers"} for i in selected_ways]
        text=text_dialog(self,"配置规则成员和标签",json.dumps({"members":members,"tags":{}},ensure_ascii=False,indent=2),
            f"将关联到 Lanelet {lanes}。成员格式 type/ref/role；交通灯用 refers、ref_line、light_bulbs；虚拟信号用 refers、ref_line、start_line、end_line。请按目标 Autoware 版本核对规则。")
        if text:
            data=json.loads(text)
            def operation():
                mem=[Member(**m) for m in data["members"]]
                for m in mem:
                    if m.type not in ("node","way","relation") or self.document.get((m.type,m.ref)) is None:raise ValueError("规则成员不存在")
                return op.add_rule(self.document,lanes,v["subtype"],mem,data.get("tags",{}))
            self.edit("添加交通规则",operation,lambda i:[("relation",i)])

    def crosswalk_rule(self):
        refs=self.selected(lanes=True,count=2)
        crossing=next((i for _,i in refs if self.document.relations[i].tags.get("subtype")=="crosswalk"),None)
        if crossing is None:raise ValueError("两条车道中必须有一条 subtype=crosswalk")
        road=next(i for _,i in refs if i!=crossing)
        self.edit("创建横道规则",lambda:op.add_rule(self.document,[road],"crosswalk",[("relation",crossing,"refers")]),lambda i:[("relation",i)])

    def right_of_way(self):
        refs=self.selected(lanes=True,count=2)
        self.edit("创建优先路权",lambda:op.add_rule(self.document,[i for _,i in refs],"right_of_way",[("relation",refs[0][1],"right_of_way"),("relation",refs[1][1],"yield")]),lambda i:[("relation",i)])

    def may_discard(self):
        if not (self.history.dirty or self.asset_dirty):return True
        reply=QMessageBox.question(self,"未保存修改","保存当前工程后再继续？",
            QMessageBox.StandardButton.Save|QMessageBox.StandardButton.Discard|QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Save)
        if reply==QMessageBox.StandardButton.Cancel:return False
        if reply==QMessageBox.StandardButton.Save:return self.save()
        return True

    def replace_document(self,document):
        self.document=document;self.history=History(document);self.canvas.document=document
        self.canvas.select([],False);self.canvas.draft=[];self.canvas.route_ids=[];self.issue_table.setRowCount(0)
        self.refresh()

    def new_project(self):
        if not self.may_discard():return
        if self.loaders:raise ValueError("点云仍在读取，请等待完成后新建工程")
        self.canvas.clouds=[];self.canvas.image=None;self.canvas.image_spec=None
        self.metadata={};self.metadata_source=None;self.project_path=None;self.asset_dirty=False
        self.replace_document(Document());self.refresh_cloud_list();self.projection_dialog()

    def assets(self):
        result=[{"kind":"cloud","source":c.source,"visible":c.visible} for c in self.canvas.clouds]
        if self.canvas.image_spec:result.append({"kind":"image",**self.canvas.image_spec})
        return result

    def settings(self):
        c=self.canvas
        keys=["show_ids","show_directions","show_grid","background","grid_color","cloud_color",
              "cloud_opacity","map_opacity","cloud_size","max_display","z_min","z_max",
              "intensity_max","snap_grid","snap_px","snap_enabled","cloud_snap","edit_z","image_opacity"]
        return {"view":{k:getattr(c,k) for k in keys},"layers":c.layers,
                "camera":{"target":c.camera.target.tolist(),"scale":c.camera.scale,
                          "yaw":c.camera.yaw,"elevation":c.camera.elevation,
                          "is_3d":c.camera.is_3d,"perspective":c.camera.perspective},
                "lane_width":self.width_spin.value(),"lane_speed":self.speed_spin.value(),
                "pcd_metadata":self.metadata}

    def apply_settings(self,settings):
        # Only known, type-checked keys are accepted from a settings/project file.
        c=self.canvas
        for key,value in settings.get("view",{}).items():
            if key not in self.settings()["view"]:continue
            default=getattr(c,key)
            if isinstance(default,bool):value=bool(value)
            elif isinstance(default,int):value=int(value)
            elif isinstance(default,float):
                value=float(value)
                if not math.isfinite(value):raise ValueError("显示设置含非有限数值")
            elif not isinstance(value,str):continue
            if key in ("cloud_opacity","map_opacity","image_opacity"):value=float(np.clip(value,0,1))
            if key=="cloud_size":value=int(np.clip(value,1,3))
            if key=="max_display":value=int(np.clip(value,1000,1000000))
            setattr(c,key,value)
        for key,value in settings.get("layers",{}).items():
            if key in c.layers:c.layers[key]=bool(value)
        for key,control in self.layer_controls.items():control.setChecked(c.layers[key])
        camera=settings.get("camera",{})
        if "target" in camera:
            t=np.asarray(camera["target"],dtype=float)
            if t.shape!=(3,) or not np.isfinite(t).all():raise ValueError("相机 target 无效")
            c.camera.target=t
        for key in ("scale","yaw","elevation"):
            if key in camera:
                value=float(camera[key])
                if not math.isfinite(value):raise ValueError("相机参数无效")
                setattr(c.camera,key,value)
        c.camera.scale=float(np.clip(c.camera.scale,.001,3000))
        c.camera.elevation=float(np.clip(c.camera.elevation,.15,1.55))
        for key in ("is_3d","perspective"):
            if key in camera:setattr(c.camera,key,bool(camera[key]))
        self.metadata=settings.get("pcd_metadata",{})
        for key,control in self.view_controls.items():control.setValue(getattr(c,key))
        self.edit_z.setValue(c.edit_z);self.snap_check.setChecked(c.snap_enabled)
        self.cloud_snap_check.setChecked(c.cloud_snap);self.perspective.setChecked(c.camera.perspective)
        self.ids_action.setChecked(c.show_ids);self.directions_action.setChecked(c.show_directions);self.grid_action.setChecked(c.show_grid)
        self.width_spin.setValue(settings.get("lane_width",3.5));self.speed_spin.setValue(settings.get("lane_speed",30))
        c.update()

    def save(self,save_as=False):
        path=self.project_path
        if not path or save_as:
            path,_=QFileDialog.getSaveFileName(self,"保存工程",str(path or f"{self.document.name}.vmbqt"),"VMBQt project (*.vmbqt)")
        if not path:return False
        if not path.endswith(".vmbqt"):path+=".vmbqt"
        save_project(path,self.document,self.assets(),self.settings())
        self.project_path=str(Path(path).resolve());self.history.mark_saved();self.asset_dirty=False
        self.refresh();self.log(f"工程已保存：{path}；点云和底图使用相对路径引用")
        return True

    def open_project(self,path=None):
        if not self.may_discard():return
        if self.loaders:raise ValueError("点云仍在读取，请等待完成后打开工程")
        if not path:path,_=QFileDialog.getOpenFileName(self,"打开工程","","VMBQt project (*.vmbqt)")
        if not path:return
        document,assets,settings=load_project(path)
        self.canvas.clouds=[];self.canvas.image=None;self.canvas.image_spec=None
        self.project_path=str(Path(path).resolve());self.asset_dirty=False
        self.replace_document(document);self.apply_settings(settings);self.refresh_cloud_list()
        for asset in assets:
            if not Path(asset["source"]).is_file():self.log(f"缺失资源，请重新导入：{asset['source']}");continue
            if asset["kind"]=="cloud":self.queue_cloud(asset["source"],asset.get("visible",True),dirty=False,fit=False)
            elif asset["kind"]=="image":
                self.canvas.image=QImage(asset["source"]);self.canvas.image_spec=asset
        self.refresh();self.log(f"已打开工程：{path}")

    def import_osm(self,path=None):
        if not self.may_discard():return
        if not path:path,_=QFileDialog.getOpenFileName(self,"导入 Lanelet2（使用当前投影配置）","","Lanelet2 map (*.osm *.xml)")
        if not path:return
        # Read successfully before replacing any current model.
        document=Document.read_osm(path,self.document.projection)
        self.replace_document(document);self.project_path=None;self.asset_dirty=True
        self.canvas.fit_all();self.run_checks();self.log(f"导入 {path}；未知标签和关系保留，未自动修复或丢弃图元")

    def merge_osm(self):
        path,_=QFileDialog.getOpenFileName(self,"合并 OSM（必须与当前投影一致）","","Lanelet2 map (*.osm *.xml)")
        if not path:return
        other=Document.read_osm(path,self.document.projection)
        issues=validate(other)
        if any(i.level=="ERROR" and "缺失" in i.message for i in issues):raise ValueError("待合并地图存在缺失引用，请先修复")
        self.edit("合并地图并重分配 ID",lambda:op.merge_document(self.document,other))

    def export_osm(self):
        path,_=QFileDialog.getSaveFileName(self,"导出 Lanelet2 OSM","lanelet2_map.osm","Lanelet2 map (*.osm)")
        if not path:return
        self.run_checks()
        errors=sum(i.level=="ERROR" for i in self.issues)
        if errors and QMessageBox.warning(self,"地图存在错误",f"内置检查发现 {errors} 个错误。仍导出供调试？",
            QMessageBox.StandardButton.Yes|QMessageBox.StandardButton.No,QMessageBox.StandardButton.No)!=QMessageBox.StandardButton.Yes:return
        self.document.write_osm(path);self.log(f"OSM 已导出：{path}；工程资源和编辑设置请另外保存 .vmbqt")

    def import_clouds(self):
        paths,_=QFileDialog.getOpenFileNames(self,"导入 PCD 文件","","Point cloud (*.pcd)")
        for path in paths:self.queue_cloud(path)

    def import_cloud_directory(self):
        path=QFileDialog.getExistingDirectory(self,"选择包含 PCD 的目录")
        if not path:return
        files=sorted(Path(path).glob("*.pcd"))
        if not files:raise ValueError("目录没有 .pcd 文件（只查找当前层）")
        metadata=Path(path)/"pointcloud_map_metadata.yaml"
        if metadata.is_file():
            value=yaml.safe_load(metadata.read_text(encoding="utf-8"))
            if not isinstance(value,dict):raise ValueError("点云 metadata 必须为 YAML 字典")
            self.metadata=value;self.metadata_source=str(metadata)
        for file in files:self.queue_cloud(str(file))

    def queue_cloud(self,path,visible=True,dirty=True,fit=True):
        path=str(Path(path).resolve())
        if path in [c.source for c in self.canvas.clouds] or path in [p[0] for p in self._pending_clouds] or path in [p.path for p in self.loaders]:return
        self._pending_clouds.append((path,visible,dirty,fit));self._start_next_cloud()

    def _start_next_cloud(self):
        if self.loaders or not self._pending_clouds:return
        path,visible,dirty,fit=self._pending_clouds.pop(0)
        loader=CloudLoader(path,self);self.loaders.append(loader)
        self.log(f"读取点云：{Path(path).name}")
        def loaded(cloud):
            cloud.visible=visible;self.canvas.clouds.append(cloud)
            self.asset_dirty|=dirty;self.refresh_cloud_list();self.refresh()
            if fit:self.canvas.fit_all()
            self.log(f"{Path(path).name}：原始 {cloud.total:,} 点，载入显示样本 {len(cloud.xyz):,} 点")
        loader.loaded.connect(loaded);loader.failed.connect(self.report_error)
        def finished():
            self.loaders.remove(loader);loader.deleteLater();self._start_next_cloud()
        loader.finished.connect(finished);loader.start()

    def refresh_cloud_list(self):
        from PySide6.QtWidgets import QListWidgetItem
        self.cloud_list.blockSignals(True);self.cloud_list.clear()
        for i,c in enumerate(self.canvas.clouds):
            item=QListWidgetItem(f"{Path(c.source).name} · {len(c.xyz):,}/{c.total:,}")
            item.setFlags(item.flags()|Qt.ItemFlag.ItemIsUserCheckable)
            item.setCheckState(Qt.CheckState.Checked if c.visible else Qt.CheckState.Unchecked)
            item.setData(Qt.ItemDataRole.UserRole,i);self.cloud_list.addItem(item)
        self.cloud_list.blockSignals(False)

    def cloud_visibility(self,item):
        i=item.data(Qt.ItemDataRole.UserRole)
        self.canvas.clouds[i].visible=item.checkState()==Qt.CheckState.Checked
        self.canvas.update();self.asset_dirty=True

    def remove_cloud(self):
        row=self.cloud_list.currentRow()
        if 0<=row<len(self.canvas.clouds):
            self.canvas.clouds.pop(row);self.asset_dirty=True;self.refresh_cloud_list();self.refresh()

    def manage_metadata(self):
        if not self.metadata:
            path,_=QFileDialog.getOpenFileName(self,"导入点云 metadata（可取消直接新建）","","YAML (*.yaml *.yml)")
            if path:
                value=yaml.safe_load(Path(path).read_text(encoding="utf-8"))
                if not isinstance(value,dict):raise ValueError("metadata 必须为 YAML 字典")
                self.metadata=value;self.metadata_source=path
        text=text_dialog(self,"点云 Metadata YAML",yaml.safe_dump(self.metadata,sort_keys=False),
            "保留原始 YAML 内容结构；本程序不从显示样本重算 PCD 分块范围。请使用 Autoware 点云分块工具生成真实边界。")
        if text is not None:
            value=yaml.safe_load(text) or {}
            if not isinstance(value,dict):raise ValueError("metadata 必须为 YAML 字典")
            self.metadata=value;self.asset_dirty=True

    def import_csv(self,kind):
        path,_=QFileDialog.getOpenFileName(self,"导入停靠点" if kind=="stop" else "导入轨迹","","CSV (*.csv)")
        if not path:return
        objects=read_csv(path,kind)
        def operation():
            if kind=="stop":self.document.stops={s.id:s for s in objects}
            else:self.document.trajectory=objects
            self.document._next=max([self.document._next]+[o.id+1 for o in objects])
        self.edit("导入 CSV（替换同类数据）",operation);self.canvas.fit_all()

    def export_csv(self,kind):
        path,_=QFileDialog.getSaveFileName(self,"导出 CSV", "stop_points.csv" if kind=="stop" else "trajectory.csv","CSV (*.csv)")
        if path:
            write_csv(path,self.document.stops.values() if kind=="stop" else self.document.trajectory,kind)
            self.log(f"CSV 已导出：{path}（本地格式；FMS 专用列需自行映射）")

    def edit_trajectory(self):self._edit_csv_objects("trajectory")
    def edit_stops(self):self._edit_csv_objects("stop")

    def _edit_csv_objects(self,kind):
        from .model import TrajectoryPoint
        objects=list(self.document.stops.values()) if kind=="stop" else self.document.trajectory
        columns=["id","x","y","z","yaw","name" if kind=="stop" else "velocity","lanelet_id"]
        dialog=QDialog(self);dialog.setWindowTitle("停靠点表" if kind=="stop" else "轨迹点表（按行顺序连接）");dialog.resize(850,500)
        layout=QVBoxLayout(dialog);table=make_table(columns)
        set_rows(table,[[getattr(o,k) for k in columns] for o in objects]);layout.addWidget(table);layout.addLayout(table_buttons(table))
        controls=QDialogButtonBox(QDialogButtonBox.StandardButton.Ok|QDialogButtonBox.StandardButton.Cancel)
        controls.accepted.connect(dialog.accept);controls.rejected.connect(dialog.reject);layout.addWidget(controls)
        if dialog.exec()!=QDialog.DialogCode.Accepted:return
        rows=table_rows(table)
        def operation():
            parsed=[];ids=set();old={o.id:o for o in objects}
            for row in rows:
                if not any(row):continue
                data=dict(zip(columns,row));oid=int(data["id"])
                if oid in ids:raise ValueError("ID 重复")
                ids.add(oid)
                for k in ("x","y","z","yaw"):
                    data[k]=float(data[k])
                    if not math.isfinite(data[k]):raise ValueError("坐标必须为有限数")
                data["id"]=oid;data["lanelet_id"]=int(data["lanelet_id"] or "0")
                if kind=="trajectory":data["velocity"]=float(data["velocity"] or "0")
                cls=StopPoint if kind=="stop" else TrajectoryPoint
                obj=cls(**data);obj.extra=copy.deepcopy(old[oid].extra) if oid in old else {};parsed.append(obj)
            if kind=="stop":self.document.stops={o.id:o for o in parsed}
            else:self.document.trajectory=parsed
            self.document._next=max([self.document._next]+[o.id+1 for o in parsed])
        self.edit("编辑 CSV 点表",operation)

    def import_image(self):
        path,_=QFileDialog.getOpenFileName(self,"选择本地栅格底图","","Images (*.png *.jpg *.jpeg *.tif *.tiff *.bmp)")
        if not path:return
        image=QImage(path)
        if image.isNull():raise ValueError("Qt 无法读取这张图片，请转换为 PNG")
        v=form_dialog(self,"设置底图地面范围",[("x","左下角 X / m",0.),("y","左下角 Y / m",0.),
            ("z","地面 Z / m",0.),("width","地面宽度 / m",100.),("height","地面高度 / m",100.)],
            "图片上方对应 +Y，右方对应 +X；坐标须与点云一致。本版本不自动重投影 GeoTIFF。")
        if v:
            if v["width"]<=0 or v["height"]<=0:raise ValueError("底图宽高必须大于零")
            self.canvas.image=image;self.canvas.image_spec={"source":str(Path(path).resolve()),**v}
            self.asset_dirty=True;self.canvas.update()

    def export_image(self):
        path,_=QFileDialog.getSaveFileName(self,"导出视图","map_view.png","PNG (*.png)")
        if path:
            if not self.canvas.grab().save(path,"PNG"):raise ValueError("无法写入图片")
            self.log(f"视图已导出：{path}")

    def export_bundle(self):
        import tempfile
        parent=QFileDialog.getExistingDirectory(self,"选择导出目录的父文件夹")
        if not parent:return
        v=form_dialog(self,"地图目录名称",[("name","创建新目录","autoware_map")],"导出完整原始 PCD，不使用显示采样替代原始文件；目标目录必须不存在。")
        if not v:return
        name=v["name"].strip()
        if not name or name in (".","..") or Path(name).name!=name:raise ValueError("请输入一个目录名")
        target=Path(parent)/name
        if target.exists():raise ValueError("目标目录已存在，请使用新的名称")
        self.run_checks()
        if any(i.level=="ERROR" for i in self.issues):raise ValueError("请先修复内置检查中的 ERROR；调试用途可单独导出 OSM")
        clouds=self.canvas.clouds
        if len(clouds)>1 and not self.metadata:raise ValueError("多块点云导出需要原始 pointcloud_map_metadata.yaml")
        basenames=[Path(c.source).name for c in clouds]
        if len(set(basenames))!=len(basenames):raise ValueError("点云存在重名文件，请先在不同名称下整理并重新加载")
        staging=Path(tempfile.mkdtemp(prefix=".vmbqt-export-",dir=parent))
        try:
            self.document.write_osm(staging/"lanelet2_map.osm")
            self.document.projection.write_yaml(staging/"map_projector_info.yaml")
            if len(clouds)==1:shutil.copy2(clouds[0].source,staging/"pointcloud_map.pcd")
            elif clouds:
                tiles=staging/"pointcloud_map";tiles.mkdir()
                for c in clouds:shutil.copy2(c.source,tiles/Path(c.source).name)
                atomic_text(staging/"pointcloud_map_metadata.yaml",yaml.safe_dump(self.metadata,sort_keys=False))
            if self.document.stops:write_csv(staging/"stop_points.csv",self.document.stops.values(),"stop")
            if self.document.trajectory:write_csv(staging/"trajectory.csv",self.document.trajectory)
            atomic_text(staging/"EDITOR_NOTES.txt","Generated by VMBQt 0.1. Run the target Autoware validator and simulator before use.\n")
            os.rename(staging,target)
        finally:
            if staging.exists():shutil.rmtree(staging)
        self.log(f"地图目录已导出：{target}；尚需在你的 Autoware 版本下验证")

    def projection_dialog(self):
        p=self.document.projection
        types=[p.projector_type]+[t for t in ("Local","MGRS","LocalCartesianUTM","TransverseMercator") if t!=p.projector_type]
        v=form_dialog(self,"投影 / 地图设置",[("name","地图名称",self.document.name),
            ("type","projector_type",types),("lat","原点纬度 / deg",p.latitude),
            ("lon","原点经度 / deg",p.longitude),("grid","MGRS grid",p.mgrs_grid),
            ("scale","TM scale_factor",p.scale_factor)],
            "这里修改坐标系声明，现有 XY 不重投影。必须与 PCD 的真实坐标系一致。Local 无法计算经纬度；Z 始终按 WGS84 绝对椭球高处理。")
        if v:
            config=ProjectionConfig(v["type"],v["lat"],v["lon"],v["grid"].upper(),v["scale"])
            Projector(config)
            def operation():self.document.projection=config;self.document.name=v["name"] or "Untitled"
            self.edit("设置地图投影",operation)

    def import_projection(self):
        path,_=QFileDialog.getOpenFileName(self,"导入投影 YAML","","YAML (*.yaml *.yml)")
        if path:
            p=ProjectionConfig.from_yaml(path);Projector(p)
            self.edit("导入投影声明（XY 不变）",lambda:setattr(self.document,"projection",p))

    def export_projection(self):
        path,_=QFileDialog.getSaveFileName(self,"导出投影 YAML","map_projector_info.yaml","YAML (*.yaml)")
        if path:self.document.projection.write_yaml(path);self.log(f"投影已导出：{path}")

    def map_meta(self):
        current=next((ET.fromstring(s).attrib for s in self.document.root_extras if ET.fromstring(s).tag=="MapMetaInfo"),{})
        text=text_dialog(self,"MapMetaInfo",json.dumps(current,indent=2,ensure_ascii=False),"仅编辑地图元信息，不会伪造验证签名。")
        if text is not None:
            data=json.loads(text)
            if not isinstance(data,dict) or not all(isinstance(k,str) and isinstance(v,str) for k,v in data.items()):raise ValueError("必须为字符串键和值的 JSON 对象")
            def operation():
                self.document.root_extras=[s for s in self.document.root_extras if ET.fromstring(s).tag!="MapMetaInfo"]
                self.document.root_extras.append(ET.tostring(ET.Element("MapMetaInfo",data),encoding="unicode"))
            self.edit("修改 MapMetaInfo",operation)

    def goto_coordinate(self):
        v=form_dialog(self,"跳转到坐标",[("mode","坐标类型",["XY","纬度经度"]),("a","X 或纬度",0.),("b","Y 或经度",0.)])
        if v:
            xy=(v["a"],v["b"]) if v["mode"]=="XY" else Projector(self.document.projection).forward(v["a"],v["b"])
            self.canvas.camera.target[:2]=xy;self.canvas.update()

    def set_view(self,key,value,camera=False):
        setattr(self.canvas.camera if camera else self.canvas,key,value);self.canvas.update()

    def set_layer(self,key,value):self.canvas.layers[key]=value;self.canvas.update()
    def set_3d(self,value):self.canvas.camera.is_3d=value;self.canvas.update()
    def reset_rotation(self):self.canvas.camera.yaw=0.;self.canvas.camera.elevation=math.radians(50);self.canvas.update()

    def edit_colors(self):
        from PySide6.QtWidgets import QColorDialog
        from PySide6.QtGui import QColor
        color=QColorDialog.getColor(QColor(self.canvas.background),self,"背景颜色")
        if color.isValid():self.canvas.background=color.name()
        color=QColorDialog.getColor(QColor(self.canvas.grid_color),self,"网格颜色")
        if color.isValid():self.canvas.grid_color=color.name()
        self.canvas.update()

    def export_settings(self):
        path,_=QFileDialog.getSaveFileName(self,"保存显示设置","view_settings.json","JSON (*.json)")
        if path:atomic_text(path,json.dumps(self.settings(),ensure_ascii=False,indent=2,allow_nan=False))

    def import_settings(self):
        path,_=QFileDialog.getOpenFileName(self,"读取显示设置","","JSON (*.json)")
        if path:self.apply_settings(json.loads(Path(path).read_text(encoding="utf-8")))

    def run_checks(self):
        self.issues=validate(self.document)
        set_rows(self.issue_table,[(i.level,f"{i.ref[0]}/{i.ref[1]}",i.message) for i in self.issues])
        self.log(f"内置检查：{sum(i.level=='ERROR' for i in self.issues)} 错误，{sum(i.level=='WARNING' for i in self.issues)} 提示（不等同 Autoware 全量验证）")
        return self.issues

    def focus_issue(self,row,column):
        if hasattr(self,"issues") and row<len(self.issues):self.canvas.select([self.issues[row].ref]);self.canvas.focus_refs()

    def compare_maps(self):
        path,_=QFileDialog.getOpenFileName(self,"选择比较地图（按 OSM 类型和 ID）","","Lanelet2 (*.osm *.xml)")
        if not path:return
        other=Document.read_osm(path,self.document.projection);changes=compare(self.document,other)
        dialog=QDialog(self);dialog.setWindowTitle("地图差异（当前 → 导入地图）");dialog.resize(900,630)
        layout=QVBoxLayout(dialog);table=make_table(["应用","变化","对象"])
        from PySide6.QtWidgets import QTableWidgetItem
        table.setRowCount(len(changes))
        for row,(status,ref) in enumerate(changes):
            item=QTableWidgetItem();item.setFlags(Qt.ItemFlag.ItemIsEnabled|Qt.ItemFlag.ItemIsUserCheckable);item.setCheckState(Qt.CheckState.Unchecked)
            table.setItem(row,0,item);table.setItem(row,1,QTableWidgetItem(status));table.setItem(row,2,QTableWidgetItem(f"{ref[0]}/{ref[1]}"))
        layout.addWidget(table);preview=QPlainTextEdit();preview.setReadOnly(True);layout.addWidget(preview)
        def show(row,col):
            if row>=len(changes):return
            _,ref=changes[row]
            from dataclasses import asdict
            a=self.document.get(ref);b=other.get(ref)
            before=json.dumps(asdict(a) if a else {},ensure_ascii=False,indent=2).splitlines()
            after=json.dumps(asdict(b) if b else {},ensure_ascii=False,indent=2).splitlines()
            preview.setPlainText("\n".join(difflib.unified_diff(before,after,fromfile="当前",tofile="导入")))
            if a:self.canvas.select([ref]);self.canvas.focus_refs()
        table.cellClicked.connect(show)
        label=QLabel("按 ID 比较，不进行几何匹配。只应用勾选项；依赖项需一并勾选。操作可撤销。")
        label.setWordWrap(True);layout.addWidget(label)
        buttons=QDialogButtonBox(QDialogButtonBox.StandardButton.Apply|QDialogButtonBox.StandardButton.Cancel)
        buttons.button(QDialogButtonBox.StandardButton.Apply).clicked.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject);layout.addWidget(buttons)
        if dialog.exec()!=QDialog.DialogCode.Accepted:return
        selected=[changes[i] for i in range(len(changes)) if table.item(i,0).checkState()==Qt.CheckState.Checked]
        def operation():
            for status,(kind,oid) in selected:
                if status=="删除":self.document.table(kind).pop(oid,None)
                else:self.document.table(kind)[oid]=copy.deepcopy(other.get((kind,oid)))
            for w in self.document.ways.values():
                if any(i not in self.document.nodes for i in w.nodes):raise ValueError("应用差异会产生缺失节点；请同时选择依赖项")
            for r in self.document.relations.values():
                if any(m.type not in ("node","way","relation") or self.document.get((m.type,m.ref)) is None for m in r.members):raise ValueError("应用差异会产生缺失关系成员；请同时选择依赖项")
            self.document._next=max([0,*self.document.nodes,*self.document.ways,*self.document.relations,*self.document.stops])+1
        self.edit("应用地图差异",operation);self.run_checks()

    def external_validator(self):
        if self.external_process and self.external_process.state()!=QProcess.ProcessState.NotRunning:raise ValueError("外部验证器正在运行")
        v=form_dialog(self,"启动外部 Autoware 验证器",[("command","命令（{map} 替换为临时 OSM 路径）",
            "ros2 run autoware_lanelet2_map_validator autoware_lanelet2_map_validator -p mgrs -m {map}")],
            "需从已 source ROS2/Autoware 的终端启动本编辑器。此处直接启动程序，不经过 shell。请按真实投影修改 -p 参数；日志显示在下方。验证的是临时副本。")
        if not v:return
        directory=Path(QStandardPaths.writableLocation(QStandardPaths.StandardLocation.CacheLocation))/"validator"
        directory.mkdir(parents=True,exist_ok=True);path=directory/"lanelet2_map.osm";self.document.write_osm(path)
        arguments=[a.replace("{map}",str(path)) for a in shlex.split(v["command"])]
        if not arguments:raise ValueError("命令不能为空")
        process=QProcess(self);self.external_process=process
        process.setWorkingDirectory(str(directory));process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        process.readyReadStandardOutput.connect(lambda:self.log(bytes(process.readAllStandardOutput()).decode("utf-8",errors="replace")))
        process.errorOccurred.connect(lambda err:self.log(f"外部程序错误：{process.errorString()}"))
        process.finished.connect(lambda code,status:self.log(f"外部验证器结束，退出码 {code}；详细结果请按该验证器版本说明读取"))
        process.start(arguments[0],arguments[1:]);self.log("启动外部验证器："+" ".join(arguments))

    def autosave(self):
        if not (self.history.dirty or self.asset_dirty):return
        try:
            self.recovery_path.parent.mkdir(parents=True,exist_ok=True)
            save_project(self.recovery_path,self.document,self.assets(),self.settings())
        except Exception as exc:self.log(f"恢复文件保存失败：{exc}")

    def show_help(self):
        text=("快速操作\n\n1. 地图菜单设置投影；导入点云。\n"
              "2. 选择车道工具，在中心位置依次单击，Enter 完成。\n"
              "3. 选择类型切换到车道；Shift 多选；属性中改标签。\n"
              "4. 选车道后添加停止线、交通灯或检测区域。\n"
              "5. 2D/3D 共用 XYZ；中键平移、右键旋转、滚轮缩放。\n"
              "6. F7 检查；保存工程；导出 OSM 和投影 YAML。\n\n"
              "范围与差异\n\n这是原创 Qt6/PySide6 离线工程，不是 TIER IV 全功能等价产品。\n"
              "3D 使用 Qt 软件投影和点云逐像素深度测试，矢量叠加层保持可见；不适合数十亿点。\n"
              "本地拓扑寻路不包含换道或完整交通规则；未实现 FMS、账号、在线卫星图、Autoware Plan Route/Path。\n"
              "虚拟信号、BusStop 等高级规则提供通用成员编辑，不提供等价一键生成。\n"
              "PCD 显示为样本；高度采样基于样本。导出地图目录复制完整原始 PCD。\n"
              "共享边界的部分拆分/变宽操作会被阻止；见 docs/FEATURE_MATRIX_zh.md。\n\n"
              "完整安装、练习、兼容性说明见随包 README_zh.md。")
        text_dialog(self,"操作说明",text,readonly=True)

    def closeEvent(self,event):
        if self.testing:event.accept();return
        if self.loaders:
            self.report_error("点云仍在读取，请等待当前导入完成后退出");event.ignore();return
        if self.external_process and self.external_process.state()!=QProcess.ProcessState.NotRunning:
            self.report_error("外部验证器仍在运行，请等待完成后退出");event.ignore();return
        if self.may_discard():
            self.recovery_path.unlink(missing_ok=True);event.accept()
        else:event.ignore()


def run(path=None):
    app=QApplication.instance() or QApplication([])
    app.setApplicationName("VMBQt");app.setOrganizationName("VMBQt-Offline")
    apply_theme(app)
    window=MainWindow();window.show()
    if path:
        if str(path).endswith(".vmbqt"):window._safe(lambda:window.open_project(path))
        elif str(path).endswith(".osm"):window._safe(lambda:window.import_osm(path))
    elif window.recovery_path.is_file():
        if QMessageBox.question(window,"发现恢复工程","恢复上次自动保存的未完成编辑？") == QMessageBox.StandardButton.Yes:
            window._safe(lambda:window.open_project(str(window.recovery_path)))
    return app.exec()
