"""Small reusable Qt dialogs with explicit validation by the calling operation."""
import json
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QDialog, QVBoxLayout, QFormLayout, QDialogButtonBox,
    QLineEdit, QDoubleSpinBox, QComboBox, QPlainTextEdit, QLabel, QTableWidget,
    QTableWidgetItem, QHeaderView, QPushButton, QHBoxLayout, QCompleter,
    QStyledItemDelegate)


def number(value=0, low=-1e9, high=1e9, decimals=4):
    widget=QDoubleSpinBox();widget.setRange(low,high);widget.setDecimals(decimals)
    widget.setValue(value);widget.setSingleStep(.1);return widget


def form_dialog(parent,title,fields,help_text=""):
    dialog=QDialog(parent);dialog.setWindowTitle(title);dialog.setMinimumWidth(440)
    layout=QVBoxLayout(dialog)
    if help_text:
        help_label=QLabel(help_text);help_label.setWordWrap(True);layout.addWidget(help_label)
    form=QFormLayout();widgets={}
    for key,label,value in fields:
        if isinstance(value,list):
            widget=QComboBox();widget.addItems(value)
        elif isinstance(value,(int,float)):
            widget=number(value)
        else:widget=QLineEdit(str(value))
        widgets[key]=widget;form.addRow(label,widget)
    layout.addLayout(form)
    buttons=QDialogButtonBox(QDialogButtonBox.StandardButton.Ok|QDialogButtonBox.StandardButton.Cancel)
    buttons.accepted.connect(dialog.accept);buttons.rejected.connect(dialog.reject);layout.addWidget(buttons)
    if dialog.exec()!=QDialog.DialogCode.Accepted:return None
    return {k:w.value() if isinstance(w,QDoubleSpinBox) else w.currentText()
            if isinstance(w,QComboBox) else w.text() for k,w in widgets.items()}


def text_dialog(parent,title,text,help_text="",readonly=False):
    dialog=QDialog(parent);dialog.setWindowTitle(title);dialog.resize(740,520)
    layout=QVBoxLayout(dialog)
    if help_text:
        label=QLabel(help_text);label.setWordWrap(True);layout.addWidget(label)
    editor=QPlainTextEdit();editor.setPlainText(text);editor.setReadOnly(readonly);layout.addWidget(editor)
    buttons=QDialogButtonBox(QDialogButtonBox.StandardButton.Close if readonly else
                            QDialogButtonBox.StandardButton.Ok|QDialogButtonBox.StandardButton.Cancel)
    buttons.accepted.connect(dialog.accept);buttons.rejected.connect(dialog.reject);layout.addWidget(buttons)
    if dialog.exec()==QDialog.DialogCode.Accepted:return editor.toPlainText()
    return None


TAG_KEYS=["type","subtype","location","one_way","speed_limit","lane_change",
          "turn_direction","turn_signal_distance","participant:vehicle","participant:pedestrian",
          "participant:bicycle","height","color","arrow","traffic_light_id","sign_type",
          "no_drivable_lane","fms_lane_passable","area","name"]
TAG_VALUES=["yes","no","urban","road","crosswalk","walkway","road_shoulder","bicycle_lane",
            "line_thin","line_thick","virtual","solid","dashed","solid_solid","dashed_dashed",
            "curbstone","guard_rail","road_border","stop_line","traffic_light","light_bulbs",
            "parking_space","parking_lot","intersection_area","detection_area","speed_bump",
            "no_stopping_area","no_parking_area","bus_stop_area","left","right","straight",
            "red","yellow","green","red_yellow_green","stop_sign"]


class TagDelegate(QStyledItemDelegate):
    def createEditor(self,parent,option,index):
        edit=QLineEdit(parent);completer=QCompleter(TAG_KEYS if index.column()==0 else TAG_VALUES,edit)
        completer.setCaseSensitivity(Qt.CaseSensitivity.CaseInsensitive)
        edit.setCompleter(completer);return edit


def make_table(columns):
    table=QTableWidget(0,len(columns));table.setHorizontalHeaderLabels(columns)
    table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
    table.verticalHeader().setVisible(False)
    table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
    return table


def set_rows(table,rows):
    table.setRowCount(len(rows))
    for row,values in enumerate(rows):
        for col,value in enumerate(values):table.setItem(row,col,QTableWidgetItem(str(value)))


def table_rows(table):
    return [[table.item(r,c).text().strip() if table.item(r,c) else ""
             for c in range(table.columnCount())] for r in range(table.rowCount())]


def table_buttons(table):
    layout=QHBoxLayout();add=QPushButton("＋ 添加行");remove=QPushButton("－ 删除行")
    add.clicked.connect(lambda:table.insertRow(table.rowCount()))
    def delete():
        for row in sorted({i.row() for i in table.selectedIndexes()},reverse=True):table.removeRow(row)
    remove.clicked.connect(delete);layout.addWidget(add);layout.addWidget(remove);return layout
