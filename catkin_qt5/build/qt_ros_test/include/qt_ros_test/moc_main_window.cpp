/****************************************************************************
** Meta object code from reading C++ file 'main_window.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/qt_ros_test/include/qt_ros_test/main_window.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'main_window.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_qt_ros_test__MainWindow_t {
    QByteArrayData data[68];
    char stringdata0[1595];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_qt_ros_test__MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_qt_ros_test__MainWindow_t qt_meta_stringdata_qt_ros_test__MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 23), // "qt_ros_test::MainWindow"
QT_MOC_LITERAL(1, 24, 7), // "sigSend"
QT_MOC_LITERAL(2, 32, 0), // ""
QT_MOC_LITERAL(3, 33, 6), // "strMsg"
QT_MOC_LITERAL(4, 40, 24), // "on_actionAbout_triggered"
QT_MOC_LITERAL(5, 65, 25), // "on_button_connect_clicked"
QT_MOC_LITERAL(6, 91, 5), // "check"
QT_MOC_LITERAL(7, 97, 40), // "on_checkbox_use_environment_s..."
QT_MOC_LITERAL(8, 138, 5), // "state"
QT_MOC_LITERAL(9, 144, 13), // "slot_cmd_read"
QT_MOC_LITERAL(10, 158, 12), // "slot_cmd_err"
QT_MOC_LITERAL(11, 171, 16), // "slot_rosShutdown"
QT_MOC_LITERAL(12, 188, 12), // "slot_rosOpen"
QT_MOC_LITERAL(13, 201, 23), // "on_checkBox_key_clicked"
QT_MOC_LITERAL(14, 225, 7), // "checked"
QT_MOC_LITERAL(15, 233, 13), // "pushButton_go"
QT_MOC_LITERAL(16, 247, 15), // "pushButton_stop"
QT_MOC_LITERAL(17, 263, 12), // "updata_image"
QT_MOC_LITERAL(18, 276, 16), // "slotbatteryState"
QT_MOC_LITERAL(19, 293, 14), // "slotspeedState"
QT_MOC_LITERAL(20, 308, 15), // "slotDataArrived"
QT_MOC_LITERAL(21, 324, 5), // "strIp"
QT_MOC_LITERAL(22, 330, 5), // "nPort"
QT_MOC_LITERAL(23, 336, 33), // "on_horizontalSlider_v_sliderM..."
QT_MOC_LITERAL(24, 370, 8), // "position"
QT_MOC_LITERAL(25, 379, 33), // "on_horizontalSlider_c_sliderM..."
QT_MOC_LITERAL(26, 413, 31), // "on_checkBox__opencamera_clicked"
QT_MOC_LITERAL(27, 445, 34), // "on_pushButton_linefollower_cl..."
QT_MOC_LITERAL(28, 480, 35), // "on_pushButton_laserfollower_c..."
QT_MOC_LITERAL(29, 516, 21), // "on_pushButton_clicked"
QT_MOC_LITERAL(30, 538, 28), // "on_pushButton_subimg_clicked"
QT_MOC_LITERAL(31, 567, 33), // "on_pushButton_visfollower_cli..."
QT_MOC_LITERAL(32, 601, 27), // "on_pushButton_xfmic_clicked"
QT_MOC_LITERAL(33, 629, 30), // "on_pushButton_kcftrack_clicked"
QT_MOC_LITERAL(34, 660, 31), // "on_pushButton_webusbcam_clicked"
QT_MOC_LITERAL(35, 692, 29), // "on_pushButton_arlabel_clicked"
QT_MOC_LITERAL(36, 722, 32), // "on_pushButton_arfollower_clicked"
QT_MOC_LITERAL(37, 755, 33), // "on_pushButton_joy_control_cli..."
QT_MOC_LITERAL(38, 789, 27), // "on_pushButton_2dmap_clicked"
QT_MOC_LITERAL(39, 817, 31), // "on_pushButton_2dmapsave_clicked"
QT_MOC_LITERAL(40, 849, 27), // "on_pushButton_2dnav_clicked"
QT_MOC_LITERAL(41, 877, 27), // "on_pushButton_3dmap_clicked"
QT_MOC_LITERAL(42, 905, 27), // "on_pushButton_3dnav_clicked"
QT_MOC_LITERAL(43, 933, 31), // "on_pushButton_pure3dmap_clicked"
QT_MOC_LITERAL(44, 965, 31), // "on_pushButton_pure3dnav_clicked"
QT_MOC_LITERAL(45, 997, 25), // "on_pushButton_rrt_clicked"
QT_MOC_LITERAL(46, 1023, 30), // "on_pushButton_moveitik_clicked"
QT_MOC_LITERAL(47, 1054, 30), // "on_pushButton_moveitfk_clicked"
QT_MOC_LITERAL(48, 1085, 32), // "on_pushButton_moveitpick_clicked"
QT_MOC_LITERAL(49, 1118, 34), // "on_pushButton_moveitpick_2_cl..."
QT_MOC_LITERAL(50, 1153, 23), // "on_pushButton_2_clicked"
QT_MOC_LITERAL(51, 1177, 29), // "on_lineEdit_cmd_returnPressed"
QT_MOC_LITERAL(52, 1207, 30), // "on_pushButton_nfsrobot_clicked"
QT_MOC_LITERAL(53, 1238, 31), // "on_pushButton_unfsrobot_clicked"
QT_MOC_LITERAL(54, 1270, 28), // "on_pushButton_nfsarm_clicked"
QT_MOC_LITERAL(55, 1299, 29), // "on_pushButton_unfsarm_clicked"
QT_MOC_LITERAL(56, 1329, 29), // "on_pushButton_settime_clicked"
QT_MOC_LITERAL(57, 1359, 32), // "on_pushButton_catkinmake_clicked"
QT_MOC_LITERAL(58, 1392, 35), // "on_pushButton_catkinmakepkg_c..."
QT_MOC_LITERAL(59, 1428, 29), // "on_pushButton_pkgmove_clicked"
QT_MOC_LITERAL(60, 1458, 25), // "on_pushButton_rqt_clicked"
QT_MOC_LITERAL(61, 1484, 26), // "on_pushButton_rviz_clicked"
QT_MOC_LITERAL(62, 1511, 13), // "keyPressEvent"
QT_MOC_LITERAL(63, 1525, 10), // "QKeyEvent*"
QT_MOC_LITERAL(64, 1536, 5), // "event"
QT_MOC_LITERAL(65, 1542, 15), // "keyReleaseEvent"
QT_MOC_LITERAL(66, 1558, 11), // "slottimeout"
QT_MOC_LITERAL(67, 1570, 24) // "on_quit_button_2_clicked"

    },
    "qt_ros_test::MainWindow\0sigSend\0\0"
    "strMsg\0on_actionAbout_triggered\0"
    "on_button_connect_clicked\0check\0"
    "on_checkbox_use_environment_stateChanged\0"
    "state\0slot_cmd_read\0slot_cmd_err\0"
    "slot_rosShutdown\0slot_rosOpen\0"
    "on_checkBox_key_clicked\0checked\0"
    "pushButton_go\0pushButton_stop\0"
    "updata_image\0slotbatteryState\0"
    "slotspeedState\0slotDataArrived\0strIp\0"
    "nPort\0on_horizontalSlider_v_sliderMoved\0"
    "position\0on_horizontalSlider_c_sliderMoved\0"
    "on_checkBox__opencamera_clicked\0"
    "on_pushButton_linefollower_clicked\0"
    "on_pushButton_laserfollower_clicked\0"
    "on_pushButton_clicked\0"
    "on_pushButton_subimg_clicked\0"
    "on_pushButton_visfollower_clicked\0"
    "on_pushButton_xfmic_clicked\0"
    "on_pushButton_kcftrack_clicked\0"
    "on_pushButton_webusbcam_clicked\0"
    "on_pushButton_arlabel_clicked\0"
    "on_pushButton_arfollower_clicked\0"
    "on_pushButton_joy_control_clicked\0"
    "on_pushButton_2dmap_clicked\0"
    "on_pushButton_2dmapsave_clicked\0"
    "on_pushButton_2dnav_clicked\0"
    "on_pushButton_3dmap_clicked\0"
    "on_pushButton_3dnav_clicked\0"
    "on_pushButton_pure3dmap_clicked\0"
    "on_pushButton_pure3dnav_clicked\0"
    "on_pushButton_rrt_clicked\0"
    "on_pushButton_moveitik_clicked\0"
    "on_pushButton_moveitfk_clicked\0"
    "on_pushButton_moveitpick_clicked\0"
    "on_pushButton_moveitpick_2_clicked\0"
    "on_pushButton_2_clicked\0"
    "on_lineEdit_cmd_returnPressed\0"
    "on_pushButton_nfsrobot_clicked\0"
    "on_pushButton_unfsrobot_clicked\0"
    "on_pushButton_nfsarm_clicked\0"
    "on_pushButton_unfsarm_clicked\0"
    "on_pushButton_settime_clicked\0"
    "on_pushButton_catkinmake_clicked\0"
    "on_pushButton_catkinmakepkg_clicked\0"
    "on_pushButton_pkgmove_clicked\0"
    "on_pushButton_rqt_clicked\0"
    "on_pushButton_rviz_clicked\0keyPressEvent\0"
    "QKeyEvent*\0event\0keyReleaseEvent\0"
    "slottimeout\0on_quit_button_2_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_qt_ros_test__MainWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      57,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,  299,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       4,    0,  302,    2, 0x0a /* Public */,
       5,    1,  303,    2, 0x0a /* Public */,
       7,    1,  306,    2, 0x0a /* Public */,
       9,    0,  309,    2, 0x0a /* Public */,
      10,    0,  310,    2, 0x0a /* Public */,
      11,    0,  311,    2, 0x0a /* Public */,
      12,    0,  312,    2, 0x0a /* Public */,
      13,    1,  313,    2, 0x08 /* Private */,
      15,    0,  316,    2, 0x08 /* Private */,
      16,    0,  317,    2, 0x08 /* Private */,
      17,    1,  318,    2, 0x08 /* Private */,
      18,    1,  321,    2, 0x08 /* Private */,
      19,    6,  324,    2, 0x08 /* Private */,
      20,    3,  337,    2, 0x08 /* Private */,
      23,    1,  344,    2, 0x08 /* Private */,
      25,    1,  347,    2, 0x08 /* Private */,
      26,    1,  350,    2, 0x08 /* Private */,
      27,    1,  353,    2, 0x08 /* Private */,
      28,    1,  356,    2, 0x08 /* Private */,
      29,    1,  359,    2, 0x08 /* Private */,
      30,    1,  362,    2, 0x08 /* Private */,
      31,    1,  365,    2, 0x08 /* Private */,
      32,    1,  368,    2, 0x08 /* Private */,
      33,    1,  371,    2, 0x08 /* Private */,
      34,    1,  374,    2, 0x08 /* Private */,
      35,    1,  377,    2, 0x08 /* Private */,
      36,    1,  380,    2, 0x08 /* Private */,
      37,    1,  383,    2, 0x08 /* Private */,
      38,    1,  386,    2, 0x08 /* Private */,
      39,    1,  389,    2, 0x08 /* Private */,
      40,    1,  392,    2, 0x08 /* Private */,
      41,    1,  395,    2, 0x08 /* Private */,
      42,    1,  398,    2, 0x08 /* Private */,
      43,    1,  401,    2, 0x08 /* Private */,
      44,    1,  404,    2, 0x08 /* Private */,
      45,    1,  407,    2, 0x08 /* Private */,
      46,    1,  410,    2, 0x08 /* Private */,
      47,    1,  413,    2, 0x08 /* Private */,
      48,    1,  416,    2, 0x08 /* Private */,
      49,    1,  419,    2, 0x08 /* Private */,
      50,    1,  422,    2, 0x08 /* Private */,
      51,    0,  425,    2, 0x08 /* Private */,
      52,    1,  426,    2, 0x08 /* Private */,
      53,    1,  429,    2, 0x08 /* Private */,
      54,    1,  432,    2, 0x08 /* Private */,
      55,    1,  435,    2, 0x08 /* Private */,
      56,    1,  438,    2, 0x08 /* Private */,
      57,    1,  441,    2, 0x08 /* Private */,
      58,    1,  444,    2, 0x08 /* Private */,
      59,    1,  447,    2, 0x08 /* Private */,
      60,    1,  450,    2, 0x08 /* Private */,
      61,    1,  453,    2, 0x08 /* Private */,
      62,    1,  456,    2, 0x08 /* Private */,
      65,    1,  459,    2, 0x08 /* Private */,
      66,    0,  462,    2, 0x08 /* Private */,
      67,    1,  463,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    3,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    6,
    QMetaType::Void, QMetaType::Int,    8,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QImage,    2,
    QMetaType::Void, QMetaType::Double,    2,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double,    2,    2,    2,    2,    2,    2,
    QMetaType::Void, QMetaType::QString, QMetaType::QString, QMetaType::Int,    3,   21,   22,
    QMetaType::Void, QMetaType::Int,   24,
    QMetaType::Void, QMetaType::Int,   24,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, QMetaType::Bool,   14,
    QMetaType::Void, 0x80000000 | 63,   64,
    QMetaType::Void, 0x80000000 | 63,   64,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   14,

       0        // eod
};

void qt_ros_test::MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MainWindow *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->sigSend((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->on_actionAbout_triggered(); break;
        case 2: _t->on_button_connect_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->on_checkbox_use_environment_stateChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->slot_cmd_read(); break;
        case 5: _t->slot_cmd_err(); break;
        case 6: _t->slot_rosShutdown(); break;
        case 7: _t->slot_rosOpen(); break;
        case 8: _t->on_checkBox_key_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 9: _t->pushButton_go(); break;
        case 10: _t->pushButton_stop(); break;
        case 11: _t->updata_image((*reinterpret_cast< QImage(*)>(_a[1]))); break;
        case 12: _t->slotbatteryState((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 13: _t->slotspeedState((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< double(*)>(_a[5])),(*reinterpret_cast< double(*)>(_a[6]))); break;
        case 14: _t->slotDataArrived((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3]))); break;
        case 15: _t->on_horizontalSlider_v_sliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 16: _t->on_horizontalSlider_c_sliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 17: _t->on_checkBox__opencamera_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 18: _t->on_pushButton_linefollower_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 19: _t->on_pushButton_laserfollower_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 20: _t->on_pushButton_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 21: _t->on_pushButton_subimg_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 22: _t->on_pushButton_visfollower_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 23: _t->on_pushButton_xfmic_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 24: _t->on_pushButton_kcftrack_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 25: _t->on_pushButton_webusbcam_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 26: _t->on_pushButton_arlabel_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 27: _t->on_pushButton_arfollower_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 28: _t->on_pushButton_joy_control_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 29: _t->on_pushButton_2dmap_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 30: _t->on_pushButton_2dmapsave_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 31: _t->on_pushButton_2dnav_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 32: _t->on_pushButton_3dmap_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 33: _t->on_pushButton_3dnav_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 34: _t->on_pushButton_pure3dmap_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 35: _t->on_pushButton_pure3dnav_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 36: _t->on_pushButton_rrt_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 37: _t->on_pushButton_moveitik_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 38: _t->on_pushButton_moveitfk_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 39: _t->on_pushButton_moveitpick_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 40: _t->on_pushButton_moveitpick_2_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 41: _t->on_pushButton_2_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 42: _t->on_lineEdit_cmd_returnPressed(); break;
        case 43: _t->on_pushButton_nfsrobot_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 44: _t->on_pushButton_unfsrobot_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 45: _t->on_pushButton_nfsarm_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 46: _t->on_pushButton_unfsarm_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 47: _t->on_pushButton_settime_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 48: _t->on_pushButton_catkinmake_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 49: _t->on_pushButton_catkinmakepkg_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 50: _t->on_pushButton_pkgmove_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 51: _t->on_pushButton_rqt_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 52: _t->on_pushButton_rviz_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 53: _t->keyPressEvent((*reinterpret_cast< QKeyEvent*(*)>(_a[1]))); break;
        case 54: _t->keyReleaseEvent((*reinterpret_cast< QKeyEvent*(*)>(_a[1]))); break;
        case 55: _t->slottimeout(); break;
        case 56: _t->on_quit_button_2_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (MainWindow::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::sigSend)) {
                *result = 0;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject qt_ros_test::MainWindow::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_meta_stringdata_qt_ros_test__MainWindow.data,
    qt_meta_data_qt_ros_test__MainWindow,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *qt_ros_test::MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *qt_ros_test::MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_qt_ros_test__MainWindow.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int qt_ros_test::MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 57)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 57;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 57)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 57;
    }
    return _id;
}

// SIGNAL 0
void qt_ros_test::MainWindow::sigSend(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
