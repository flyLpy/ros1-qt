/****************************************************************************
** Meta object code from reading C++ file 'CConnectionForSshClient.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/qt_ros_test/include/qt_ros_test/CConnectionForSshClient.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'CConnectionForSshClient.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_CConnectionForSshClient_t {
    QByteArrayData data[26];
    char stringdata0[362];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CConnectionForSshClient_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CConnectionForSshClient_t qt_meta_stringdata_CConnectionForSshClient = {
    {
QT_MOC_LITERAL(0, 0, 23), // "CConnectionForSshClient"
QT_MOC_LITERAL(1, 24, 15), // "sigInitForClild"
QT_MOC_LITERAL(2, 40, 0), // ""
QT_MOC_LITERAL(3, 41, 22), // "sigConnectStateChanged"
QT_MOC_LITERAL(4, 64, 6), // "bState"
QT_MOC_LITERAL(5, 71, 5), // "strIp"
QT_MOC_LITERAL(6, 77, 5), // "nPort"
QT_MOC_LITERAL(7, 83, 14), // "sigDataArrived"
QT_MOC_LITERAL(8, 98, 6), // "strMsg"
QT_MOC_LITERAL(9, 105, 19), // "slotResetConnection"
QT_MOC_LITERAL(10, 125, 9), // "strIpPort"
QT_MOC_LITERAL(11, 135, 8), // "slotSend"
QT_MOC_LITERAL(12, 144, 10), // "strMessage"
QT_MOC_LITERAL(13, 155, 20), // "slotSendByQByteArray"
QT_MOC_LITERAL(14, 176, 6), // "arrMsg"
QT_MOC_LITERAL(15, 183, 16), // "slotDisconnected"
QT_MOC_LITERAL(16, 200, 16), // "slotDataReceived"
QT_MOC_LITERAL(17, 217, 16), // "slotInitForClild"
QT_MOC_LITERAL(18, 234, 20), // "slotCreateConnection"
QT_MOC_LITERAL(19, 255, 13), // "slotConnected"
QT_MOC_LITERAL(20, 269, 18), // "slotThreadFinished"
QT_MOC_LITERAL(21, 288, 19), // "slotSshConnectError"
QT_MOC_LITERAL(22, 308, 14), // "QSsh::SshError"
QT_MOC_LITERAL(23, 323, 8), // "sshError"
QT_MOC_LITERAL(24, 332, 14), // "slotShellStart"
QT_MOC_LITERAL(25, 347, 14) // "slotShellError"

    },
    "CConnectionForSshClient\0sigInitForClild\0"
    "\0sigConnectStateChanged\0bState\0strIp\0"
    "nPort\0sigDataArrived\0strMsg\0"
    "slotResetConnection\0strIpPort\0slotSend\0"
    "strMessage\0slotSendByQByteArray\0arrMsg\0"
    "slotDisconnected\0slotDataReceived\0"
    "slotInitForClild\0slotCreateConnection\0"
    "slotConnected\0slotThreadFinished\0"
    "slotSshConnectError\0QSsh::SshError\0"
    "sshError\0slotShellStart\0slotShellError"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CConnectionForSshClient[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      16,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   94,    2, 0x06 /* Public */,
       3,    3,   95,    2, 0x06 /* Public */,
       7,    3,  102,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       9,    1,  109,    2, 0x0a /* Public */,
      11,    2,  112,    2, 0x0a /* Public */,
      11,    1,  117,    2, 0x0a /* Public */,
      13,    2,  120,    2, 0x0a /* Public */,
      15,    0,  125,    2, 0x0a /* Public */,
      16,    0,  126,    2, 0x0a /* Public */,
      17,    0,  127,    2, 0x08 /* Private */,
      18,    0,  128,    2, 0x08 /* Private */,
      19,    0,  129,    2, 0x08 /* Private */,
      20,    0,  130,    2, 0x08 /* Private */,
      21,    1,  131,    2, 0x08 /* Private */,
      24,    0,  134,    2, 0x08 /* Private */,
      25,    0,  135,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool, QMetaType::QString, QMetaType::Int,    4,    5,    6,
    QMetaType::Void, QMetaType::QString, QMetaType::QString, QMetaType::Int,    8,    5,    6,

 // slots: parameters
    QMetaType::Void, QMetaType::QString,   10,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,   10,   12,
    QMetaType::Void, QMetaType::QString,    8,
    QMetaType::Void, QMetaType::QString, QMetaType::QByteArray,   10,   14,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 22,   23,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void CConnectionForSshClient::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<CConnectionForSshClient *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->sigInitForClild(); break;
        case 1: _t->sigConnectStateChanged((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3]))); break;
        case 2: _t->sigDataArrived((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3]))); break;
        case 3: _t->slotResetConnection((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 4: _t->slotSend((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2]))); break;
        case 5: _t->slotSend((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 6: _t->slotSendByQByteArray((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QByteArray(*)>(_a[2]))); break;
        case 7: _t->slotDisconnected(); break;
        case 8: _t->slotDataReceived(); break;
        case 9: _t->slotInitForClild(); break;
        case 10: _t->slotCreateConnection(); break;
        case 11: _t->slotConnected(); break;
        case 12: _t->slotThreadFinished(); break;
        case 13: _t->slotSshConnectError((*reinterpret_cast< QSsh::SshError(*)>(_a[1]))); break;
        case 14: _t->slotShellStart(); break;
        case 15: _t->slotShellError(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (CConnectionForSshClient::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CConnectionForSshClient::sigInitForClild)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (CConnectionForSshClient::*)(bool , QString , int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CConnectionForSshClient::sigConnectStateChanged)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (CConnectionForSshClient::*)(QString , QString , int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CConnectionForSshClient::sigDataArrived)) {
                *result = 2;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject CConnectionForSshClient::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_CConnectionForSshClient.data,
    qt_meta_data_CConnectionForSshClient,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *CConnectionForSshClient::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CConnectionForSshClient::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CConnectionForSshClient.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int CConnectionForSshClient::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 16)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 16;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 16)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 16;
    }
    return _id;
}

// SIGNAL 0
void CConnectionForSshClient::sigInitForClild()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void CConnectionForSshClient::sigConnectStateChanged(bool _t1, QString _t2, int _t3)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void CConnectionForSshClient::sigDataArrived(QString _t1, QString _t2, int _t3)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
