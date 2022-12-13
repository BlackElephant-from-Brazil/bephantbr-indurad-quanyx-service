/****************************************************************************
** Meta object code from reading C++ file 'SensorControlThread.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../SensorControlThread.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SensorControlThread.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SensorControlThread_t {
    QByteArrayData data[27];
    char stringdata0[315];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SensorControlThread_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SensorControlThread_t qt_meta_stringdata_SensorControlThread = {
    {
QT_MOC_LITERAL(0, 0, 19), // "SensorControlThread"
QT_MOC_LITERAL(1, 20, 9), // "newFrames"
QT_MOC_LITERAL(2, 30, 0), // ""
QT_MOC_LITERAL(3, 31, 20), // "const unsigned char*"
QT_MOC_LITERAL(4, 52, 5), // "right"
QT_MOC_LITERAL(5, 58, 10), // "rightWidth"
QT_MOC_LITERAL(6, 69, 11), // "rightHeight"
QT_MOC_LITERAL(7, 81, 14), // "QImage::Format"
QT_MOC_LITERAL(8, 96, 11), // "rightFormat"
QT_MOC_LITERAL(9, 108, 4), // "left"
QT_MOC_LITERAL(10, 113, 9), // "leftWidth"
QT_MOC_LITERAL(11, 123, 10), // "leftHeight"
QT_MOC_LITERAL(12, 134, 10), // "leftFormat"
QT_MOC_LITERAL(13, 145, 4), // "disp"
QT_MOC_LITERAL(14, 150, 9), // "dispWidth"
QT_MOC_LITERAL(15, 160, 10), // "dispHeight"
QT_MOC_LITERAL(16, 171, 10), // "dispFormat"
QT_MOC_LITERAL(17, 182, 9), // "rightRect"
QT_MOC_LITERAL(18, 192, 14), // "rightRectWidth"
QT_MOC_LITERAL(19, 207, 15), // "rightRectHeight"
QT_MOC_LITERAL(20, 223, 15), // "rightRectFormat"
QT_MOC_LITERAL(21, 239, 8), // "leftRect"
QT_MOC_LITERAL(22, 248, 13), // "leftRectWidth"
QT_MOC_LITERAL(23, 262, 14), // "leftRectHeight"
QT_MOC_LITERAL(24, 277, 14), // "leftRectFormat"
QT_MOC_LITERAL(25, 292, 11), // "noMoreFrame"
QT_MOC_LITERAL(26, 304, 10) // "unrefFrame"

    },
    "SensorControlThread\0newFrames\0\0"
    "const unsigned char*\0right\0rightWidth\0"
    "rightHeight\0QImage::Format\0rightFormat\0"
    "left\0leftWidth\0leftHeight\0leftFormat\0"
    "disp\0dispWidth\0dispHeight\0dispFormat\0"
    "rightRect\0rightRectWidth\0rightRectHeight\0"
    "rightRectFormat\0leftRect\0leftRectWidth\0"
    "leftRectHeight\0leftRectFormat\0noMoreFrame\0"
    "unrefFrame"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SensorControlThread[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,   20,   29,    2, 0x06 /* Public */,
      25,    0,   70,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      26,    0,   71,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, QMetaType::Int, QMetaType::Int, 0x80000000 | 7, 0x80000000 | 3, QMetaType::Int, QMetaType::Int, 0x80000000 | 7, 0x80000000 | 3, QMetaType::Int, QMetaType::Int, 0x80000000 | 7, 0x80000000 | 3, QMetaType::Int, QMetaType::Int, 0x80000000 | 7, 0x80000000 | 3, QMetaType::Int, QMetaType::Int, 0x80000000 | 7,    4,    5,    6,    8,    9,   10,   11,   12,   13,   14,   15,   16,   17,   18,   19,   20,   21,   22,   23,   24,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,

       0        // eod
};

void SensorControlThread::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        SensorControlThread *_t = static_cast<SensorControlThread *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->newFrames((*reinterpret_cast< const unsigned char*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< QImage::Format(*)>(_a[4])),(*reinterpret_cast< const unsigned char*(*)>(_a[5])),(*reinterpret_cast< int(*)>(_a[6])),(*reinterpret_cast< int(*)>(_a[7])),(*reinterpret_cast< QImage::Format(*)>(_a[8])),(*reinterpret_cast< const unsigned char*(*)>(_a[9])),(*reinterpret_cast< int(*)>(_a[10])),(*reinterpret_cast< int(*)>(_a[11])),(*reinterpret_cast< QImage::Format(*)>(_a[12])),(*reinterpret_cast< const unsigned char*(*)>(_a[13])),(*reinterpret_cast< int(*)>(_a[14])),(*reinterpret_cast< int(*)>(_a[15])),(*reinterpret_cast< QImage::Format(*)>(_a[16])),(*reinterpret_cast< const unsigned char*(*)>(_a[17])),(*reinterpret_cast< int(*)>(_a[18])),(*reinterpret_cast< int(*)>(_a[19])),(*reinterpret_cast< QImage::Format(*)>(_a[20]))); break;
        case 1: _t->noMoreFrame(); break;
        case 2: _t->unrefFrame(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            typedef void (SensorControlThread::*_t)(const unsigned char * , int , int , QImage::Format , const unsigned char * , int , int , QImage::Format , const unsigned char * , int , int , QImage::Format , const unsigned char * , int , int , QImage::Format , const unsigned char * , int , int , QImage::Format );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SensorControlThread::newFrames)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (SensorControlThread::*_t)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&SensorControlThread::noMoreFrame)) {
                *result = 1;
                return;
            }
        }
    }
}

const QMetaObject SensorControlThread::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_SensorControlThread.data,
      qt_meta_data_SensorControlThread,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *SensorControlThread::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SensorControlThread::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SensorControlThread.stringdata0))
        return static_cast<void*>(this);
    return QThread::qt_metacast(_clname);
}

int SensorControlThread::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void SensorControlThread::newFrames(const unsigned char * _t1, int _t2, int _t3, QImage::Format _t4, const unsigned char * _t5, int _t6, int _t7, QImage::Format _t8, const unsigned char * _t9, int _t10, int _t11, QImage::Format _t12, const unsigned char * _t13, int _t14, int _t15, QImage::Format _t16, const unsigned char * _t17, int _t18, int _t19, QImage::Format _t20)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)), const_cast<void*>(reinterpret_cast<const void*>(&_t5)), const_cast<void*>(reinterpret_cast<const void*>(&_t6)), const_cast<void*>(reinterpret_cast<const void*>(&_t7)), const_cast<void*>(reinterpret_cast<const void*>(&_t8)), const_cast<void*>(reinterpret_cast<const void*>(&_t9)), const_cast<void*>(reinterpret_cast<const void*>(&_t10)), const_cast<void*>(reinterpret_cast<const void*>(&_t11)), const_cast<void*>(reinterpret_cast<const void*>(&_t12)), const_cast<void*>(reinterpret_cast<const void*>(&_t13)), const_cast<void*>(reinterpret_cast<const void*>(&_t14)), const_cast<void*>(reinterpret_cast<const void*>(&_t15)), const_cast<void*>(reinterpret_cast<const void*>(&_t16)), const_cast<void*>(reinterpret_cast<const void*>(&_t17)), const_cast<void*>(reinterpret_cast<const void*>(&_t18)), const_cast<void*>(reinterpret_cast<const void*>(&_t19)), const_cast<void*>(reinterpret_cast<const void*>(&_t20)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void SensorControlThread::noMoreFrame()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
