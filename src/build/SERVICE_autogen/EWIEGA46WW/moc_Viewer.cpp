/****************************************************************************
** Meta object code from reading C++ file 'Viewer.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../Viewer.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Viewer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_Viewer_t {
    QByteArrayData data[19];
    char stringdata0[212];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Viewer_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Viewer_t qt_meta_stringdata_Viewer = {
    {
QT_MOC_LITERAL(0, 0, 6), // "Viewer"
QT_MOC_LITERAL(1, 7, 12), // "disposeFrame"
QT_MOC_LITERAL(2, 20, 0), // ""
QT_MOC_LITERAL(3, 21, 13), // "displayFrames"
QT_MOC_LITERAL(4, 35, 20), // "const unsigned char*"
QT_MOC_LITERAL(5, 56, 5), // "right"
QT_MOC_LITERAL(6, 62, 10), // "rightWidth"
QT_MOC_LITERAL(7, 73, 11), // "rightHeight"
QT_MOC_LITERAL(8, 85, 14), // "QImage::Format"
QT_MOC_LITERAL(9, 100, 11), // "rightFormat"
QT_MOC_LITERAL(10, 112, 4), // "disp"
QT_MOC_LITERAL(11, 117, 9), // "dispWidth"
QT_MOC_LITERAL(12, 127, 10), // "dispHeight"
QT_MOC_LITERAL(13, 138, 10), // "dispFormat"
QT_MOC_LITERAL(14, 149, 9), // "rightRect"
QT_MOC_LITERAL(15, 159, 14), // "rightRectWidth"
QT_MOC_LITERAL(16, 174, 15), // "rightRectHeight"
QT_MOC_LITERAL(17, 190, 15), // "rightRectFormat"
QT_MOC_LITERAL(18, 206, 5) // "leave"

    },
    "Viewer\0disposeFrame\0\0displayFrames\0"
    "const unsigned char*\0right\0rightWidth\0"
    "rightHeight\0QImage::Format\0rightFormat\0"
    "disp\0dispWidth\0dispHeight\0dispFormat\0"
    "rightRect\0rightRectWidth\0rightRectHeight\0"
    "rightRectFormat\0leave"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Viewer[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   29,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       3,   12,   30,    2, 0x0a /* Public */,
      18,    0,   55,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 4, QMetaType::Int, QMetaType::Int, 0x80000000 | 8, 0x80000000 | 4, QMetaType::Int, QMetaType::Int, 0x80000000 | 8, 0x80000000 | 4, QMetaType::Int, QMetaType::Int, 0x80000000 | 8,    5,    6,    7,    9,   10,   11,   12,   13,   14,   15,   16,   17,
    QMetaType::Void,

       0        // eod
};

void Viewer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Viewer *_t = static_cast<Viewer *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->disposeFrame(); break;
        case 1: _t->displayFrames((*reinterpret_cast< const unsigned char*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< QImage::Format(*)>(_a[4])),(*reinterpret_cast< const unsigned char*(*)>(_a[5])),(*reinterpret_cast< int(*)>(_a[6])),(*reinterpret_cast< int(*)>(_a[7])),(*reinterpret_cast< QImage::Format(*)>(_a[8])),(*reinterpret_cast< const unsigned char*(*)>(_a[9])),(*reinterpret_cast< int(*)>(_a[10])),(*reinterpret_cast< int(*)>(_a[11])),(*reinterpret_cast< QImage::Format(*)>(_a[12]))); break;
        case 2: _t->leave(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            typedef void (Viewer::*_t)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&Viewer::disposeFrame)) {
                *result = 0;
                return;
            }
        }
    }
}

const QMetaObject Viewer::staticMetaObject = {
    { &QApplication::staticMetaObject, qt_meta_stringdata_Viewer.data,
      qt_meta_data_Viewer,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *Viewer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Viewer::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_Viewer.stringdata0))
        return static_cast<void*>(this);
    return QApplication::qt_metacast(_clname);
}

int Viewer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QApplication::qt_metacall(_c, _id, _a);
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
void Viewer::disposeFrame()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
