/****************************************************************************
** Meta object code from reading C++ file 'ejemplo1.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../ejemplo1.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ejemplo1.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_ejemplo1_t {
    QByteArrayData data[7];
    char stringdata0[49];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ejemplo1_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ejemplo1_t qt_meta_stringdata_ejemplo1 = {
    {
QT_MOC_LITERAL(0, 0, 8), // "ejemplo1"
QT_MOC_LITERAL(1, 9, 6), // "itsTen"
QT_MOC_LITERAL(2, 16, 0), // ""
QT_MOC_LITERAL(3, 17, 4), // "cont"
QT_MOC_LITERAL(4, 22, 8), // "doButton"
QT_MOC_LITERAL(5, 31, 6), // "incLCD"
QT_MOC_LITERAL(6, 38, 10) // "printHello"

    },
    "ejemplo1\0itsTen\0\0cont\0doButton\0incLCD\0"
    "printHello"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ejemplo1[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       4,    0,   37,    2, 0x0a /* Public */,
       5,    0,   38,    2, 0x0a /* Public */,
       6,    1,   39,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Int,    3,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    3,

       0        // eod
};

void ejemplo1::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<ejemplo1 *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->itsTen((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->doButton(); break;
        case 2: _t->incLCD(); break;
        case 3: _t->printHello((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (ejemplo1::*)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&ejemplo1::itsTen)) {
                *result = 0;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject ejemplo1::staticMetaObject = { {
    &QWidget::staticMetaObject,
    qt_meta_stringdata_ejemplo1.data,
    qt_meta_data_ejemplo1,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *ejemplo1::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ejemplo1::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ejemplo1.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "Ui_Counter"))
        return static_cast< Ui_Counter*>(this);
    return QWidget::qt_metacast(_clname);
}

int ejemplo1::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void ejemplo1::itsTen(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
