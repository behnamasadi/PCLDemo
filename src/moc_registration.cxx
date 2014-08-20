/****************************************************************************
** Meta object code from reading C++ file 'registration.h'
**
** Created: Wed Nov 21 18:36:32 2012
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "registration.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'registration.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Registration[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      14,   13,   13,   13, 0x08,
      64,   13,   13,   13, 0x08,
      98,   13,   13,   13, 0x08,
     128,   13,   13,   13, 0x08,
     164,   13,   13,   13, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_Registration[] = {
    "Registration\0\0"
    "SelectFolderContainingPCDFilespushButtonClicked()\0"
    "VoxelGridSizeSlidervalueChanged()\0"
    "DownSamplepushButtonclicked()\0"
    "VoxelGridSizeSlidersliderReleased()\0"
    "CalculateCorrespondencespushButtonclicked()\0"
};

void Registration::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Registration *_t = static_cast<Registration *>(_o);
        switch (_id) {
        case 0: _t->SelectFolderContainingPCDFilespushButtonClicked(); break;
        case 1: _t->VoxelGridSizeSlidervalueChanged(); break;
        case 2: _t->DownSamplepushButtonclicked(); break;
        case 3: _t->VoxelGridSizeSlidersliderReleased(); break;
        case 4: _t->CalculateCorrespondencespushButtonclicked(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData Registration::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject Registration::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_Registration,
      qt_meta_data_Registration, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Registration::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Registration::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Registration::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Registration))
        return static_cast<void*>(const_cast< Registration*>(this));
    return QWidget::qt_metacast(_clname);
}

int Registration::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
