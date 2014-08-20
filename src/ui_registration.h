/********************************************************************************
** Form generated from reading UI file 'registration.ui'
**
** Created: Wed Nov 21 18:36:31 2012
**      by: Qt User Interface Compiler version 4.8.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_REGISTRATION_H
#define UI_REGISTRATION_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QSpinBox>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_RegistrationClass
{
public:
    QLabel *label;
    QSlider *VoxelGridSize;
    QComboBox *comboBox;
    QLabel *label_3;
    QPushButton *SelectFolderContainingPCDFilespushButton;
    QLabel *VoxelGridSizelabel;
    QPushButton *DownSamplepushButton;
    QGroupBox *groupBox;
    QLabel *label_9;
    QLabel *label_8;
    QDoubleSpinBox *doubleSpinBox;
    QSpinBox *spinBox;
    QSpinBox *spinBox_2;
    QLabel *label_10;
    QDoubleSpinBox *doubleSpinBox_2;
    QLabel *label_12;
    QGroupBox *groupBox_2;
    QSpinBox *spinBox_3;
    QDoubleSpinBox *doubleSpinBox_5;
    QLabel *label_5;
    QLabel *label_4;
    QDoubleSpinBox *doubleSpinBox_4;
    QLabel *label_7;
    QGroupBox *groupBox_3;
    QComboBox *comboBox_2;
    QLabel *label_11;
    QDoubleSpinBox *doubleSpinBox_3;
    QLabel *label_2;
    QPushButton *CalculateCorrespondencespushButton;
    QGroupBox *groupBox_4;

    void setupUi(QWidget *RegistrationClass)
    {
        if (RegistrationClass->objectName().isEmpty())
            RegistrationClass->setObjectName(QString::fromUtf8("RegistrationClass"));
        RegistrationClass->resize(1038, 637);
        label = new QLabel(RegistrationClass);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(20, 50, 221, 17));
        VoxelGridSize = new QSlider(RegistrationClass);
        VoxelGridSize->setObjectName(QString::fromUtf8("VoxelGridSize"));
        VoxelGridSize->setGeometry(QRect(250, 40, 301, 29));
        VoxelGridSize->setOrientation(Qt::Horizontal);
        comboBox = new QComboBox(RegistrationClass);
        comboBox->setObjectName(QString::fromUtf8("comboBox"));
        comboBox->setGeometry(QRect(260, 80, 211, 27));
        label_3 = new QLabel(RegistrationClass);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(20, 80, 181, 20));
        SelectFolderContainingPCDFilespushButton = new QPushButton(RegistrationClass);
        SelectFolderContainingPCDFilespushButton->setObjectName(QString::fromUtf8("SelectFolderContainingPCDFilespushButton"));
        SelectFolderContainingPCDFilespushButton->setGeometry(QRect(10, 10, 241, 27));
        VoxelGridSizelabel = new QLabel(RegistrationClass);
        VoxelGridSizelabel->setObjectName(QString::fromUtf8("VoxelGridSizelabel"));
        VoxelGridSizelabel->setGeometry(QRect(560, 47, 101, 16));
        DownSamplepushButton = new QPushButton(RegistrationClass);
        DownSamplepushButton->setObjectName(QString::fromUtf8("DownSamplepushButton"));
        DownSamplepushButton->setGeometry(QRect(680, 40, 111, 27));
        groupBox = new QGroupBox(RegistrationClass);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(20, 130, 341, 161));
        label_9 = new QLabel(groupBox);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setGeometry(QRect(20, 80, 221, 17));
        label_8 = new QLabel(groupBox);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(20, 50, 181, 17));
        doubleSpinBox = new QDoubleSpinBox(groupBox);
        doubleSpinBox->setObjectName(QString::fromUtf8("doubleSpinBox"));
        doubleSpinBox->setGeometry(QRect(260, 40, 62, 27));
        doubleSpinBox->setValue(0.3);
        spinBox = new QSpinBox(groupBox);
        spinBox->setObjectName(QString::fromUtf8("spinBox"));
        spinBox->setGeometry(QRect(260, 100, 59, 27));
        spinBox->setValue(5);
        spinBox_2 = new QSpinBox(groupBox);
        spinBox_2->setObjectName(QString::fromUtf8("spinBox_2"));
        spinBox_2->setGeometry(QRect(260, 130, 59, 27));
        spinBox_2->setValue(3);
        label_10 = new QLabel(groupBox);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setGeometry(QRect(20, 110, 151, 17));
        doubleSpinBox_2 = new QDoubleSpinBox(groupBox);
        doubleSpinBox_2->setObjectName(QString::fromUtf8("doubleSpinBox_2"));
        doubleSpinBox_2->setGeometry(QRect(260, 70, 62, 27));
        doubleSpinBox_2->setDecimals(3);
        doubleSpinBox_2->setValue(0.001);
        label_12 = new QLabel(groupBox);
        label_12->setObjectName(QString::fromUtf8("label_12"));
        label_12->setGeometry(QRect(20, 140, 221, 17));
        groupBox_2 = new QGroupBox(RegistrationClass);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setGeometry(QRect(20, 450, 381, 161));
        spinBox_3 = new QSpinBox(groupBox_2);
        spinBox_3->setObjectName(QString::fromUtf8("spinBox_3"));
        spinBox_3->setGeometry(QRect(260, 100, 61, 27));
        spinBox_3->setMaximum(10000);
        spinBox_3->setValue(5000);
        doubleSpinBox_5 = new QDoubleSpinBox(groupBox_2);
        doubleSpinBox_5->setObjectName(QString::fromUtf8("doubleSpinBox_5"));
        doubleSpinBox_5->setGeometry(QRect(260, 60, 62, 27));
        doubleSpinBox_5->setValue(0.03);
        label_5 = new QLabel(groupBox_2);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(20, 70, 141, 17));
        label_4 = new QLabel(groupBox_2);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(20, 30, 121, 17));
        doubleSpinBox_4 = new QDoubleSpinBox(groupBox_2);
        doubleSpinBox_4->setObjectName(QString::fromUtf8("doubleSpinBox_4"));
        doubleSpinBox_4->setGeometry(QRect(260, 20, 62, 27));
        doubleSpinBox_4->setValue(1);
        label_7 = new QLabel(groupBox_2);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(20, 110, 201, 17));
        groupBox_3 = new QGroupBox(RegistrationClass);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        groupBox_3->setGeometry(QRect(20, 320, 351, 91));
        comboBox_2 = new QComboBox(groupBox_3);
        comboBox_2->setObjectName(QString::fromUtf8("comboBox_2"));
        comboBox_2->setGeometry(QRect(220, 30, 101, 27));
        label_11 = new QLabel(groupBox_3);
        label_11->setObjectName(QString::fromUtf8("label_11"));
        label_11->setGeometry(QRect(20, 60, 191, 17));
        doubleSpinBox_3 = new QDoubleSpinBox(groupBox_3);
        doubleSpinBox_3->setObjectName(QString::fromUtf8("doubleSpinBox_3"));
        doubleSpinBox_3->setGeometry(QRect(260, 60, 62, 27));
        doubleSpinBox_3->setValue(0.08);
        label_2 = new QLabel(groupBox_3);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(20, 30, 181, 20));
        CalculateCorrespondencespushButton = new QPushButton(RegistrationClass);
        CalculateCorrespondencespushButton->setObjectName(QString::fromUtf8("CalculateCorrespondencespushButton"));
        CalculateCorrespondencespushButton->setGeometry(QRect(500, 590, 221, 27));
        groupBox_4 = new QGroupBox(RegistrationClass);
        groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
        groupBox_4->setGeometry(QRect(390, 140, 611, 441));

        retranslateUi(RegistrationClass);

        QMetaObject::connectSlotsByName(RegistrationClass);
    } // setupUi

    void retranslateUi(QWidget *RegistrationClass)
    {
        RegistrationClass->setWindowTitle(QApplication::translate("RegistrationClass", "Registration", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("RegistrationClass", "DownSampling(Voxel Grid Size):", 0, QApplication::UnicodeUTF8));
        comboBox->clear();
        comboBox->insertItems(0, QStringList()
         << QApplication::translate("RegistrationClass", "SIFT", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("RegistrationClass", "NARF", 0, QApplication::UnicodeUTF8)
        );
        label_3->setText(QApplication::translate("RegistrationClass", "Select Feature Method", 0, QApplication::UnicodeUTF8));
        SelectFolderContainingPCDFilespushButton->setText(QApplication::translate("RegistrationClass", "Select Folder Containing PCD Files", 0, QApplication::UnicodeUTF8));
        VoxelGridSizelabel->setText(QApplication::translate("RegistrationClass", "voxel grid size", 0, QApplication::UnicodeUTF8));
        DownSamplepushButton->setText(QApplication::translate("RegistrationClass", "DownSample", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("RegistrationClass", "SIFT Parameters", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("RegistrationClass", "Minimum  Scale Size", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("RegistrationClass", "Surface Normals Radius", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("RegistrationClass", "Number Of Octaves", 0, QApplication::UnicodeUTF8));
        label_12->setText(QApplication::translate("RegistrationClass", "Number Of Octave In Each Scale", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("RegistrationClass", "Correspondence Rejector Parameter", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("RegistrationClass", "RANSAC Threshold", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("RegistrationClass", "Median Factor", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("RegistrationClass", "RANSAC Max Iterations Run", 0, QApplication::UnicodeUTF8));
        groupBox_3->setTitle(QApplication::translate("RegistrationClass", "KeyPoint Parameters", 0, QApplication::UnicodeUTF8));
        comboBox_2->clear();
        comboBox_2->insertItems(0, QStringList()
         << QApplication::translate("RegistrationClass", "PFH", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("RegistrationClass", "FPFH", 0, QApplication::UnicodeUTF8)
        );
        label_11->setText(QApplication::translate("RegistrationClass", "Feature Radius", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("RegistrationClass", "Select KeyPoints Method", 0, QApplication::UnicodeUTF8));
        CalculateCorrespondencespushButton->setText(QApplication::translate("RegistrationClass", "Calculates Correspondences", 0, QApplication::UnicodeUTF8));
        groupBox_4->setTitle(QApplication::translate("RegistrationClass", "Result", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class RegistrationClass: public Ui_RegistrationClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_REGISTRATION_H
