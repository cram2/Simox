#pragma once
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>

#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/nodes/SoSeparator.h>

#include "ui_CoinViewer.h"

class CoinViewerExample : public QMainWindow
{
    Q_OBJECT
public:
    CoinViewerExample();
    ~CoinViewerExample() override;
    int main();

public slots:
    void quit();
    void closeEvent(QCloseEvent* event) override;

protected:
    void timerEvent(QTimerEvent* event) override;

    Ui::MainWindowCamera UI;
    SoQtExaminerViewer* viewer;
    SoSeparator* sceneSep;
};

