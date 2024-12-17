#pragma once

#include <vector>

#include <QtCore/QtGlobal>
#include <QtGui/QtGui>

#include "ui_RGBOffscreenRendering.h"
#include <Inventor/SoOffscreenRenderer.h>
#include <Inventor/nodes/SoSeparator.h>

class RGBOffscreenRenderingExample : public QMainWindow
{
    Q_OBJECT
public:
    RGBOffscreenRenderingExample();
    ~RGBOffscreenRenderingExample() override;
    int main();

public slots:
    void quit();
    void closeEvent(QCloseEvent* event) override;
    void camYawUpdated(double y);

protected:
    Ui::MainWindowCamera UI;
    SoSeparator* sceneSep;
    SoOffscreenRenderer* camRenderer;
    std::vector<unsigned char> camRGBBuffer;
};
