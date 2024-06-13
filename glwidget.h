#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include "scara.h"

/*
 * 自定义的gl类，显示仿真用
 * 校对完毕；鑫2024.6.13
*/
class GLWidget : public QGLWidget
{
    Q_OBJECT
public:
    explicit GLWidget(QWidget *parent = 0);

signals:

protected:
    //覆写该类的固有函数
    void initializeGL();//参数初始化，不管
    void paintGL();//绘图，
    void resizeGL(int width, int height);//更改界面大小

public slots:
    void setPitch(double pitch);
    void setYaw(double yaw);
    void setDistance(double distance);//设置摄像机参数

private:
    void setView();//根据摄像机参数设置视角
    void setLight();//设置光线，固定

    int w, h;
    double pitch, yaw, distance;//摄像机参数，有默认值,前两者可滑动修改
};

#endif // GLWIDGET_H
