#ifndef MODEL_H
#define MODEL_H
#include <QObject>
#include <QGLWidget>
#include <string>
/*
 * 模型类，每一个对象对应一个模型块
 * 校对完毕；鑫2024.6.13
*/
typedef struct {
    std::string name;
    GLfloat ambient[4];
    GLfloat diffuse[4];
    GLfloat specular[4];
    GLfloat shininess;
} Material;
typedef struct {
    GLfloat x[3];
} Vertex;
typedef struct {
    int normal;
    int vertex[3];
    int material;
} Face;

class Model : public QObject
{
    Q_OBJECT
public:
    explicit Model(QObject *parent = 0);

signals:

public slots:
    void load(const char *filename);//从obj文件读取文件，注意源文件的模型长度单位和坐标系！！
    void display();//绘制图像，需提前定好坐标原点

private:
    std::string matfile;
    std::vector<Material> materials;
    std::vector<Vertex> vertex;
    std::vector<Vertex> normals;
    std::vector<Face> faces;

    void load_materials();//load的子函数，加载材料信息
};

#endif // MODEL_H
