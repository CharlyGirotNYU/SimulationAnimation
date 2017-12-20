#include "myWindow.hpp"

#include "myWidgetGL.hpp"
#include "../../lib/common/error_handling.hpp"
#include "ui_mainwindow.h"

#include <iostream>

myWindow::myWindow(QWidget *parent)
    :QMainWindow(parent),ui(new Ui::MainWindow)
{
    try
    {
        //Setup window layout
        ui->setupUi(this);

        //Create openGL context
        QGLFormat qglFormat;
        qglFormat.setVersion(1,2);

        //Create OpenGL Widget renderer
        glWidget=new myWidgetGL(qglFormat);

        //Add the OpenGL Widget into the layout
        ui->layout_scene->addWidget(glWidget);
    }
    catch(cpe::exception_cpe const& e)
    {
        std::cout<<std::endl<<e.report_exception()<<std::endl;
    }

    //Connect slot and signals
    connect(ui->quit,SIGNAL(clicked()),this,SLOT(action_quit()));
    connect(ui->draw,SIGNAL(clicked()),this,SLOT(action_draw()));
    connect(ui->wireframe,SIGNAL(clicked()),this,SLOT(action_wireframe()));
    connect(ui->K_structural,SIGNAL(valueChanged(int)),this,SLOT(action_K(int)));
    connect(ui->K_shearing,SIGNAL(valueChanged(int)),this,SLOT(action_K(int)));
    connect(ui->K_bending,SIGNAL(valueChanged(int)),this,SLOT(action_K(int)));
    connect(ui->wind,SIGNAL(valueChanged(int)),this,SLOT(action_wind(int)));
    connect(ui->dt,SIGNAL(valueChanged(int)),this,SLOT(action_dt(int)));
    connect(ui->attache,SIGNAL(valueChanged(int)),this,SLOT(action_attache(int)));

}

myWindow::~myWindow()
{}

void myWindow::action_quit()
{
    close();
}

void myWindow::action_draw()
{
    glWidget->change_draw_state();
}

void myWindow::action_wireframe()
{
    bool const state_wireframe=ui->wireframe->isChecked();
    glWidget->wireframe(state_wireframe);
}

void myWindow::action_wind(int v)
{
    glWidget->change_wind(v);
}

void myWindow::action_K(int v)
{
    glWidget->change_K_structural(ui->K_structural->value());
    glWidget->change_K_shearing(ui->K_shearing->value());
    glWidget->change_K_bending(ui->K_bending->value());
}

void myWindow::action_dt(int v)
{
    glWidget->change_dt(v);
}

void myWindow::action_attache(int v)
{
    glWidget->change_attache(v);
}
