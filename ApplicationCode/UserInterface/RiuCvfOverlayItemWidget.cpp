/////////////////////////////////////////////////////////////////////////////////
//
//  Copyright (C) 2018-     Statoil ASA
// 
//  ResInsight is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
// 
//  ResInsight is distributed in the hope that it will be useful, but WITHOUT ANY
//  WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.
// 
//  See the GNU General Public License at <http://www.gnu.org/licenses/gpl.html> 
//  for more details.
//
/////////////////////////////////////////////////////////////////////////////////


#include "RiuCvfOverlayItemWidget.h"

#include <QApplication>
#include <QFrame>
#include <QLabel>
#include <QPixmap>
#include <QPushButton>
#include <QResizeEvent>
#include <QBoxLayout>

//--------------------------------------------------------------------------------------------------
/// 
//--------------------------------------------------------------------------------------------------
RiuCvfOverlayItemWidget::RiuCvfOverlayItemWidget(QWidget* parent/*=0*/)
: QWidget(parent)
{
    this->setLayout(new QHBoxLayout(this));
    m_overlayItemLabel = new QLabel(this);
    this->layout()->addWidget(m_overlayItemLabel);
}

//--------------------------------------------------------------------------------------------------
/// 
//--------------------------------------------------------------------------------------------------
RiuCvfOverlayItemWidget::~RiuCvfOverlayItemWidget()
{

}

#include "cafViewer.h"
#include "cvfRendering.h"
#include "cvfRenderSequence.h"
#include "cvfFramebufferObject.h"
#include "cvfRenderbufferObject.h"
#include "cvfqtUtils.h"
#include "glew/GL/glew.h"
#include "RiaApplication.h"
#include "cvfCamera.h"

//--------------------------------------------------------------------------------------------------
/// 
//--------------------------------------------------------------------------------------------------
void RiuCvfOverlayItemWidget::updateFromOverlyItem( cvf::OverlayItem * item)
{
    //m_scalarMapperLegend->setTitle("Hei og hopp");
    //m_scalarMapperLegend->computeLayoutAndExtents({0,0}, {100, 400});
    //unsigned int width = m_scalarMapperLegend->minimumWidth() + 100;
    unsigned int width = item->sizeHint().x();
    unsigned int height =  item->sizeHint().y();

    QGLFormat glFormat;
    glFormat.setDirectRendering(RiaApplication::instance()->useShaders());

    caf::Viewer*  viewer = new caf::Viewer(glFormat, nullptr);
    cvf::OpenGLContext* cvfOglContext = viewer->cvfOpenGLContext();
    viewer->resize(width, height);

    // Create a rendering

    cvf::ref<cvf::Rendering> rendering = new cvf::Rendering;
    item->setLayoutFixedPosition({0,0});
    rendering->addOverlayItem(item);


    rendering->camera()->setViewport(0,0,width, height);
    rendering->camera()->viewport()->setClearColor({1,1,1,0});

    cvf::ref<cvf::RenderSequence> renderingSequence = new cvf::RenderSequence;
    renderingSequence->addRendering(rendering.p());

    if (RiaApplication::instance()->useShaders())
    {
        // Set up frame and render buffers    

        cvf::ref<cvf::FramebufferObject> fbo = new cvf::FramebufferObject;

        cvf::ref<cvf::RenderbufferObject> rboColor = new cvf::RenderbufferObject(cvf::RenderbufferObject::RGBA, width, height);
        cvf::ref<cvf::RenderbufferObject> rboDepth = new cvf::RenderbufferObject(cvf::RenderbufferObject::DEPTH_COMPONENT24, width, height);

        fbo->attachDepthRenderbuffer(rboDepth.p());
        fbo->attachColorRenderbuffer(0, rboColor.p());

        fbo->applyOpenGL(cvfOglContext);
        rendering->setTargetFramebuffer(fbo.p());
        fbo->bind(cvfOglContext);
    }

    renderingSequence->render(cvfOglContext);

    // Read data from framebuffer

    cvf::UByteArray arr(4*width*height);
    glReadPixels(0, 0, static_cast<GLsizei>(width), static_cast<GLsizei>(height), GL_RGBA, GL_UNSIGNED_BYTE, arr.ptr());

    // Create a cvf texture image

    cvf::ref<cvf::TextureImage> img = new cvf::TextureImage;
    img->setData(arr.ptr(), width, height);

    QImage image = cvfqt::Utils::toQImage(*img.p());
    //image.save("jjsLegendImageTest.png");

    QPixmap pixmap = QPixmap::fromImage(image);

    delete viewer;

    m_overlayItemLabel->setPixmap(pixmap);
    this->setMinimumSize(QSize(width, height));

}

#if 0

#include "cafViewer.h"
#include "cvfRendering.h"
#include "cvfRenderSequence.h"
#include "cvfFramebufferObject.h"
#include "cvfRenderbufferObject.h"
#include "cvfqtUtils.h"
#include "glew/GL/glew.h"

//--------------------------------------------------------------------------------------------------
/// 
//--------------------------------------------------------------------------------------------------
QPixmap RimLegendConfig::drawLegend()
{   
    m_scalarMapperLegend->setTitle("Hei og hopp");
    m_scalarMapperLegend->computeLayoutAndExtents({0,0}, {100, 400});
    unsigned int width = m_scalarMapperLegend->minimumWidth() + 100;
    unsigned int height =  m_scalarMapperLegend->sizeHint().y();

    QGLFormat glFormat;
    glFormat.setDirectRendering(RiaApplication::instance()->useShaders());

    caf::Viewer*  viewer = new caf::Viewer(glFormat, nullptr);
    cvf::OpenGLContext* cvfOglContext = viewer->cvfOpenGLContext();
    viewer->resize(width, height);


    // Create a rendering

    cvf::ref<cvf::Rendering> rendering = new cvf::Rendering;
    m_scalarMapperLegend->setLayoutFixedPosition({0,0});
    rendering->addOverlayItem(m_scalarMapperLegend.p());


    rendering->camera()->setViewport(0,0,width, height);
    rendering->camera()->viewport()->setClearColor({1,1,1,0});

    cvf::ref<cvf::RenderSequence> renderingSequence = new cvf::RenderSequence;
    renderingSequence->addRendering(rendering.p());

    if (RiaApplication::instance()->useShaders())
    {
        // Set up frame and render buffers    

        cvf::ref<cvf::FramebufferObject> fbo = new cvf::FramebufferObject;

        cvf::ref<cvf::RenderbufferObject> rboColor = new cvf::RenderbufferObject(cvf::RenderbufferObject::RGBA, width, height);
        cvf::ref<cvf::RenderbufferObject> rboDepth = new cvf::RenderbufferObject(cvf::RenderbufferObject::DEPTH_COMPONENT24, width, height);

        fbo->attachDepthRenderbuffer(rboDepth.p());
        fbo->attachColorRenderbuffer(0, rboColor.p());

        fbo->applyOpenGL(cvfOglContext);
        rendering->setTargetFramebuffer(fbo.p());
        fbo->bind(cvfOglContext);
    }

    renderingSequence->render(cvfOglContext);

    // Read data from framebuffer

    cvf::UByteArray arr(4*width*height);
    glReadPixels(0, 0, static_cast<GLsizei>(width), static_cast<GLsizei>(height), GL_RGBA, GL_UNSIGNED_BYTE, arr.ptr());

    // Create a cvf texture image

    cvf::ref<cvf::TextureImage> img = new cvf::TextureImage;
    img->setData(arr.ptr(), width, height);

    QImage image = cvfqt::Utils::toQImage(*img.p());
    //image.save("jjsLegendImageTest.png");

    QPixmap pixmap = QPixmap::fromImage(image);

    delete viewer;

    return pixmap;
}
#endif