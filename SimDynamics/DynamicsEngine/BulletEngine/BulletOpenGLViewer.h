/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    SimDynamics
* @author     Nikolaus Vahrenkamp
* @copyright  2012 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/

#pragma once

#include "../../DynamicsWorld.h"
#include "../../SimDynamics.h"
#include "BulletEngine.h"

#ifdef _WINDOWS
#include "BulletOpenGL/Win32DemoApplication.h"
#define PlatformDemoApplication Win32DemoApplication
#else
#include "BulletOpenGL/GlutDemoApplication.h"
#define PlatformDemoApplication GlutDemoApplication
#endif

#include "BulletOpenGL/GLDebugDrawer.h"
#include "BulletOpenGL/GL_ShapeDrawer.h"
#include "BulletOpenGL/GlutStuff.h"
#include <LinearMath/btIDebugDraw.h>
#include <btBulletDynamicsCommon.h>

#ifdef _WIN32
#pragma warning(disable : 4275)
#endif

namespace SimDynamics
{

    class SIMDYNAMICS_IMPORT_EXPORT BulletOpenGLViewer : public PlatformDemoApplication
    {
    public:
        BulletOpenGLViewer(DynamicsWorldPtr world);
        ~BulletOpenGLViewer() override;

        void clientMoveAndDisplay() override;
        void displayCallback() override;
        void keyboardCallback(unsigned char key, int x, int y) override;
        void initPhysics() override;
        void myinit() override;

        virtual void enableContraintsDebugDrawing();

    protected:
        /*static DemoApplication* Create()
        {
            BulletOpenGLViewer* demo = new BulletOpenGLViewer;
            demo->myinit();
            demo->initPhysics();
            return demo;
        }*/

        DynamicsWorldPtr world;

        void updateRobotConstraints();
        GLDebugDrawer debugDrawer;

        BulletEnginePtr bulletEngine;
    };

    typedef std::shared_ptr<BulletOpenGLViewer> BulletOpenGLViewerPtr;

} // namespace SimDynamics
