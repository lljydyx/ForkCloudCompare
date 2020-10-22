//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: ExamplePlugin                      #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                             COPYRIGHT: XXX                             #
//#                                                                        #
//##########################################################################

// First:
//	Replace all occurrences of 'ExamplePlugin' by your own plugin class name in this file.
//	This includes the resource path to info.json in the constructor.

// Second:
//	Open ExamplePlugin.qrc, change the "prefix" and the icon filename for your plugin.
//	Change the name of the file to <yourPluginName>.qrc

// Third:
//	Open the info.json file and fill in the information about the plugin.
//	 "type" should be one of: "Standard", "GL", or "I/O" (required)
//	 "name" is the name of the plugin (required)
//	 "icon" is the Qt resource path to the plugin's icon (from the .qrc file)
//	 "description" is used as a tootip if the plugin has actions and is displayed in the plugin dialog
//	 "authors", "maintainers", and "references" show up in the plugin dialog as well

#include <QtGui>

#include "ExamplePlugin.h"

////PDAL
//#include <vector>
//#include <memory>
//#include <pdal/Options.hpp>
//#include <pdal/PointTable.hpp>
//#include <pdal/PointView.hpp>
//#include <pdal/StageFactory.hpp>
//#include <pdal/SpatialReference.hpp>
//#include <pdal/io/LasHeader.hpp>
//#include <pdal/io/LasReader.hpp>
//#include <pdal/io/BufferReader.hpp>
//#include <pdal/io/LasWriter.hpp>

// Default constructor:
//	- pass the Qt resource path to the info.json file (from <yourPluginName>.qrc file) 
//  - constructor should mainly be used to initialize actions and other members
ExamplePlugin::ExamplePlugin( QObject *parent )
	: QObject( parent )
	, ccStdPluginInterface( ":/CC/plugin/ExamplePlugin/info.json" )
	, m_action( nullptr )
{
}

// This method should enable or disable your plugin actions
// depending on the currently selected entities ('selectedEntities').
void ExamplePlugin::onNewSelection( const ccHObject::Container &selectedEntities )
{
	if ( m_action == nullptr )
	{
		return;
	}
	
	// If you need to check for a specific type of object, you can use the methods
	// in ccHObjectCaster.h or loop and check the objects' classIDs like this:
	//
	//	for ( ccHObject *object : selectedEntities )
	//	{
	//		if ( object->getClassID() == CC_TYPES::VIEWPORT_2D_OBJECT )
	//		{
	//			// ... do something with the viewports
	//		}
	//	}
	
	// For example - only enable our action if something is selected.
	m_action->setEnabled( !selectedEntities.empty() );
}

// This method returns all the 'actions' your plugin can perform.
// getActions() will be called only once, when plugin is loaded.
QList<QAction *> ExamplePlugin::getActions()
{
	// default action (if it has not been already created, this is the moment to do it)
	if ( !m_action )
	{
		// Here we use the default plugin name, description, and icon,
		// but each action should have its own.
		m_action = new QAction( getName(), this );
		m_action->setToolTip( getDescription() );
		m_action->setIcon( getIcon() );
		
		// Connect appropriate signal
		connect( m_action, &QAction::triggered, this, &ExamplePlugin::doAction );
	}

	return { m_action };
}

// This is an example of an action's method called when the corresponding action
// is triggered (i.e. the corresponding icon or menu entry is clicked in CC's
// main interface). You can access most of CC's components (database,
// 3D views, console, etc.) via the 'm_app' variable (see the ccMainAppInterface
// class in ccMainAppInterface.h).
void ExamplePlugin::doAction()
{
	if (m_app == nullptr)
	{
		// m_app should have already been initialized by CC when plugin is loaded
		Q_ASSERT(false);

		return;
	}

	/* test PDAL */
	int PointCount = 0;
	//if (QFile::exists("F:/test.las"))
	//{
	//	using namespace pdal;
	//	using namespace pdal::Dimension;
	//	pdal::Option las_opt("filename", "F:/test.las");//参数1："filename"（键）
	//	pdal::Options las_opts;
	//	las_opts.add(las_opt);

	//	pdal::PointTable table;
	//	pdal::LasReader las_reader;
	//	las_reader.setOptions(las_opts);
	//	las_reader.prepare(table);

	//	pdal::PointViewSet point_view_set = las_reader.execute(table);
	//	pdal::PointViewPtr point_view = *point_view_set.begin();
	//	pdal::Dimension::IdList dims = point_view->dims();
	//	pdal::LasHeader las_header = las_reader.header();

	//	//头文件信息
	//	PointCount = las_header.pointCount();
	//}

	/*** HERE STARTS THE ACTION ***/

	// Put your code here
	// --> you may want to start by asking for parameters (with a custom dialog, etc.)

	// This is how you can output messages
	// Display a standard message in the console
	m_app->dispToConsole("[ExamplePlugin] Hello world!" + QString::number(PointCount), ccMainAppInterface::STD_CONSOLE_MESSAGE);

	// Display a warning message in the console
	m_app->dispToConsole("[ExamplePlugin] Warning: example plugin shouldn't be used as is", ccMainAppInterface::WRN_CONSOLE_MESSAGE);

	// Display an error message in the console AND pop-up an error box
	m_app->dispToConsole("Example plugin shouldn't be used - it doesn't do anything!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);

	/*** HERE ENDS THE ACTION ***/
}
