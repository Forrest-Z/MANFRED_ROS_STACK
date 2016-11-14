#include "gnuplot_gui.h"


namespace Gnuplot{

std::string& replaceAll(std::string& context, const std::string& from, const std::string& to)
{
	size_t lookHere = 0;
	size_t foundHere;
	while((foundHere = context.find(from, lookHere)) != std::string::npos)
	{
		context.replace(foundHere, from.size(), to);
		lookHere = foundHere + to.size();
	}
	return context;
}

GnuplotGui::GnuplotGui()
{
	if ( getenv ( "DISPLAY" ) == NULL )
	{
		this->valid = false;
		throw GnuplotException ( "cannot find DISPLAY variable" );
	}

	if ( !this->get_program_path ( "gnuplot" ) )
	{
		this->valid = false;
		throw GnuplotException ( "Can't find gnuplot in your PATH" );
	}

	this->gnucmd = popen ( "gnuplot", "w" );

	if ( !this->gnucmd )
	{
		this->valid = false;
		throw GnuplotException ( "Could'nt open connection to gnuplot" );
	}
	gs.x.min = 0.;
	gs.x.max = 0.;
	gs.y.min = 0.;
	gs.y.max = 0.;
	gs.xlabel = "X";
	gs.ylabel = "Y";
	tick_count = 0;
}

GnuplotGui::~GnuplotGui()
{
	if ( pclose ( this->gnucmd ) == -1 )
		std::cerr << "Problem closing communication to gnuplot" << std::endl;

	//if ((this->to_delete).size() > 0)
	//{
	//for (vector<string>::size_type i = 0; i < this->to_delete.size(); i++)
	//remove(this->to_delete[i].c_str());
	//to_delete.clear();
	//}
	return;
}



bool GnuplotGui::get_program_path ( const std::string pname )
{
	std::list<std::string> ls;
	char *path;

	path = getenv ( "PATH" );

	if ( !path )
	{
		std::cerr << "Path is not set" << std::endl;
		return false;
	}
	else
	{
		stringtok ( ls, path, ":" );

		for ( std::list<std::string>::const_iterator i = ls.begin();
					i != ls.end(); ++i )
		{
			std::string tmp = ( *i ) + "/" + pname;

			if ( access ( tmp.c_str(), X_OK ) == 0 )
				return true;
		}
	}

	return false;
}

void GnuplotGui::cmd(const std::string& cmdstr)
{
	__cmdstr = cmdstr;
	fputs(cmdstr.c_str(),this->gnucmd);
	fputs("\n",this->gnucmd);
	fflush(this->gnucmd);
	return;
}

void GnuplotGui::set_ylabel(const std::string &label)
{
	gs.ylabel = label;
	return;
}

void GnuplotGui::set_xlabel(const std::string &label)
{
	gs.xlabel = label;
	return;
}

// set the xrange
void GnuplotGui::set_xrange(int from, int to)
{
	gs.x.min = from;
	gs.x.max = to;
}
// set the yrange
void GnuplotGui::set_yrange(int from, int to)
{
	gs.y.min = from;
	gs.y.max = to;
}



void GnuplotGui::draw_point ( const Point& p, const std::string& tag, const std::string& title, const std::string& style )
{
	canvas.insert(make_pair(tag,Drawable(p,title,style)));
}



void GnuplotGui::clear_labels()
{
	labels.clear();
	this->cmd("unset label\n");
}


void GnuplotGui::clear_objects()
{
	objects.clear();
	this->cmd("unset object\n");
}

std::string GnuplotGui::__get_labels()
{
	std::ostringstream oss;
	for (LabelMap::iterator it=labels.begin();
				 it!=labels.end();
				 ++it
			)
	{
		Label& lbl = it->second;
		size_t delta = tick_count - lbl.lastValidTick;
		if (delta == 0) // a new label
		{
			oss << "set label "
					<< it->first << " at "
					<< lbl.location << " '"
					<< lbl.text	<< "';";
		}
		else if (delta <= GNUPLOT_TIME_WINDOW_FOR_VALID_DRAWING) // an updated drawing
		{
			oss << "set label "
					<< it->first << " at "
					<< lbl.location << ";";
		}
		else
		{
			oss << "unset label " << it->first << ";";
			labels.erase(it);
		}
	}
	
	return oss.str();
}

std::string GnuplotGui::__get_objects()
{
	std::ostringstream oss;
	for (ObjectMap::iterator it=objects.begin();
				it!=objects.end();
				++it
			)
	{
		Object& obj = it->second;
		size_t delta = tick_count - obj.lastValidTick;
		if (delta == 0) // a new object
		{
			oss << "set object "
					<< it->first << " " << obj.type
					<< " at " << obj.location
					<< " size " << obj.w << "," << obj.h << ";";
		}
		else if (delta <= GNUPLOT_TIME_WINDOW_FOR_VALID_DRAWING) // an updated label
		{
			oss << "set object "
					<< it->first << " " << obj.type
					<< " at " << obj.location << ";";
		}
		else
		{
			oss << "unset object " << it->first << ";";
			objects.erase(it);
		}
	}
	
	return oss.str();
}

void GnuplotGui::redraw()
{
	if (!canvas.size() > 0)
		return;

	std::ostringstream cmdstr;

	// settings
	if (gs.x.min != gs.x.max)
	{
		cmdstr << "set xrange [" << gs.x.min << ":" << gs.x.max << "];"; 
	}
	if (gs.y.min != gs.y.max)
	{
		cmdstr << "set yrange [" << gs.y.min << ":" << gs.y.max << "];";
	}
	cmdstr << "set xlabel '" << gs.xlabel << "';";
	cmdstr << "set ylabel '" << gs.ylabel << "';";
	cmdstr << __get_labels();
	cmdstr << __get_objects();

	cmdstr << std::endl;
	
	// plot
	cmdstr << "plot ";
	for (DrawableMap::const_iterator it=canvas.begin();
				it!=canvas.end();
				++it)
	{
		it!=canvas.begin() && cmdstr << ",";
		cmdstr << " '-' ";
		if (it->second.title != "notitle")
		{
			cmdstr << " title '" << it->second.title << "'";
		}
		else
		{
			cmdstr << it->second.title;
		}
		cmdstr << " " << it->second.style;
	}
	cmdstr << std::endl;

	for (DrawableMap::const_iterator it=canvas.begin();
				it != canvas.end();
				++it)
	{
		cmdstr << it->second.obj << " # " << it->first << std::endl << "e" << std::endl;
	}
	cmdstr << std::endl;
// 	std::cout << "Canvas Size: " << canvas.size() << " objects." << std::endl;
// 	std::cout << cmdstr.str() << std::endl;
	this->cmd(cmdstr.str());
	++tick_count;
}

void GnuplotGui::__draw_line(const CanvasObject& o, const std::string& tag, const std::string& title, const std::string& style )
{
	std::ostringstream sstyle;
	sstyle << "w l " << style;
	if (canvas.find(tag) != canvas.end())
	{
		canvas[tag].obj.append(o.toString());
	}
	else
	{
		canvas.insert(make_pair(tag,Drawable(o,title,sstyle.str())));
	}
}

}// end namespace

