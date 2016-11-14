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

/////////////////////////////
//
// A string tokenizer taken from
// http://www.sunsite.ualberta.ca/Documentation/Gnu/libstdc++-2.90.8/html/21_strings/stringtok_std_h.txt
//
/////////////////////////////
#define PATH_MAXNAMESZ       4096
template <typename Container>
void
stringtok ( Container &container, std::string const &in,
						const char * const delimiters = " \t\n" )
{
	const std::string::size_type len = in.length();
	std::string::size_type i = 0;

	while ( i < len )
	{
		// eat leading whitespace
		i = in.find_first_not_of ( delimiters, i );

		if ( i == std::string::npos )
			return;   // nothing left but white space

		// find the end of the token
		std::string::size_type j = in.find_first_of ( delimiters, i );

		// push token
		if ( j == std::string::npos )
		{
			container.push_back ( in.substr ( i ) );
			return;
		}
		else
			container.push_back ( in.substr ( i, j - i ) );

		// set up for next loop
		i = j + 1;
	}
}

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

template <typename P>
void GnuplotGui::put_label(int tag, const std::string& text, const P& location)
{
	Label& l = labels[tag];
	l.location = location.toString();
	replaceAll(l.location, " ", ",");
	l.lastValidTick = tick_count;
	if ( l.text.compare(text) != 0)
	{
		l.text = text;
	}
}

void GnuplotGui::clear_labels()
{
	labels.clear();
	this->cmd("unset label\n");
}

template <typename P>
void GnuplotGui::put_object(int tag, const std::string& type, float width, float height, float rotation, const P& location)
{
// 	ObjectMap::const_iterator it=objects.find(tag);
// 	if ((it!=objects.end()) && (it->second.text.compare(text) == 0))
// 	{
// 		Object& o = objects[tag];
// 		o.location = location.toString();
// 		o.lastValidTick = tick_count;
// 		o.w = width;
// 		o.h = height;
// 		o.r = rotation;
// 		replaceAll(o.location, " ", ",");
// 	}
// 	else
// 	{
// 		Objects gpo;
// 		gpo.text = text;
// 		gpo.location = location.toString();
// 		gpo.w = width;
// 		gpo.h = height;
// 		gpo.r = rotation;
// 		replaceAll(gpo.location, " ", ",");
// 		gpo.lastValidTick = tick_count;
// 		objects[tag] = gpo;
// 	}
	Object& o = objects[tag];
	o.location = location.toString();
	replaceAll(o.location, " ", ",");
	o.type = type;
	o.w = width;
	o.h = height;
	o.r = rotation;
	o.lastValidTick = tick_count;
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

void GnuplotGui::draw_point ( const Point& p, const std::string& tag, const std::string& title, const std::string& style )
{
	canvas.insert(make_pair(tag,Drawable(p,title,style)));
}

template <typename P>
void GnuplotGui::draw_points ( const std::vector<P>& vp, const std::string& tag, const std::string& title, const std::string& style )
{
	std::ostringstream obj;
	for (typename std::vector<P>::const_iterator it=vp.begin(); it!=vp.end(); ++it )
	{
		obj << it->toString() << std::endl;
	}
	if (canvas.find(tag) != canvas.end())
	{
		canvas[tag].obj.append(obj.str());
	}
	else
	{
		std::ostringstream sstyle;
		sstyle << "w p " << style;
		canvas.insert(make_pair(tag,Drawable(obj.str(),title,sstyle.str())));
	}
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

template <typename P>
void GnuplotGui::draw_line ( const Line<P>& l, const std::string& tag, const std::string& title, const std::string& style )
{
	__draw_line(l, tag, title, style);
}

template <typename P>
void GnuplotGui::draw_line ( const P& p0, const P& p1, const std::string& tag, const std::string& title, const std::string& style )
{
	Line<P> l(p0,p1);
	__draw_line(l, tag, title, style);
}

template <typename P>
void GnuplotGui::draw_lines ( const std::vector<Line<P> >& vl, const std::string& tag, const std::string& title, const std::string& style )
{
	std::ostringstream sstyle, obj;
	sstyle << "w l " << style;
	for (typename std::vector<Line<P> >::const_iterator it=vl.begin(); it!=vl.end(); ++it )
	{
		obj << it->toString() << std::endl << std::endl;
	}
	if (canvas.find(tag) != canvas.end())
	{
		canvas[tag].obj.append(obj.str());
	}
	else
	{
		canvas.insert(make_pair(tag,Drawable(obj.str(),title,sstyle.str())));
	}
}

template <typename P>
void GnuplotGui::draw_path(const Path<P>& p, const std::string& tag, const std::string& title, const std::string& style)
{
	__draw_line(p, tag, title, style);
}

template <typename P>
void GnuplotGui::draw_path(const std::vector<P>& vp, const std::string& tag, const std::string& title, const std::string& style)
{
	std::ostringstream sstyle, obj;
	sstyle << "w l " << style;
	for (typename std::vector<P>::const_iterator it=vp.begin(); it!=vp.end(); ++it )
	{
		obj << it->toString() << std::endl;
	}
	if (canvas.find(tag) != canvas.end())
	{
		canvas[tag].obj.append(obj.str());
	}
	else
	{
		canvas.insert(make_pair(tag,Drawable(obj.str(),title,sstyle.str())));
	}
}

template <typename P>
void GnuplotGui::draw_paths(const std::vector<Path<P> >& vp, const std::string& tag, const std::string& title, const std::string& style)
{
	std::ostringstream sstyle, obj;
	sstyle << "w l " << style;
	for (typename std::vector<Path<P> >::const_iterator it=vp.begin(); it!=vp.end(); ++it )
	{
		obj << it->toString() << std::endl;
	}
	if (canvas.find(tag) != canvas.end())
	{
		canvas[tag].obj.append(obj.str());
	}
	else
	{
		canvas.insert(make_pair(tag,Drawable(obj.str(),title,sstyle.str())));
	}
}

