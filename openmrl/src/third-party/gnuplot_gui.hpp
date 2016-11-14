
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

template <typename P>
void GnuplotGui::put_object(int tag, const std::string& type, float width, float height, float rotation, const P& location){
	Object& o = objects[tag];
	o.location = location.toString();
	replaceAll(o.location, " ", ",");
	o.type = type;
	o.w = width;
	o.h = height;
	o.r = rotation;
	o.lastValidTick = tick_count;
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

