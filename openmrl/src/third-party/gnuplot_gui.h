#ifndef __gnuplot_gui_h__
#define __gnuplot_gui_h__

#include <string>
#include <sstream>
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <list>
#include <vector>
#include <cstdio>
#include <map>
#include <cstdlib>
#include <unistd.h>


namespace Gnuplot
{

#define GP_CMD_SIZE         1024

class GnuplotException : public std::runtime_error
{

	public:
		GnuplotException ( const std::string &msg ) : runtime_error ( "Gnuplot Exception: " + msg ) {}
};

class CanvasObject
{

	public:
		CanvasObject() {};
		virtual ~CanvasObject() {};
		virtual std::string toString() const { return ""; };
};

typedef CanvasObject Point;

class Point2d: public Point
{

	public:
		Point2d ( float _x=0., float _y=0. ) :
				x ( _x ),
				y ( _y )
		{}

		float x;
		float y;

		virtual std::string toString() const
		{
			std::ostringstream oss;
			oss << std::setprecision ( 3 ) << x << " " << y;
			return oss.str();
		}
};

class Point3d: public Point2d
{

	public:
		float z;

		Point3d ( float _x=0., float _y=0., float _z=0. ) :
				Point2d ( _x, _y ),
				z ( _z )
		{}

		virtual std::string toString() const
		{
			std::ostringstream oss;
			oss << this->Point2d::toString() << " " << std::setprecision ( 3 ) << z;
			return oss.str();
		}
};

template <typename P>
class Line: public CanvasObject
{

	public:
		P p0, p1;

		Line ( const P& _p0, const P& _p1 ) :
				p0 ( _p0 ),
				p1 ( _p1 )
		{}

		virtual std::string toString() const
		{
			std::ostringstream oss;
			oss << p0.toString() << std::endl << p1.toString();
			return oss.str();
		}
};

template <typename P>
class Path: public CanvasObject
{
	private:
		std::vector<P> m_data;

	public:

		virtual std::string toString() const
		{
			std::ostringstream oss;
			for (typename std::vector<P>::const_iterator it=m_data.begin(); it!=m_data.end(); ++it )
			{
				oss << it->toString() << std::endl;
			}
			return oss.str();
		}

		void push ( const P& p )
		{
			m_data.push_back ( p );
		}

		const P& back() const
		{
			return m_data.back();
		}

		const P& end()
		{
			return *(m_data.end());
		}

		size_t length()
		{
			return m_data.size();
		}
};

struct Drawable
{
	std::string obj;
	std::string title;
	std::string style;

	Drawable ():
			obj(""),
			title(""),
			style("")
	{}
			

	Drawable ( const CanvasObject& o, const std::string& t, const std::string& s ) :
			title ( t ),
			style ( s )
	{
		obj = o.toString();
	}
	
	Drawable ( const std::string& o, const std::string& t, const std::string& s ) :
			obj(o),
			title ( t ),
			style ( s )
	{}
};

struct Range
{
	int min,max;
};

struct Settings
{
	Range x,y;
	std::string xlabel,ylabel;
};

struct Label
{
	std::string text;
	std::string location;
	long int lastValidTick;
};

struct Object
{
	std::string type;
	std::string location;
	float w, h, r;
	long int lastValidTick;
};

static const size_t GNUPLOT_TIME_WINDOW_FOR_VALID_DRAWING = 1; // how long a drawing is persistent: 0 means never, 1 or more specify how many ticks a drawing is shown

class GnuplotGui
{

	public:
		GnuplotGui();
		virtual ~GnuplotGui();

		void clear() { canvas.clear(); clear_labels(); clear_objects(); }
		void redraw();
		void draw_point ( const Point& p, const std::string& tag, const std::string& title = "notitle", const std::string& style = "" );
		template <typename P>
		void draw_points ( const std::vector<P>& p, const std::string& tag, const std::string& title = "notitle", const std::string& style = "" );

		template <typename P>
		void draw_line ( const Line<P>& l, const std::string& tag, const std::string& title = "notitle", const std::string& style = "" );
		template <typename P>
		void draw_line ( const P& p0, const P& p1,  const std::string& tag, const std::string& title = "notitle", const std::string& style = "" );
		template <typename P>
		void draw_lines ( const std::vector<Line<P> >& vl,  const std::string& tag, const std::string& title = "notitle", const std::string& style = "" );
		template <typename P>
		void draw_path ( const Path<P>& p,  const std::string& tag, const std::string& title = "notitle", const std::string& style = "" );
		template <typename P>
		void draw_path ( const std::vector<P>& vp,  const std::string& tag, const std::string& title = "notitle", const std::string& style = "" );
		template <typename P>
		void draw_paths ( const std::vector<Path<P> >& vp,  const std::string& tag, const std::string& title = "notitle", const std::string& style = "" );

		template <typename P>
		void put_label(int tag, const std::string& text, const P& location);
		void clear_labels();
		
		template <typename P>
		void put_object(int tag, const std::string& type, float width, float height, float rotation, const P& location);
		void clear_objects();

		// send a command to gnuplot
// 		void cmd ( const char*, ... );
		void cmd ( const std::string& cmdstr );
		void set_xrange(int from, int to);
		void set_yrange(int from, int to);
		void set_xlabel(const std::string& label);
		void set_ylabel(const std::string& label);

		typedef std::map<std::string, Drawable> DrawableMap;
		typedef std::map<int, Label> LabelMap;
		typedef std::map<int, Object> ObjectMap;

		const std::string& getCmd() { return __cmdstr; }

	private:
		FILE *gnucmd;
		Settings gs;
		long int tick_count;
		bool valid;
		bool get_program_path ( const std::string );
		DrawableMap canvas;
		LabelMap labels;
		ObjectMap objects;

		void __draw_line(const CanvasObject& o, const std::string& tag, const std::string& title, const std::string& style);
		std::string __get_labels();
		std::string __get_objects();
		std::string __cmdstr;
};

std::string& replaceAll(std::string& context, const std::string& from, const std::string& to);

#include "gnuplot_gui.hpp"

}


#endif
