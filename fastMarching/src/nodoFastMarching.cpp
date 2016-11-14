#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"

#include "datatype/gridmap_includes.h"
#include "datatype/geometry/path2d.h"
#include "algorithms/planning/fm2pathplanner.h"

int main(int argc, char* argv[])
{

  ros::init(argc, argv, "nodo_fast_marching");

  std::string file_name = "../../mapas/Mapa_parcial_orig.bmp";
  double cell_size_m = 0.05;
  mr::Image* img = new mr::Image(1, 1);
  mr::LabeledGridMap* gridmap = new mr::LabeledGridMap();
  double x_min = 0;
  double y_min = 0;

  img->load(file_name);
  cout << "Width: " << img->getWidth() << " pixels     Height: " << img->getHeight() << " pixels" << std::endl;
  cout << "Width: " << cell_size_m * img->getWidth() << " m     Height: " << cell_size_m * img->getHeight() << " m" << std::endl;
  gridmap->loadFromImage(img, cell_size_m * img->getWidth(), cell_size_m, x_min, y_min);
  mr::FM2PathPlanner2d* fm2pathplanner = new mr::FM2PathPlanner2d(gridmap, true);
  fm2pathplanner->configure(1.5, 2);
  double x_init = 25;
  double y_init = 4.50;

  double x_goal = 275 * cell_size_m;
  double y_goal = (img->getHeight() - 206) * cell_size_m;

  // Origin
  mr::Pose* p_init = new mr::Pose(x_init, y_init);
  mr::Pose* p_goal = new mr::Pose(x_goal, y_goal);

  std::cout << "LLega.\n";

  bool path_found_star = fm2pathplanner->computePath(*p_init, *p_goal);

  if (path_found_star)
  {
    cout << "Camino encontrado" << endl;
  }
  else
  {
    cout << "Camino no encontrado" << endl;
  }

  mr::Path2D path;

  if (path_found_star)
  {
    path = fm2pathplanner->getPath();
    int x, y;
    for (unsigned int i = 0; i < path.points.size(); i++)
    {
      gridmap->WorldToGrid(path.points[i].x, path.points[i].y, x, y);
      cout << x << " " << y << " " << endl;
    }
  }

  delete img;
  delete gridmap;
  delete fm2pathplanner;
  delete p_init;
  delete p_goal;

}
