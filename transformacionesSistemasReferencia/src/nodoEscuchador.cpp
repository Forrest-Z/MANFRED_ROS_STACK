#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv) {

	ros::init(argc, argv, "nodoEscuchador");

	ros::NodeHandle nodoManejador;

	/* Here, we create a TransformListener object. A TransformListener object automatically
	 * subscribes to the transform message topic over ROS and manages all transform
	 * data coming in over the wire. Once the listener is created, it starts receiving tf
	 * transformations over the wire, and buffers them for up to 10 seconds.
	 * The TransformListener object should be scoped to persist otherwise it's cache will be unable
	 * to fill and almost every query will fail. A common method is to make the TransformListener
	 * object a member variable of a class.
	 */

	tf::TransformListener escuchadorTransformacion;
	tf::StampedTransform transformacion;
	ros::Time marcaTemporal;


	while (nodoManejador.ok()) {
		try {
			marcaTemporal = ros::Time::now();
			printf("\n");
			printf("Peticion %.4f \n", marcaTemporal.toSec());
			// You can also see we specified a time equal to 0. For tf, time 0 means "the latest available" transform in the buffer.
			// Now, change this line to get the transform at the current time, "now()":
			//escuchadorTransformacion.waitForTransform("\sr_sicks3000", "/sr_base", ros:Time)
			if (escuchadorTransformacion.waitForTransform("sr_mapa",
					"sr_hombro", marcaTemporal, ros::Duration(1.0))) {

				escuchadorTransformacion.lookupTransform("sr_mapa",
						"sr_hombro", marcaTemporal, transformacion);

				ROS_INFO(
						"Ubicacion hombro: (%.4f m, %.4f m, %.4f rad) \n",
						transformacion.getOrigin().x(),
						transformacion.getOrigin().y(),
						tf::getYaw(transformacion.getRotation()));
			}
		} catch (tf::TransformException& ex) {
			ROS_ERROR("%s", ex.what());
		}
	}

	return 0;
}
