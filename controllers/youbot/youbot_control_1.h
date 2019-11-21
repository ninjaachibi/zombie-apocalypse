



#include <math.h>
#include <stdio.h>







void rotate_robot(int angle)
{
	WbNodeRef robot_node = wb_supervisor_node_get_from_def("Youbot");
    WbFieldRef rot_field = wb_supervisor_node_get_field(robot_node, "rotation");
    double rotation[4];
	if (angle == 0) { rotation[0] = 1; rotation[1] = 0; rotation[2] = 0; rotation[3] = -1.57; }
	if (angle == 15) { rotation[0] = -0.985; rotation[1] = 0.126; rotation[2] = 0.122; rotation[3] = 1.59; }
	if (angle == 30) { rotation[0] = -0.938; rotation[1] = 0.247; rotation[2] = 0.244; rotation[3] = 1.63; }
	if (angle == 45) { rotation[0] = -0.866; rotation[1] = 0.355; rotation[2] = 0.352; rotation[3] = 1.71; }
	if (angle == 60) { rotation[0] = -0.778; rotation[1] = 0.445; rotation[2] = 0.443; rotation[3] = 1.82; }
	if (angle == 75) { rotation[0] = -0.681; rotation[1] = 0.519; rotation[2] = 0.516; rotation[3] = 1.94; }
	if (angle == 90) { rotation[0] = -0.581; rotation[1] = 0.577; rotation[2] = 0.572; rotation[3] = 2.09; }
	if (angle == 105) { rotation[0] = -0.48; rotation[1] = 0.621; rotation[2] = 0.619; rotation[3] = 2.24; }
	if (angle == 120) { rotation[0] = -0.381; rotation[1] = 0.654; rotation[2] = 0.653; rotation[3] = 2.41; }
	if (angle == 135) { rotation[0] = -0.284; rotation[1] = 0.679; rotation[2] = 0.677; rotation[3] = 2.58; }
	if (angle == 150) { rotation[0] = -0.189; rotation[1] = 0.695; rotation[2] = 0.694; rotation[3] = 2.76; }
	if (angle == 165) { rotation[0] = -0.095; rotation[1] = 0.704; rotation[2] = 0.704; rotation[3] = 2.95; }
	if (angle == 180) { rotation[0] = 0.000; rotation[1] = 0.707; rotation[2] = 0.707; rotation[3] = 3.13; }
	if (angle == 195) { rotation[0] = 0.090; rotation[1] = 0.704; rotation[2] = 0.704; rotation[3] = -2.97; }
	if (angle == 210) { rotation[0] = 0.183; rotation[1] = 0.695; rotation[2] = 0.695; rotation[3] = -2.78; }
	if (angle == 225) { rotation[0] = 0.278; rotation[1] = 0.679; rotation[2] = 0.680; rotation[3] = -2.6; }
	if (angle == 240) { rotation[0] = 0.375; rotation[1] = 0.655; rotation[2] = 0.656; rotation[3] = -2.43; }
	if (angle == 255) { rotation[0] = 0.473; rotation[1] = 0.622; rotation[2] = 0.624; rotation[3] = -2.26; }
	if (angle == 270) { rotation[0] = 0.574; rotation[1] = 0.578; rotation[2] = 0.580; rotation[3] = -2.10; }
	if (angle == 285) { rotation[0] = 0.674; rotation[1] = 0.521; rotation[2] = 0.524; rotation[3] = -1.96; }
	if (angle == 300) { rotation[0] = 0.771; rotation[1] = 0.449; rotation[2] = 0.452; rotation[3] = -1.83; }
	if (angle == 315) { rotation[0] = 0.860; rotation[1] = 0.360; rotation[2] = 0.363; rotation[3] = -1.72; }
	if (angle == 330) { rotation[0] = 0.933; rotation[1] = 0.254; rotation[2] = 0.257; rotation[3] = -1.64; }
	if (angle == 345) { rotation[0] = 0.982; rotation[1] = 0.133; rotation[2] = 0.137; rotation[3] = -1.59; }
	wb_supervisor_field_set_sf_rotation(rot_field,rotation);
	
}


void go_forward()
{
	base_forwards();
}

void stop()
{
	base_reset();
}

void robot_control()
{
	////////////// TO ROTATE THE ROBOT (BETWEEN 0 - 345) WITH 15 DEGREE INTERVALS ///////////////
	//rotate_robot(45);
	//rotate_robot(255);
	/////////////////////////////////////////////////////////////////////////////////////////////
	
	////////////// TO MOVE ROBOT FORWARD AND TO STOP IT /////////////////////////////////////////
	// go_forward();
	// stop();
	/////////////////////////////////////////////////////////////////////////////////////////////
	
	////////////// TO GET RGB FROM THE CAMERA ///////////////////////////////////////////////////
	//const unsigned char *image = wb_camera_get_image(3);
	//for (int x = 0; x < 64; x++)
	//{
		//for (int y = 0; y < 64; y++) 
		//{
			//int r = wb_camera_image_get_red(image, 64, x, y);
			//int g = wb_camera_image_get_green(image, 64, x, y);
			//int b = wb_camera_image_get_blue(image, 64, x, y);
			////printf("red=%d, green=%d, blue=%d", r, g, b);
		//}
	//}
	/////////////////////////////////////////////////////////////////////////////////////////////
	
}
