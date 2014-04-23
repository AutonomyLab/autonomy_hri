#include "lkgrid_interface.h"



LikelihoodGridInterface::LikelihoodGridInterface():
    update_rate(0),
    update_time_ratio(0),
    free_cell_probability(0)
{
    ROS_INFO("Constructing an instace of LikelihoodGridInterface.");
}

LikelihoodGridInterface::LikelihoodGridInterface(ros::NodeHandle _n,
                                                 tf::TransformListener *_tf_listener,
                                                 GridFOV_t _globalGridFOV,
                                                float _update_rate,
                                                float _update_time_ratio,
                                                float _free_cell_probability,
                                                 float _unknown_cell_probability):
    n(_n),
    tf_listener(_tf_listener),
    globalGridFOV(_globalGridFOV),
    update_rate(_update_rate),
    update_time_ratio(_update_time_ratio),
    free_cell_probability(_free_cell_probability),
    unknown_cell_probability(_unknown_cell_probability)
{
    ROS_INFO("Constructing an instace of LikelihoodGridInterface.");
    tf_listener = new tf::TransformListener();
    legsLKGrid_pub = n.advertise<sensor_msgs::PointCloud>("legLKGrid",10);
    facesLKGrid_pub = n.advertise<sensor_msgs::PointCloud>("faceLKGrid",10);
    humanLKGrid_pub = n.advertise<sensor_msgs::PointCloud>("humanLKGrid",10);
}

geometry_msgs::PointStamped LikelihoodGridInterface::transform_to_base_footprint(geometry_msgs::PointStamped& tmp_point)
{

    geometry_msgs::PointStamped base_point;
    //tf::TransformListener listener;

    try{
//        listener.waitForTransform("base_footprint", tmp_point.header.frame_id, ros::Time(0),
//                    ros::Duration(5.0));
        tf_listener->transformPoint("base_footprint", ros::Time(0), tmp_point, tmp_point.header.frame_id, base_point);
//        ROS_INFO("%s: (%.2f, %.2f. %.2f) -----> base_footprint: (%.2f, %.2f, %.2f) at time %.2f",tmp_point.header.frame_id.c_str(),
//                 tmp_point.point.x, tmp_point.point.y, tmp_point.point.z, base_point.point.x, base_point.point.y, base_point.point.z,
//                 base_point.header.stamp.toSec());
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"base_footprint\": %s", tmp_point.header.frame_id.c_str(), ex.what());
    }
    return base_point;
}

void LikelihoodGridInterface::init_legs(GridFOV_t sensorGridFOV)
{
    try
    {
        legGrid = new LikelihoodGrid(sensorGridFOV, globalGridFOV, free_cell_probability, unknown_cell_probability);
    }
    catch (std::bad_alloc& ba)
    {
        std::cerr << "In new legGrid: bad_alloc caught: " << ba.what() << '\n';
    }
}


void LikelihoodGridInterface::legs_cb(const geometry_msgs::PoseArray& msg)
{
    std::vector<geometry_msgs::Pose> legs_laser;
    std::vector<PolarPose> legs_polar_base;

    if(!msg.poses.empty()){
        legs_polar_base.clear();
        lastLegsTime = ros::Time::now();

        // FRAME TRANSFORM: /laser to /base_footprint
        legs_laser = msg.poses;
        geometry_msgs::PointStamped tmp_laser, tmp_base;
        PolarPose tmp_polar;
        for(size_t i = 0; i < legs_laser.size(); i++){
            tmp_laser.header = msg.header;
            tmp_laser.point = legs_laser.at(i).position;
            tmp_base = transform_to_base_footprint(tmp_laser); // TRANSFORM TO BASE_FOOTPRINT
            tmp_polar.fromCart(tmp_base.point.x, tmp_base.point.y);
            legs_polar_base.push_back(tmp_polar);
        }

        assert(legs_polar_base.size() == legs_laser.size());
        leg_frame = "base_footprint";
        legGrid->assign(legs_polar_base);
        legs_polar_base.clear();
    }
}

void LikelihoodGridInterface::init_faces(GridFOV_t sensorGridFOV)
{
    try
    {
        faceGrid = new LikelihoodGrid(sensorGridFOV, globalGridFOV, free_cell_probability, unknown_cell_probability);
    }
    catch (std::bad_alloc& ba)
    {
        std::cerr << "In new faceGrid: bad_alloc caught: " << ba.what() << '\n';
    }
}

void LikelihoodGridInterface::faces_cb(const autonomy_human::human& msg)
{
    std::vector<PolarPose> face_polar_base;
    if(msg.numFaces > 0)
    {
        lastFacesTime = ros::Time::now();
        geometry_msgs::PointStamped tmp_camera, tmp_base;
        PolarPose tmp_polar;
        float theta, x;
        float image_width = 640.0;

        x = msg.faceROI.x_offset + msg.faceROI.width/2;
        theta = ((image_width/2-x)/image_width/2)*toRadian(65.0/2);

        //ROS_INFO("x: %f     theta: %f",x,theta);

//        tmp_camera.header = msg.header;
//        tmp_camera.point.x = msg.faceROI.x_offset + msg.faceROI.width/2;
//        tmp_camera.point.y = msg.faceROI.y_offset + msg.faceROI.height/2;
//        tmp_camera.point.z = 0.0; //TODO: FIX ME!
//        tmp_base = transform_to_base_footprint(tmp_camera); // TRANSFORM TO BASE_FOOTPRINT
//        tmp_polar.angle = atan2(tmp_base.point.y, tmp_base.point.x);
        tmp_polar.angle = theta;

        float r = faceGrid->sensorGridFOV.range.min;
        while(r <= faceGrid->sensorGridFOV.range.max){
            tmp_polar.range = r;
            r += faceGrid->sensorGridFOV.range.resolution;
            face_polar_base.push_back(tmp_polar);
        }
        face_frame = "base_footprint";
        faceGrid->assign(face_polar_base);
        face_polar_base.clear();
    }

}

void LikelihoodGridInterface::init_human()
{
    try
    {
        humanGrid = new LikelihoodGrid(globalGridFOV, globalGridFOV, free_cell_probability, unknown_cell_probability);
    }
    catch (std::bad_alloc& ba)
    {
        std::cerr << "In new humanGrid: bad_alloc caught: " << ba.what() << '\n';
    }
}

void LikelihoodGridInterface::spin()
{
    humanGrid->set(humanGrid->data, 1.0);

    // LEGS
    diffLegs = ros::Time::now() - lastLegsTime;
    legGrid->flag = !(diffLegs.toSec() > 2.0);
    legGrid->update(update_rate);
    //legGrid->normalize();


    // FACES
    diffFaces = ros::Time::now() - lastFacesTime;
    faceGrid->flag = !(diffFaces.toSec() > 2.0);
    faceGrid->update(update_rate);
    //faceGrid->normalize();

    // HUMAN
    humanGrid->fuse(legGrid->data);
    humanGrid->fuse(faceGrid->data);
//    humanGrid->fuse(legGrid->data, faceGrid->data);
    humanGrid->normalize();

    publish();
}

void LikelihoodGridInterface::publish()
{

    //LEGS
    sensor_msgs::PointCloud legLKGrid = LikelihoodGridInterface::pc_grid(legGrid);
    legLKGrid.header.stamp = ros::Time::now();
    legLKGrid.header.frame_id = "base_footprint";
    legsLKGrid_pub.publish(legLKGrid);

    // FACES
    sensor_msgs::PointCloud faceLKGrid = pc_grid(faceGrid);
    faceLKGrid.header.stamp = ros::Time::now();
    faceLKGrid.header.frame_id = "base_footprint";
    facesLKGrid_pub.publish(faceLKGrid);

    // HUMAN
    sensor_msgs::PointCloud humanLKGrid = pc_grid(humanGrid);
    humanLKGrid.header.stamp = ros::Time::now();
    humanLKGrid.header.frame_id = "base_footprint";
    humanLKGrid_pub.publish(humanLKGrid);
    //humanGrid->output();
}

sensor_msgs::PointCloud LikelihoodGridInterface::pc_grid(LikelihoodGrid* polar_grid)
{
    float r, t;
    geometry_msgs::Point32 points;
    sensor_msgs::PointCloud pointcloud_lkGrid;

    for(size_t i = 0; i < polar_grid->globalGridSize; i++){
        r = polar_grid->data[i].range;
        t = polar_grid->data[i].angle;
        points.x = r*cos(t);
        points.y = r*sin(t);
        points.z = polar_grid->data[i].probability;
        pointcloud_lkGrid.points.push_back(points);
    }
    return pointcloud_lkGrid;
}

LikelihoodGridInterface::~LikelihoodGridInterface()
{
    ROS_INFO("Deconstructing the constructed LikelihoodGridInterface.");
    delete legGrid;
    delete faceGrid;
    delete tf_listener;
}
