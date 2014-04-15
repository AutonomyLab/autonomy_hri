#include "lkgrid_interface.h"

LikelihoodGridInterface::LikelihoodGridInterface():
    update_rate(0),
    update_time_ratio(0),
    free_cell_probability(0)
{
    ROS_INFO("Constructing an instace of LikelihoodGridInterface.");
}

LikelihoodGridInterface::LikelihoodGridInterface(ros::NodeHandle _n,
                                                 GridFOV_t _globalGridFOV,
                                                float _update_rate,
                                                float _update_time_ratio,
                                                float _free_cell_probability):
    n(_n),
    globalGridFOV(_globalGridFOV),
    update_rate(_update_rate),
    update_time_ratio(_update_time_ratio),
    free_cell_probability(_free_cell_probability)
{
    ROS_INFO("Constructing an instace of LikelihoodGridInterface.");
    legsLKGrid_pub = n.advertise<sensor_msgs::PointCloud>("legLKGrid",10);
    facesLKGrid_pub = n.advertise<sensor_msgs::PointCloud>("faceLKGrid",10);
    humanLKGrid_pub = n.advertise<sensor_msgs::PointCloud>("humanLKGrid",10);
}

void LikelihoodGridInterface::init_legs(GridFOV_t sensorGridFOV)
{
    try
    {
        legGrid = new LikelihoodGrid(sensorGridFOV, globalGridFOV, free_cell_probability);
    } catch (std::bad_alloc& ba)
    {
        std::cerr << "In new legGrid: bad_alloc caught: " << ba.what() << '\n';
    }
}


void LikelihoodGridInterface::legs_cb(const geometry_msgs::PoseArray& msg)
{
    std::vector<geometry_msgs::Pose> legs;
    std::vector<PolarPose> legPoses;

    if(!msg.poses.empty()){
        lastLegsTime = ros::Time::now();
        legs = msg.poses;
        leg_frame = msg.header.frame_id;
        legPoses.clear();
        PolarPose tempPos;
        // TODO: before this step, make sure they are in the same coordinate system.
        for(size_t i = 0; i < legs.size(); i++){
            tempPos.fromCart(legs.at(i).position.x, legs.at(i).position.y);
            legPoses.push_back(tempPos);
        }
        assert(legPoses.size() == legs.size());
        legGrid->assign(legPoses);
        legPoses.clear();
    }
}

void LikelihoodGridInterface::init_faces(GridFOV_t sensorGridFOV)
{
    try
    {
        faceGrid = new LikelihoodGrid(sensorGridFOV, globalGridFOV, free_cell_probability);
    } catch (std::bad_alloc& ba)
    {
        std::cerr << "In new faceGrid: bad_alloc caught: " << ba.what() << '\n';
    }
}

void LikelihoodGridInterface::faces_cb(const autonomy_human::human& msg)
{
    geometry_msgs::Pose faces;
    std::vector<PolarPose> facePoses;
    PolarPose tempPos;
    if(msg.numFaces > 0)
    {
        lastFacesTime = ros::Time::now();
        face_frame = msg.header.frame_id;
        faces.position.x = msg.faceROI.x_offset + msg.faceROI.width/2;
        faces.position.y = msg.faceROI.y_offset + msg.faceROI.height/2;
        tempPos.angle = atan2(faces.position.y, faces.position.x);
        float r = faceGrid->sensorGridFOV.range.min;
        while(r <= faceGrid->sensorGridFOV.range.max){
            tempPos.range = r;
            r += faceGrid->sensorGridFOV.range.resolution;
            facePoses.push_back(tempPos);
        }
        faceGrid->assign(facePoses);
        facePoses.clear();
    }

}

void LikelihoodGridInterface::init_human()
{
    try
    {
        humanGrid = new LikelihoodGrid(globalGridFOV, globalGridFOV, free_cell_probability);
    } catch (std::bad_alloc& ba)
    {
        std::cerr << "In new humanGrid: bad_alloc caught: " << ba.what() << '\n';
    }
}

void LikelihoodGridInterface::spin(float cycleTime)
{
    humanGrid->set(humanGrid->data, 1.0);

    // LEGS
    diffLegs = ros::Time::now() - lastLegsTime;
//    legGrid->flag = !(diffLegs.toSec() > update_time_ratio*cycleTime);
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
    humanGrid->normalize();

    publish();
}

void LikelihoodGridInterface::publish()
{

    //LEGS
    sensor_msgs::PointCloud legLKGrid = LikelihoodGridInterface::pc_grid(legGrid);
    legLKGrid.header.stamp = ros::Time::now();
    legLKGrid.header.frame_id = leg_frame;
    legsLKGrid_pub.publish(legLKGrid);

    // FACES
    sensor_msgs::PointCloud faceLKGrid = pc_grid(faceGrid);
    faceLKGrid.header.stamp = ros::Time::now();
    faceLKGrid.header.frame_id = face_frame;
    facesLKGrid_pub.publish(faceLKGrid);

    // HUMAN
    sensor_msgs::PointCloud humanLKGrid = pc_grid(humanGrid);
    humanLKGrid.header.stamp = ros::Time::now();
    humanLKGrid.header.frame_id = leg_frame;
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
}
