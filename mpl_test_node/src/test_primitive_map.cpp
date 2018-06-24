#include <motion_primitive_library/planner/mp_map_util.h>
#include <motion_primitive_library/primitive/poly_solver.h>
#include <planning_ros_msgs/VoxelMap.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>
#include <planning_ros_utils/voxel_grid.h>
#include <ros/ros.h>

#include <iarc7_msgs/ObstacleArray.h>

void generateVoxelMap(planning_ros_msgs::VoxelMap& voxel_map,
                      const iarc7_msgs::ObstacleArray& obstacles) {
  ros::Time t1 = ros::Time::now();

  sensor_msgs::PointCloud cloud;

  // Generate cloud from obstacle data
  ROS_INFO("Number of obstacles: [%zu]", obstacles.obstacles.size());
  cloud.header.stamp = ros::Time::now();
  cloud.header.frame_id = "cloud";
  cloud.channels.resize(1);

  Vec3f voxel_map_origin;
  voxel_map_origin(0) = voxel_map.origin.x;
  voxel_map_origin(1) = voxel_map.origin.y;
  voxel_map_origin(2) = voxel_map.origin.z;

  ROS_INFO("Mapping obstacles to the cloud");
  for (auto& obstacle : obstacles.obstacles) {
    // Map each obstacle to the cloud

    float pipe_radius = obstacle.pipe_radius;
    float pipe_height = obstacle.pipe_height;
    float pipe_x = obstacle.odom.pose.pose.position.x;
    float pipe_y = obstacle.odom.pose.pose.position.y;
    float px, py, pz;
    for (pz = voxel_map_origin(2); pz <= pipe_height; pz += 0.1) {
      for (float theta = 0; theta < 2 * M_PI; theta += 0.15) {
        for (float r = pipe_radius - voxel_map.resolution; r < pipe_radius;
             r += voxel_map.resolution) {
          px = r * cos(theta) + pipe_x + voxel_map_origin(0);
          py = r * sin(theta) + pipe_y + voxel_map_origin(1);
          geometry_msgs::Point32 point;
          point.x = px;
          point.y = py;
          point.z = pz;
          cloud.points.push_back(point);
        }
      }
    }
  }

  ROS_INFO("Takes %f sec for mapping obstalces",
           (ros::Time::now() - t1).toSec());

  t1 = ros::Time::now();

  vec_Vec3f pts = cloud_to_vec(cloud);

  ROS_INFO("Takes %f sec for cloud to vec", (ros::Time::now() - t1).toSec());

  Vec3f voxel_map_dim;
  voxel_map_dim(0) = voxel_map.dim.x * voxel_map.resolution;
  voxel_map_dim(1) = voxel_map.dim.y * voxel_map.resolution;
  voxel_map_dim(2) = voxel_map.dim.z * voxel_map.resolution;

  std::unique_ptr<VoxelGrid> voxel_grid(
      new VoxelGrid(voxel_map_origin, voxel_map_dim, voxel_map.resolution));
  voxel_grid->addCloud(pts);

  voxel_map = voxel_grid->getMap();
  voxel_map.header = cloud.header;
}

void setMap(std::shared_ptr<MPL::VoxelMapUtil>& map_util,
            const planning_ros_msgs::VoxelMap& msg) {
  Vec3f ori(msg.origin.x, msg.origin.y, msg.origin.z);
  Vec3i dim(msg.dim.x, msg.dim.y, msg.dim.z);
  decimal_t res = msg.resolution;
  std::vector<signed char> map = msg.data;

  map_util->setMap(ori, dim, map, res);
}

void getMap(std::shared_ptr<MPL::VoxelMapUtil>& map_util,
            planning_ros_msgs::VoxelMap& map) {
  Vec3f ori = map_util->getOrigin();
  Vec3i dim = map_util->getDim();
  decimal_t res = map_util->getRes();

  map.origin.x = ori(0);
  map.origin.y = ori(1);
  map.origin.z = ori(2);

  map.dim.x = dim(0);
  map.dim.y = dim(1);
  map.dim.z = dim(2);
  map.resolution = res;

  map.data = map_util->getMap();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");

  ros::Publisher map_pub =
      nh.advertise<planning_ros_msgs::VoxelMap>("voxel_map", 1, true);
  ros::Publisher sg_pub =
      nh.advertise<sensor_msgs::PointCloud>("start_and_goal", 1, true);
  ros::Publisher prs_pub =
      nh.advertise<planning_ros_msgs::Primitives>("primitives", 1, true);
  ros::Publisher traj_pub =
      nh.advertise<planning_ros_msgs::Trajectory>("trajectory", 1, true);
  ros::Publisher refined_traj_pub = nh.advertise<planning_ros_msgs::Trajectory>(
      "trajectory_refined", 1, true);
  ros::Publisher cloud_pub =
      nh.advertise<sensor_msgs::PointCloud>("cloud", 1, true);

  ros::Time t1 = ros::Time::now();

  // Standard header
  std_msgs::Header header;
  header.frame_id = std::string("map");

  planning_ros_msgs::VoxelMap map;
  map.resolution = 0.25;
  map.origin.x = 0.0;
  map.origin.y = 0.0;
  map.origin.z = 0.0;
  map.dim.x = 179.0;
  map.dim.y = 179.0;
  map.dim.z = 39.0;
  std::vector<int8_t> data(map.dim.x * map.dim.y * map.dim.z, 0);
  map.data = data;

  iarc7_msgs::ObstacleArray obstacles;

  iarc7_msgs::Obstacle new_obstacle;
  new_obstacle.pipe_height = 2.0;
  new_obstacle.pipe_radius = 1.0;
  new_obstacle.odom.pose.pose.position.x = 9;
  new_obstacle.odom.pose.pose.position.y = 11;
  new_obstacle.odom.pose.pose.position.z = 0;
  obstacles.obstacles.push_back(new_obstacle);

  for (double i = 0; i < 2; i += 0.8) {
    new_obstacle.pipe_height = 1.5;
    new_obstacle.pipe_radius = 0.7;
    new_obstacle.odom.pose.pose.position.x = 9 - i * 2;
    new_obstacle.odom.pose.pose.position.y = 9 + 0.5 * i;
    new_obstacle.odom.pose.pose.position.z = 0;
    obstacles.obstacles.push_back(new_obstacle);
  }

  generateVoxelMap(map, obstacles);

  // Initialize map util
  std::shared_ptr<MPL::VoxelMapUtil> map_util(new MPL::VoxelMapUtil);
  setMap(map_util, map);

  map_util->info();

  // Free unknown space and dilate obstacles
  map_util->freeUnknown();
  // map_util->dilate(0.2, 0.1);

  ROS_INFO("Takes %f sec for building map", (ros::Time::now() - t1).toSec());

  // Publish the dilated map for visualization
  getMap(map_util, map);
  map.header = header;
  map_pub.publish(map);
  try {
    // Set start and goal
    double start_x, start_y, start_z;
    nh.param("start_x", start_x, 12.5);
    nh.param("start_y", start_y, 1.4);
    nh.param("start_z", start_z, 0.0);
    double start_vx, start_vy, start_vz;
    nh.param("start_vx", start_vx, 0.5);
    nh.param("start_vy", start_vy, 0.5);
    nh.param("start_vz", start_vz, 0.1);
    double goal_x, goal_y, goal_z;
    nh.param("goal_x", goal_x, 6.4);
    nh.param("goal_y", goal_y, 16.6);
    nh.param("goal_z", goal_z, 0.0);

    Waypoint3 start;
    start.pos = Vec3f(start_x, start_y, start_z);
    start.vel = Vec3f(.5, .5, .1);
    start.acc = Vec3f(0, 0, 0);
    start.use_pos = true;
    start.use_vel = false;
    start.use_acc = false;
    start.use_jrk = false;

    Waypoint3 goal;
    goal.pos = Vec3f(goal_x, goal_y, goal_z);
    goal.vel = Vec3f(1, 1, 1);
    goal.acc = Vec3f(0, 0, 0);
    goal.use_pos = start.use_pos;
    goal.use_vel = start.use_vel;
    goal.use_acc = start.use_acc;
    goal.use_jrk = start.use_jrk;

    // Initialize planner
    double dt, eps, v_max, a_max, j_max, u_max1, u_max;
    int max_num, num, ndt;
    nh.getParam("dt", dt);
    nh.getParam("eps", eps);
    nh.getParam("ndt", ndt);
    nh.getParam("v_max", v_max);
    nh.getParam("a_max", a_max);
    nh.getParam("j_max", j_max);
    nh.getParam("u_max", u_max);
    nh.param("max_num", max_num, -1);
    nh.getParam("num", num);

    double p_tol, v_tol, a_tol;
    nh.getParam("p_tol", p_tol);
    nh.getParam("v_tol", v_tol);
    nh.getParam("a_tol", a_tol);

    u_max1 = u_max;
    u_max = v_max;

    vec_Vec3f U;
    const decimal_t du = u_max / num;
    for (decimal_t dx = -u_max; dx <= u_max; dx += du)
      for (decimal_t dy = -u_max; dy <= u_max; dy += du)
        for (decimal_t dz = -u_max; dz <= u_max; dz += du)
          U.push_back(Vec3f(dx, dy, dz));

    std::unique_ptr<MPMap3DUtil> planner_ptr;
    planner_ptr.reset(new MPMap3DUtil(true));
    planner_ptr->setMapUtil(map_util);  // Set collision checking function
    planner_ptr->setEpsilon(eps);       // Set greedy param (default equal to 1)
    planner_ptr->setVmax(v_max);        // Set max velocity
    // planner_ptr->setAmax(a_max); // Set max acceleration (as control input)
    planner_ptr->setUmax(u_max);     // 2D discretization with 1
    planner_ptr->setDt(dt);          // Set dt for each primitive
    planner_ptr->setTmax(ndt * dt);  // Set the planning horizon: n*dt
    planner_ptr->setMaxNum(200000);  // Set maximum allowed expansion, -1 means no limitation
    planner_ptr->setU(U);
    planner_ptr->setTol(p_tol);  // Tolerance for goal region

    // Planning thread!
    ros::Time t0 = ros::Time::now();
    bool valid = planner_ptr->plan(start, goal);

    if (!valid) {
      ROS_WARN("Failed! Takes %f sec for planning, expand [%zu] nodes",
               (ros::Time::now() - t0).toSec(),
               planner_ptr->getCloseSet().size());
    } else {
      ROS_INFO("Succeed! Takes %f sec for planning, expand [%zu] nodes",
               (ros::Time::now() - t0).toSec(),
               planner_ptr->getCloseSet().size());
    }

    auto traj = planner_ptr->getTraj();

    U.clear();
    u_max = u_max1;

    const decimal_t du2 = u_max / num;
    for (decimal_t dx = -u_max; dx <= u_max; dx += du2)
      for (decimal_t dy = -u_max; dy <= u_max; dy += du2)
        for (decimal_t dz = -u_max; dz <= u_max; dz += du2)
          U.push_back(Vec3f(dx, dy, dz));

    // reset planner
    planner_ptr->setU(U);
    planner_ptr->setUmax(u_max);  // 2D discretization with 1
    planner_ptr->setAmax(a_max);  // Set max acceleration (as control input)
    // planner_ptr->setPriorTrajectory(traj);

    start.pos = Vec3f(start_x, start_y, start_z);
    start.vel = Vec3f(start_vx, start_vy, start_vz);
    start.acc = Vec3f(0, 0, 0);
    start.use_pos = true;
    start.use_vel = true;
    start.use_acc = true;
    start.use_jrk = false;

    goal.pos = Vec3f(goal_x, goal_y, goal_z);
    goal.vel = Vec3f(0, 0, 0);
    goal.acc = Vec3f(1, 1, 0.2);
    goal.use_pos = start.use_pos;
    goal.use_vel = start.use_vel;
    goal.use_acc = start.use_acc;
    goal.use_jrk = start.use_jrk;

    // Publish location of start and goal
    sensor_msgs::PointCloud sg_cloud;
    sg_cloud.header = header;
    geometry_msgs::Point32 pt1, pt2;
    pt1.x = start_x, pt1.y = start_y, pt1.z = start_z;
    pt2.x = goal_x, pt2.y = goal_y, pt2.z = goal_z;
    sg_cloud.points.push_back(pt1), sg_cloud.points.push_back(pt2);
    sg_pub.publish(sg_cloud);

    // Planning thread!
    t0 = ros::Time::now();
    valid = planner_ptr->plan(start, goal);

    // Publish expanded nodes
    sensor_msgs::PointCloud ps = vec_to_cloud(planner_ptr->getCloseSet());
    ps.header = header;
    cloud_pub.publish(ps);

    // Publish primitives
    planning_ros_msgs::Primitives prs_msg =
        toPrimitivesROSMsg(planner_ptr->getPrimitivesToGoal());
    prs_msg.header = header;
    // prs_pub.publish(prs_msg);

    if (!valid) {
      ROS_WARN("Failed! Takes %f sec for planning, expand [%zu] nodes",
               (ros::Time::now() - t0).toSec(),
               planner_ptr->getCloseSet().size());
    } else {
      ROS_INFO("Succeed! Takes %f sec for planning, expand [%zu] nodes",
               (ros::Time::now() - t0).toSec(),
               planner_ptr->getCloseSet().size());
    }

    // Publish trajectory
    traj = planner_ptr->getTraj();
    planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);
    traj_msg.header = header;
    // traj_pub.publish(traj_msg);

    printf("==================  Raw traj -- total J: %f, total time: %f\n",
           traj.J(2), traj.getTotalTime());

    // Get intermediate waypoints
    vec_E<Waypoint3> waypoints = planner_ptr->getWs();

    // Get time allocation
    std::vector<decimal_t> dts;
    dts.resize(waypoints.size() - 1, dt);

    // Generate higher order polynomials
    PolySolver3 poly_solver(2, 3);
    poly_solver.solve(waypoints, dts);

    auto traj_refined =
        Trajectory3(poly_solver.getTrajectory()->toPrimitives());

    // Publish refined trajectory
    planning_ros_msgs::Trajectory refined_traj_msg =
        toTrajectoryROSMsg(traj_refined);
    refined_traj_msg.header = header;
    // refined_traj_pub.publish(refined_traj_msg);

    printf("================ Refined traj -- total J: %f, total time: %f\n",
           traj_refined.J(2), traj_refined.getTotalTime());
  } catch (const std::exception& e) {
    ROS_ERROR("Exception: %s", e.what());
    return 0;
  }

  ros::spin();

  return 0;
}
