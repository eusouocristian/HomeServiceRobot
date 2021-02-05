// After writing the c++ code, create the CMakeLists.txt on the project root
// compile the code using cmake command on the linux terminal:
// cd into the build folder
// run: >> cmake ../
// run: >> make
// include libhello.so file on the path using the following command: (change the path)
// export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/Documents/udacity/Robozone/build

// copy and paste: <plugin name="hello" filename="libhello.so"/>
// under: <world name='default'>
// in the <world_name>.sdf file 

// cd into world folder and load the gazebo world file with the plugin.
// run: >> gazebo <myworld_filename>.sdf --verbose
#include <gazebo/gazebo.hh>

namespace gazebo
{
  class WorldPluginMyRobot : public WorldPlugin
  {
    public: WorldPluginMyRobot() : WorldPlugin()
            {
              printf("Welcome to my world! Please come on in!\n");
              printf("That's a pleasure to have you! Cristian\n");
            }

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
            {
            }
  };
  GZ_REGISTER_WORLD_PLUGIN(WorldPluginMyRobot)
}
