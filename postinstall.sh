export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:~/velodyne_plugin/build
# Add the Velodyne library path to .bashrc
echo 'export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:~/velodyne_plugin/build' >> ~/.bashrc
source ~/.bashrc