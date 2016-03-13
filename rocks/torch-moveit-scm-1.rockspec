package = "torch-moveit"
version = "scm-1"

source = {
   url = "git://github.com/Xamla/torch-moveit.git",
}

description = {
   summary = "ROS moveit bindings for Torch",
   detailed = [[
   ]],
   homepage = "https://github.com/Xamla/torch-moveit",
   license = "BSD"
}

dependencies = {
   "torch >= 7.0",
   "torch-ros"
}

build = {
   type = "command",
   build_command = [[
cmake -E make_directory build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$(PREFIX)" && $(MAKE)
]],
   install_command = "cd build && $(MAKE) install"
}
