import os
import sys
import yaml
import shutil
from pathlib import Path


def camel_case(s):
    newString = ''
    temp = s.split('_')
    for st in temp:
        newString += (st[0].upper() + st[1:])
    return newString
    
def createNodePython(name, node):
    path = f"{Path(__file__).parents[1]}/src"
   
    with open(f"{path}/{name}/{name}/{node}.py", "w") as file:

        data = ["import rclpy",
                "from rclpy.node import Node",
                "import time",
                "",
                "",
                f"class {camel_case(name)}(Node):",
                "\tdef __init__(self):",
                f"\t\tsuper().__init__('{node}')",
                '',
                "\t\tself.declare_parameters(",
                '\t\t\tnamespace="",',
                '\t\t\tparameters=[',
                '',
                '\t\t\t]',
                '\t\t)',
                "",
                '\t\tself.sim_time = self.get_parameter("use_sim_time").get_parameter_value().bool_value',
                '',
                '\t\tself.get_logger().info(f"sim_time is set to {self.sim_time}")',
                '',
                "\t\ttime.sleep(1)",
                "",
                "",
                "",
                "def main(args=None):",
                "\trclpy.init(args=args)",
                "",
                f"\t{name} = {camel_case(name)}()",
                "",
                f"\trate = {name}.create_rate(20)",
                "\twhile rclpy.ok():",
                f"\t\trclpy.spin_once({name})",
                "",
                f"\t{name}.destroy_node()",
                "\trclpy.shutdown()"
                
                ]

        for line in data:
           file.write(line + '\n')


   

    with open(f"{path}/{name}/setup.py", "r") as f:
        data = f.readlines()

    data.insert(1, "import os\n")
    data.insert(2, "from glob import glob\n")
    data.insert(14, "\t\t(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),\n")
    data.insert(15, "\t\t(os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*[yaml]*'))),\n")
    data.insert(30, f"\t\t\t'{node} = {name}.{node}:main'\n")

    s = ""
    for i in data:
        s += i

    os.remove(f"{path}/{name}/setup.py")

    with open(f"{path}/{name}/setup.py", "w") as f:
        f.write(s)
    
    os.mkdir(f"{path}/{name}/config")
    os.mkdir(f"{path}/{name}/launch")

    with open(f"{path}/{name}/config/params.yaml", "w") as f:
        data = [{f"{node}": {'ros__parameters': {'use_sim_time': False}}}]
        yaml.dump_all(data, f)

    with open(f"{path}/{name}/launch/{name}.launch.py", "w") as f:
        
        data = [
            "import os",
            " ",
            "from ament_index_python.packages import get_package_share_directory",
            "from launch.substitutions import LaunchConfiguration",
            "from launch.actions import DeclareLaunchArgument",
            "from launch import LaunchDescription",
            "from launch_ros.actions import Node",
            "from pathlib import Path",
            " ",
            " ",
            "def generate_launch_description():",
            " ",
                f"\tpackage_name = '{name}'\n",
            " ",
            "\tparams_file = os.path.join(get_package_share_directory(package_name), 'config', 'params.yaml')",
            "\tcwd = f'{Path(__file__).parents[5]}/src/{package_name}/config/params.yaml'",
	        "\tos.system(f'cp {cwd} {params_file}')",
            " ",
            f"\t{name} = Node(",
            "\t\tpackage=package_name,",
            f"\t\texecutable='{node}',",
            "\t\tparameters=[params_file]",
            " ",
            "\t)",
            " ",
            "\treturn LaunchDescription([",
            f"\t\t{name}",
            "\t])"
        ]

        for line in data:
           f.write(line + '\n')

def createNodeCpp(name, node):
    path = f"{Path(__file__).parents[1]}/src"
    with open(f"{path}/{name}/src/{node}.cpp", "w") as file:

        data = [
            "#include <iostream>",
            "#include <chrono>",
            "#include <functional>",
            "#include <memory>",
            "#include <string>",
            "#include <vector>",
            "#include <ament_index_cpp/get_package_share_directory.hpp>",
            " ",
            " ",
            '#include "rclcpp/rclcpp.hpp"',
            " ",
            "using namespace std::chrono_literals;",
            " ",
            f"class {camel_case(name)} : public rclcpp::Node",
            "{",
            " ",
            f"    public:",
            f"       {camel_case(name)}()",
            f'        : Node("{node}")',
            f"        {'{ }'}",
            " ",
            " ",
            "};",
            " ",
            " ",
            "int main(int argc, char * argv[])",
            "{",
            f"    rclcpp::init(argc, argv);",
            f"    rclcpp::spin(std::make_shared<{camel_case(name)}>());",
            f"    rclcpp::shutdown();",
            " ",
            f"    return 0;",
            "}",
        ]

        for line in data:
           file.write(line + '\n')

    
    with open(f"{path}/{name}/CMakeLists.txt", "r") as f:
        data = f.readlines()

    data.insert(9, "find_package(rclcpp REQUIRED)\n")
    data.insert(10, "find_package(std_msgs REQUIRED)\n")
    data.insert(26, "include_directories(include)\n")
    
    lines = [
        " ",
        "set(dependencies",
        "\trclcpp",
        "\tstd_msgs",
        ")",
        " ",
        "## COMPILE",
        "add_library(",
        "\t${PROJECT_NAME}",
        "\tSHARED",
        f"\tsrc/{node}.cpp",
        ")",
        " ",
        "target_include_directories(",
        "\t${PROJECT_NAME}",
        "\tPRIVATE",
        "\tinclude",
        ")",
        " ",
        "ament_target_dependencies(",
        "\t${PROJECT_NAME}",
        "\tPUBLIC",
        "\t${dependencies}",
        " ",
        ")",
        " ",
        "add_executable(",
        f"\t{node}",
        f"\tsrc/{node}.cpp",
        ")",
        " ",
        f"ament_target_dependencies({node} ${'{dependencies}'})",
        " ",
        "install(",
        "\tTARGETS", 
        "\t${PROJECT_NAME}",
        "\tDESTINATION lib/${PROJECT_NAME}",
        ")",
        " ",
        "# INSTALL",
        "install(",
        "\tTARGETS ",
        f"\t{node}",
        "\tDESTINATION lib/${PROJECT_NAME}",
        ")",
        " ",
        "install(DIRECTORY",
        "launch",
        "DESTINATION share/${PROJECT_NAME}/",
        ")",
        " ",
        "install(DIRECTORY",
        "config",
        "DESTINATION share/${PROJECT_NAME}/",
        ")",
        " ",
        "ament_export_include_directories(",
        "\tinclude",
        ")",
        " ",
        "ament_export_libraries(",
        "\t${PROJECT_NAME}",
        ")",
        " ",
        "\tament_export_dependencies(",
        "\t${dependencies}",
        ")",
        " ",
        " ",
        "ament_export_include_directories(include)"
    ]
    ss = ""
    for i in lines:
        ss = ss + i + "\n"
    data.insert(27, ss)

    s = ""
    for i in data:
        s += i

    os.remove(f"{path}/{name}/CMakeLists.txt")

    with open(f"{path}/{name}/CMakeLists.txt", "w") as f:
        f.write(s)
    
    os.mkdir(f"{path}/{name}/config")
    os.mkdir(f"{path}/{name}/launch")

    with open(f"{path}/{name}/config/params.yaml", "w") as f:
        data = [{f"{node}": {'ros__parameters': {'use_sim_time': False}}}]
        yaml.dump_all(data, f)

    with open(f"{path}/{name}/launch/{name}.launch.py", "w") as f:
        
        data = [
            "import os",
            " ",
            "from ament_index_python.packages import get_package_share_directory",
            "from launch.substitutions import LaunchConfiguration",
            "from launch.actions import DeclareLaunchArgument",
            "from launch import LaunchDescription",
            "from launch_ros.actions import Node",
            " ",
            " ",
            "def generate_launch_description():",
            " ",
                f"\tpackage_name = '{name}'\n",
            " ",
            "\tparams_file = os.path.join(get_package_share_directory(package_name), 'config', 'params.yaml')",
            " ",
            f"\t{name} = Node(",
            "\t\tpackage=package_name,",
            f"\t\texecutable='{node}',",
            "\t\tparameters=[params_file]",
            " ",
            "\t)",
            " ",
            "\treturn LaunchDescription([",
            f"\t\t{name}",
            "\t])"

        ]

        for line in data:
           f.write(line + '\n')

def createMainPackage(name):
    path = f"{Path(__file__).parents[1]}/src"

    with open(f"{path}/{name}/CMakeLists.txt", "r") as f:
        data = f.readlines()


    lines = [
        "",
        "install(",
        "\tDIRECTORY config description launch worlds",
        "\tDESTINATION share/${PROJECT_NAME}",
        ")",
        ""
    ]

    ss = ""
    for i in lines:
        ss = ss + i + "\n"
    data.insert(25, ss)

    s = ""
    for i in data:
        s += i

    os.remove(f"{path}/{name}/CMakeLists.txt")
    shutil.rmtree(f"{path}/{name}/include")
    os.rmdir(f"{path}/{name}/src")

    with open(f"{path}/{name}/CMakeLists.txt", "w") as f:
        f.write(s)

    os.mkdir(f"{path}/{name}/config")
    os.mkdir(f"{path}/{name}/launch")
    os.mkdir(f"{path}/{name}/description")
    os.mkdir(f"{path}/{name}/worlds")

    xacro_robot = [
        '<?xml version="1.0"?>',
        f'<robot xmlns:xacro="http://www.ros.org/wiki/xacro"  name="{name}">\n',
        '<xacro:arg name="use_ros2_control" default="true"/>\n',
        f'<xacro:include filename="{name}_core.xacro" />\n',
        '<xacro:if value="$(arg use_ros2_control)">',
        '\t<xacro:include filename="ros2_control.xacro" />',
        '</xacro:if>\n',
        '<xacro:unless value="$(arg use_ros2_control)">',
        '\t<xacro:include filename="gazebo_control.xacro" />',
        '</xacro:unless>\n\n',
        '</robot>'

    ]

    xacro = [
        '<?xml version="1.0"?>',
        f'<robot xmlns:xacro="http://www.ros.org/wiki/xacro">',
        "",
        '<xacro:include filename="inertial_macros.xacro"/>\n',
        '<!-- BASE LINK -->\n',
        '<link name="base_link">\n',
        '</link>\n',
        '<!-- BASE_FOOTPRINT LINK -->\n',
        '<joint name="base_footprint_joint" type="fixed">',
        '\t<parent link="base_link"/>',
        '\t<child link="base_footprint"/>',
        '\t<origin xyz="0 0 0" rpy="0 0 0"/>\n',
        '</joint>\n',
        '<link name="base_footprint">\n',
        '</link>',
        '\n\n',
        '</robot>'

    ]
    ss = ""
    for i in xacro_robot:
        ss = ss + i + "\n"

    with open(f"{path}/{name}/description/{name}.urdf.xacro", "w") as f:
        f.write(ss)

    ss = ""
    for i in xacro:
        ss = ss + i + "\n"

    with open(f"{path}/{name}/description/{name}_core.xacro", "w") as f:
        f.write(ss)

    shutil.copy(f"{os.path.dirname(__file__)}/files/inertial_macros.xacro", f"{path}/{name}/description/")
    shutil.copy(f"{os.path.dirname(__file__)}/files/ros2_control.xacro", f"{path}/{name}/description/")
    

    with open(f"{path}/{name}/description/ros2_control.xacro", "r") as f:
        data = f.readlines()

    data.insert(25, f"\t\t\t<parameters>$(find {name})/config/my_controllers.yaml</parameters>")
    
    s = ""
    for i in data:
        s += i
    
    with open(f"{path}/{name}/description/ros2_control.xacro", "w") as f:
        f.write(s)

    shutil.copy(f"{os.path.dirname(__file__)}/files/gazebo_params.yaml", f"{path}/{name}/config/")
    shutil.copy(f"{os.path.dirname(__file__)}/files/my_controllers.yaml", f"{path}/{name}/config/")

    shutil.copy(f"{os.path.dirname(__file__)}/files/rsp.launch.py", f"{path}/{name}/launch/")

    with open(f"{path}/{name}/launch/rsp.launch.py", "r") as f:
        data = f.readlines()

    data.insert(19, f"    pkg_path = os.path.join(get_package_share_directory('{name}'))")
    s = ""
    for i in data:
        s += i
    
    with open(f"{path}/{name}/launch/rsp.launch.py", "w") as f:
        f.write(s)

    shutil.copy(f"{os.path.dirname(__file__)}/files/launch_sim.launch.py", f"{path}/{name}/launch/")

    with open(f"{path}/{name}/launch/launch_sim.launch.py", "r") as f:
        data = f.readlines()

    data.insert(19, f"    package_name='{name}'")
    s = ""
    for i in data:
        s += i
    
    with open(f"{path}/{name}/launch/launch_sim.launch.py", "w") as f:
        f.write(s)

    shutil.copy(f"{os.path.dirname(__file__)}/files/empty.world", f"{path}/{name}/worlds/")

def createMessagePackage(name):
    path = f"{Path(__file__).parents[1]}/src"
    shutil.rmtree(f"{path}/{name}/src")
    shutil.rmtree(f"{path}/{name}/include")
    os.mkdir(f"{path}/{name}/msg")
    os.mkdir(f"{path}/{name}/srv")

    cmake = [
        "cmake_minimum_required(VERSION 3.5)",
        f"project({name})",
        "",
        "# Default to C99",
        "if(NOT CMAKE_C_STANDARD)",
        "\tset(CMAKE_C_STANDARD 99)",
        "endif()",
        "",
        "# Default to C++14",
        "if(NOT CMAKE_CXX_STANDARD)",
        "\tset(CMAKE_CXX_STANDARD 14)",
        "endif()",
        "",
        'if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")',
        "\tadd_compile_options(-Wall -Wextra -Wpedantic)",
        "endif()",
        "",
        "# find dependencies",
        "find_package(ament_cmake REQUIRED)",
        "find_package(nav_msgs REQUIRED)",
        "# uncomment the following section in order to fill in",
        "# further dependencies manually.",
        "# find_package(<dependency> REQUIRED)",
        "",
        "if(BUILD_TESTING)",
        "\tfind_package(ament_lint_auto REQUIRED)",
        "\t# the following line skips the linter which checks for copyrights",
        "\t# uncomment the line when a copyright and license is not present in all source files",
        "\t#set(ament_cmake_copyright_FOUND TRUE)",
        "\t# the following line skips cpplint (only works in a git repo)",
        "\t# uncomment the line when this package is not in a git repo",
        "\t#set(ament_cmake_cpplint_FOUND TRUE)",
        "\tament_lint_auto_find_test_dependencies()",
        "endif()",
        "",
        "find_package(rosidl_default_generators REQUIRED)",
        "",
        "rosidl_generate_interfaces(${PROJECT_NAME}",
        '\t"msg/Placeholder.msg"',
        '\t"srv/Placeholder.srv"',
        ")",
        "",
        "",
        "ament_package()"
    ]

    ss = ""
    for i in cmake:
        ss = ss + i + "\n"

    with open(f"{path}/{name}/CMakeLists.txt", "w") as f:
        f.write(ss)

    with open(f"{path}/{name}/package.xml", "r") as f:
        data = f.readlines()

    data.insert(14, "\n")
    data.insert(15, "\t<build_depend>rosidl_default_generators</build_depend>\n")
    data.insert(16, "\t<exec_depend>rosidl_default_runtime</exec_depend>\n")
    data.insert(17, "\t<member_of_group>rosidl_interface_packages</member_of_group>\n")
    data.insert(18, "\n")

    ss = ""
    for i in data:
        ss = ss + i

    with open(f"{path}/{name}/package.xml", "w") as f:
        f.write(ss)

    msg = "int32 data"
    srv = "int32 input\n---\nint32 output"

    with open(f"{path}/{name}/msg/Placeholder.msg", 'w') as f:
        f.write(msg)

    with open(f"{path}/{name}/srv/Placeholder.srv", 'w') as f:
        f.write(srv)

    
    


if __name__ == '__main__':
    
    name = sys.argv[1]
    code = sys.argv[2]
    node = sys.argv[3]
    if code == "python":
        createNodePython(name, node)
    elif code == "c++":
        createNodeCpp(name, node)
    elif code == "main":
        createMainPackage(name)
    elif code == "messages":
        createMessagePackage(name)
