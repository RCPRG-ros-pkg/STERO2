#!/usr/bin/env python3

# Copyright (c) 2024, Robot Control and Pattern Recognition Group,
# Institute of Control and Computation Engineering
# Warsaw University of Technology
#
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Dawid Seredynski
#

import sys
import os
from os import environ, pathsep
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, SetLaunchConfiguration,\
    GroupAction, PushLaunchConfigurations, PopLaunchConfigurations, PushEnvironment,\
    PopEnvironment, ResetEnvironment, ResetLaunchConfigurations, IncludeLaunchDescription,\
    OpaqueFunction, ExecuteProcess

from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage

from launch.conditions import IfCondition, LaunchConfigurationNotEquals

from launch.substitutions import TextSubstitution, LaunchConfiguration, PythonExpression

from importlib.machinery import SourceFileLoader

indent_size = '  '

def generic_visit(ent, indent):
    try:
        if not ent.condition is None:
            print('{}condition:'.format(indent+indent_size))
            printInfo(ent.condition, indent=indent+indent_size*2)
    except:
        pass

    try:
        for sub_ent in ent.get_sub_entities():
            printInfo(sub_ent, indent=indent+indent_size)
    except:
        pass

    try:
        for cmd in ent.cmd:
            for c in cmd:
                printInfo(c, indent=indent+indent_size*2)
    except:
        pass

def printInfo(ent, indent=None):
    if indent is None:
        indent = ''

    if isinstance(ent, LaunchDescription):
        for sub_ent in ent.entities:
            printInfo(sub_ent, indent=indent+indent_size)

    elif isinstance(ent, DeclareLaunchArgument):
        #print('{}DeclareLaunchArgument: {}, {}'.format(indent, ent.name, ent.description))
        print('{}DeclareLaunchArgument: {}'.format(indent, ent.name))

    elif isinstance(ent, SetLaunchConfiguration):
        print('{}SetLaunchConfiguration:'.format(indent))
        for sub_ent, sub_ent_val in zip(ent.name, ent.value):
            print('{}name:'.format(indent+indent_size))
            printInfo(sub_ent, indent=indent+indent_size*2)
            print('{}value:'.format(indent+indent_size))
            printInfo(sub_ent_val, indent=indent+indent_size*2)

    elif isinstance(ent, TextSubstitution):
        print('{}TextSubstitution: {}'.format(indent, ent.text))

    elif isinstance(ent, GroupAction):
        print('{}GroupAction:'.format(indent))

    elif isinstance(ent, PushLaunchConfigurations):
        # Action that pushes the current state of launch configurations to a stack
        print('{}PushLaunchConfigurations'.format(indent))

    elif isinstance(ent, PopLaunchConfigurations):
        print('{}PopLaunchConfigurations'.format(indent))

    elif isinstance(ent, PushEnvironment):
        print('{}PushEnvironment'.format(indent))

    elif isinstance(ent, PopEnvironment):
        print('{}PopEnvironment'.format(indent))

    elif isinstance(ent, ResetEnvironment):
        print('{}ResetEnvironment'.format(indent))

    elif isinstance(ent, ResetLaunchConfigurations):
        print('{}ResetLaunchConfigurations'.format(indent))

    elif isinstance(ent, SetEnvironmentVariable):
        print('{}SetEnvironmentVariable:'.format(indent))
        for sub_ent, sub_ent_val in zip(ent.name, ent.value):
            print('{}name:'.format(indent+indent_size))
            printInfo(sub_ent, indent=indent+indent_size*2)
            print('{}value:'.format(indent+indent_size))
            printInfo(sub_ent_val, indent=indent+indent_size*2)

    elif isinstance(ent, IncludeLaunchDescription):
        print('{}IncludeLaunchDescription:'.format(indent))

    elif isinstance(ent, Node):
        print('{}Node: {} {}'.format(indent, ent.node_package, ent.node_executable))

    elif isinstance(ent, ExecuteProcess):
        print('{}ExecuteProcess:'.format(indent))

    elif isinstance(ent, IfCondition):
        print('{}IfCondition'.format(indent))

    elif isinstance(ent, LaunchConfigurationNotEquals):
        print('{}LaunchConfigurationNotEquals'.format(indent))

    elif isinstance(ent, OpaqueFunction):
        print('{}OpaqueFunction:'.format(indent))

    elif isinstance(ent, ExecutableInPackage):
        print('{}ExecutableInPackage:'.format(indent))
        for sub_ent, sub_ent_val in zip(ent.package, ent.executable):
            print('{}package:'.format(indent+indent_size))
            printInfo(sub_ent, indent=indent+indent_size*2)
            print('{}executable:'.format(indent+indent_size))
            printInfo(sub_ent_val, indent=indent+indent_size*2)

    elif isinstance(ent, LaunchConfiguration):
        print('{}LaunchConfiguration:'.format(indent))
        for sub_ent in ent.variable_name:
            printInfo(sub_ent, indent=indent+indent_size)

    elif isinstance(ent, PythonExpression):
        print('{}PythonExpression:'.format(indent))
        for sub_ent in ent.expression:
            printInfo(sub_ent, indent=indent+indent_size)

    else:
        print('{}not supported: {}'.format(indent, ent))
        print('{}{}'.format(indent, dir(ent)))

    generic_visit(ent, indent)


def main(launchfile_name):

    launchfile = SourceFileLoader("launchfile", launchfile_name).load_module()

    ld = launchfile.generate_launch_description()

    printInfo(ld)

    return 0

if __name__ == "__main__":
    exit(main(sys.argv[1]))
