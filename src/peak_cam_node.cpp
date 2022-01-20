// Copyright (c) 2020, Sherif Nekkah 
// All rights reserved. 
// 
// DISCLAMER:
//
//
// This package was created and used within an academic project and should
// be considered as experimental code. There may be bugs and deficiencies in the
// software. Feel free for suggestions, pull requests or any possible issue. 
//
//
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met: 
// 
//  * Redistributions of source code must retain the above copyright notice, 
//    this list of conditions and the following disclaimer. 
//  * Redistributions in binary form must reproduce the above copyright 
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the distribution. 
//  * Neither the name of  nor the names of its contributors may be used to 
//    endorse or promote products derived from this software without specific 
//    prior written permission. 
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE. 



#include <iostream>
#include <ros/ros.h>
#include "peak_cam.hpp"
#include <thread> 
#include <csignal>


std::atomic<bool>* acquisitionLoop_running_ptr;

void setFinish(int s)
{
    *acquisitionLoop_running_ptr = false;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "peak_cam");
    ros::NodeHandle nodeHandle = ros::NodeHandle("~");

    // Spawn peak node object
    peak_cam::Peak_Cam peak_Cam(nodeHandle);

    // Pointer on boolean to interrupt acquisition loop later on
    acquisitionLoop_running_ptr = &peak_Cam.acquisitionLoop_running;

    if (!acquisitionLoop_running_ptr->is_lock_free()) 
        return 10; // error

    // refers "SIGINT" = ctrl-C in consol to "setFinish" function for controlled shutdown of acquisition Loop
    signal(SIGINT, setFinish);
  
    // Seperating acquisition Loop on particular thread
    std::thread acquisitionThread(&peak_cam::Peak_Cam::acquisitionLoop, &peak_Cam);

    while (ros::ok())
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();

        if (!*acquisitionLoop_running_ptr)
            return EXIT_SUCCESS;
    }

    acquisitionThread.join();

    return EXIT_SUCCESS;
}
