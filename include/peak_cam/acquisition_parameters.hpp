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
#ifndef PEAK_CAM__ACQUISITION_PARAMETERS_HPP_
#define PEAK_CAM__ACQUISITION_PARAMETERS_HPP_
#include <peak_ipl/peak_ipl.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>


namespace peak_cam
{

struct Peak_Params
{
    std::string selectedDevice{"000000"}; // default to all 0's
    int ExposureTime{100};
    std::string TriggerMode{"Off"};
    std::string TriggerSource{"Off"};
    std::string TriggerActivation{"RisingEdge"};
    int TriggerDivider{1};
    std::string Line1Source{"Off"};
    double AcquisitionFrameRate{1.0};
    int ImageHeight{480};
    int ImageWidth{640};
    double Gamma{1.2};
    std::string ExposureAuto{"Off"};
    std::string GainAuto{"Off"};
    std::string GainSelector;
    std::string PixelFormat;
    int DeviceLinkThroughputLimit{125000000};
};
}
#endif  // PEAK_CAM__ACQUISITION_PARAMETERS_HPP_
