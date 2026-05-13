/*
* Copyright (C) 2024 Texas Instruments Incorporated
*
* All rights reserved not granted herein.
* Limited License.  
*
* Texas Instruments Incorporated grants a world-wide, royalty-free, 
* non-exclusive license under copyrights and patents it now or hereafter 
* owns or controls to make, have made, use, import, offer to sell and sell ("Utilize")
* this software subject to the terms herein.  With respect to the foregoing patent 
* license, such license is granted  solely to the extent that any such patent is necessary 
* to Utilize the software alone.  The patent license shall not apply to any combinations which 
* include this software, other than combinations with devices manufactured by or for TI ("TI Devices").  
* No hardware patent is licensed hereunder.
*
* Redistributions must preserve existing copyright notices and reproduce this license (including the 
* above copyright notice and the disclaimer and (if applicable) source code license limitations below) 
* in the documentation and/or other materials provided with the distribution
*
* Redistribution and use in binary form, without modification, are permitted provided that the following
* conditions are met:
*
*	* No reverse engineering, decompilation, or disassembly of this software is permitted with respect to any 
*     software provided in binary form.
*	* any redistribution and use are licensed by TI for use only with TI Devices.
*	* Nothing shall obligate TI to provide you with source code for the software licensed and provided to you in object code.
*
* If software source code is provided to you, modification and redistribution of the source code are permitted 
* provided that the following conditions are met:
*
*   * any redistribution and use of the source code, including any resulting derivative works, are licensed by 
*     TI for use only with TI Devices.
*   * any redistribution and use of any object code compiled from the source code and any resulting derivative 
*     works, are licensed by TI for use only with TI Devices.
*
* Neither the name of Texas Instruments Incorporated nor the names of its suppliers may be used to endorse or 
* promote products derived from this software without specific prior written permission.
*
* DISCLAIMER.
*
* THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
* BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
* IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
* OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE.
*/

#include "boundaryArc.h"


/* Template for the Zone Occupancy Function.
 * Requires :
 *
 * Input_point must be a float[2] of x, then y coordinates.
 * zone_boundaries_ptr must be a float[6], of xMin, xMax, yMin, yMax, zMin, Zmax coordinates
 *
 * Modifies :
 * None
 *
 * Effects : Returns whether the point is within the boundaries specified
 *   Computes whether an object is inside a rectangular box.
 *   NOTE - only calculates for x/y positions. z coordinates do not get taken into account
 *
 **/
uint8_t zoneOccupancyFxnArc(float *input_point, void *zone_boundaries_ptr)
{
    float x = input_point[0];
    float y = input_point[1];

    float r_meters      = sqrt(pow(x, 2) + pow(y, 2));
    float theta_degrees = 180.0 / M_PI * atan(x / y);

    float rMin     = ((float *)zone_boundaries_ptr)[0];
    float rMax     = ((float *)zone_boundaries_ptr)[1];
    float thetaMin = ((float *)zone_boundaries_ptr)[2];
    float thetaMax = ((float *)zone_boundaries_ptr)[3];
    /* z coordinates not used */
    //float zMin     = ((float *)zone_boundaries_ptr)[4];
    //float zMax     = ((float *)zone_boundaries_ptr)[5];

    return (r_meters > rMin && r_meters <= rMax && theta_degrees > thetaMin && theta_degrees <= thetaMax);
}
