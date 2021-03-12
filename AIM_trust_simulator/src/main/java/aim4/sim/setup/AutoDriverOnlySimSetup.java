/*
Copyright (c) 2011 Tsz-Chiu Au, Peter Stone
University of Texas at Austin
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the University of Texas at Austin nor the names of its
contributors may be used to endorse or promote products derived from this
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package aim4.sim.setup;

import aim4.config.Debug;
import aim4.config.SimConfig;
import aim4.driver.pilot.V2IPilot;
import aim4.im.v2i.reservation.ReservationGridManager;
import aim4.map.GridMap;
import aim4.map.GridMapUtil;
import aim4.sim.AutoDriverOnlySimulator;
import aim4.sim.Simulator;

import java.io.IOException;

/**
 * The setup for the simulator in which all vehicles are autonomous.
 */
public class AutoDriverOnlySimSetup extends BasicSimSetup implements SimSetup {

  /////////////////////////////////
  // NESTED CLASSES
  /////////////////////////////////

  /**
   * The traffic type.
   */
  public enum TrafficType {
    UNIFORM_RANDOM,
    UNIFORM_TURNBASED,
    HVDIRECTIONAL_RANDOM,
    FILE,
  }

  /////////////////////////////////
  // PRIVATE FIELDS
  /////////////////////////////////

  /** The traffic type */
  private TrafficType trafficType = TrafficType.UNIFORM_RANDOM;
  /** The traffic level in the horizontal direction */
  private double hTrafficLevel;
  /** The traffic level in the vertical direction */
  private double vTrafficLevel;
  /** The name of the file about the traffic volume */
  private String trafficVolumeFileName = null;

  /////////////////////////////////
  // CONSTRUCTORS
  /////////////////////////////////

  /**
   * Create a setup for the simulator in which all vehicles are autonomous.
   *
   * @param basicSimSetup  the basic simulator setup
   */
  public AutoDriverOnlySimSetup(BasicSimSetup basicSimSetup) {
    super(basicSimSetup);
  }


  /////////////////////////////////
  // PUBLIC METHODS
  /////////////////////////////////

  /**
   * {@inheritDoc}
   */
  @Override
  public Simulator getSimulator(){
    double currentTime = 0.0;
    GridMap layout = new GridMap(currentTime,
                                       laneWidth,
                                       speedLimit,
                                       lanesPerRoad,
                                       medianSize,
                                       distanceBetween);
/* standard */
    // The static buffer size
//    double staticBufferSize = 0.25;
    // The time buffer for internal tiles
    double internalTileTimeBufferSize = 0.1;
    // The time buffer for edge tiles
    double edgeTileTimeBufferSize = 0.25;
    // Whether the edge time buffer is enabled
    boolean isEdgeTileTimeBufferEnabled = true;
    // The granularity of the reservation grid
    double granularity = 1.0;
    ReservationGridManager.Config gridConfig = new ReservationGridManager.Config(SimConfig.TIME_STEP,
            SimConfig.GRID_TIME_STEP,
//            staticBufferSize,
            internalTileTimeBufferSize,
            edgeTileTimeBufferSize,
            isEdgeTileTimeBufferEnabled,
            granularity);  // granularity


    Debug.SHOW_VEHICLE_COLOR_BY_MSG_STATE = true;
    GridMapUtil.setFCFSManagers(layout, currentTime, gridConfig);
//    GridMapUtil.setFCFSManagers2(layout, currentTime, gridConfig);
    GridMapUtil.setUniformRandomSpawnPoints(layout, trafficLevel);
    V2IPilot.DEFAULT_STOP_DISTANCE_BEFORE_INTERSECTION = stopDistBeforeIntersection;
    return new AutoDriverOnlySimulator(layout);
  }
}
