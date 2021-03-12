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

import aim4.sim.Simulator;

import java.io.IOException;

/**
 * The basic simulator setup
 */
public class BasicSimSetup implements SimSetup {

  /** The width of lanes */
  protected double laneWidth;
  /** The speed limit of the roads */
  protected double speedLimit;
  /** The number of lanes per road */
  protected int lanesPerRoad;
  /** The width of the area between the opposite directions of a road */
  protected double medianSize;
  /** The distance between intersection */
  protected double distanceBetween;
  /** The traffic level */
  protected double trafficLevel;
  /** The stopping distance before intersection */
  protected double stopDistBeforeIntersection;

  /**
   * Create a copy of a given basic simulator setup.
   *
   * @param basicSimSetup  a basic simulator setup
   */
  public BasicSimSetup(BasicSimSetup basicSimSetup) {

    this.laneWidth = basicSimSetup.laneWidth;
    this.speedLimit = basicSimSetup.speedLimit;
    this.lanesPerRoad = basicSimSetup.lanesPerRoad;
    this.medianSize = basicSimSetup.medianSize;
    this.distanceBetween = basicSimSetup.distanceBetween;
    this.trafficLevel = basicSimSetup.trafficLevel;
    this.stopDistBeforeIntersection = basicSimSetup.stopDistBeforeIntersection;
  }

  /**
   * Create a basic simulator setup.

   * @param laneWidth                   the width of lanes
   * @param speedLimit                  the speed limit of the roads
   * @param lanesPerRoad                the number of lanes per road
   * @param medianSize                  the width of the area between the
   *                                    opposite directions of a road
   * @param distanceBetween             the distance between intersections
   * @param trafficLevel                the traffic level
   * @param stopDistBeforeIntersection  the stopping distance before
   *                                    intersection
   */
  public BasicSimSetup(
                       double laneWidth, double speedLimit,
                       int lanesPerRoad,
                       double medianSize, double distanceBetween,
                       double trafficLevel,
                       double stopDistBeforeIntersection) {

    this.laneWidth = laneWidth;
    this.speedLimit = speedLimit;
    this.lanesPerRoad = lanesPerRoad;
    this.medianSize = medianSize;
    this.distanceBetween = distanceBetween;
    this.trafficLevel = trafficLevel;
    this.stopDistBeforeIntersection = stopDistBeforeIntersection;
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public Simulator getSimulator() {
    // TODO: think how to avoid using the following assertation.
    assert false : ("Cannot instantiate BasicSimSetup");
    return null;
  }


  /**
   * Get the speed limit of the roads.
   *
   * @return the speed limit of the roads.
   */
  public double getSpeedLimit() {
    return speedLimit;
  }

  /**
   * Get the number of lanes per road.
   *
   * @return the number of lanes per road
   */
  public int getLanesPerRoad() {
    return lanesPerRoad;
  }

  /**
   * Get the traffic level.
   *
   * @return the traffic level
   */
  public double getTrafficLevel() {
    return trafficLevel;
  }

  /**
   * Get the stopping distance before intersection.
   *
   * @return the stopping distance before intersection
   */
  public double getStopDistBeforeIntersection() {
    return stopDistBeforeIntersection;
  }


  /**
   * {@inheritDoc}
   */
  @Override
  public void setTrafficLevel(double trafficLevel) {
    this.trafficLevel = trafficLevel;
  }

  /**
   * Set the speed limit.
   *
   * @param speedLimit  the speed limit
   */
  public void setSpeedLimit(double speedLimit) {
    this.speedLimit = speedLimit;
  }

  // TODO: maybe move to AutoDriverOnlySimSetup

  /**
   * {@inheritDoc}
   */
  @Override
  public void setStopDistBeforeIntersection(double stopDistBeforeIntersection) {
    this.stopDistBeforeIntersection = stopDistBeforeIntersection;
  }



  /**
   * Set the number of lanes per road.
   *
   * @param lanesPerRoad  the number of lanes per road
   */
  public void setLanesPerRoad(int lanesPerRoad) {
    this.lanesPerRoad = lanesPerRoad;
  }



}
