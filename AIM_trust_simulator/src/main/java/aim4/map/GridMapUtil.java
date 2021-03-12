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
package aim4.map;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import aim4.config.SimConfig;
import aim4.im.RoadBasedIntersection;
import aim4.im.RoadBasedTrackModel;

import aim4.im.v2i.V2IManager;
import aim4.im.v2i.RequestHandler.RequestHandler;
import aim4.im.v2i.policy.Policy;
import aim4.im.v2i.reservation.ReservationGridManager;
import aim4.map.SpawnPoint.SpawnSpec;
import aim4.map.SpawnPoint.SpawnSpecGenerator;
import aim4.util.Util;
import aim4.vehicle.VehicleSpec;
import aim4.vehicle.VehicleSpecDatabase;

/**
 * The utility class for GridMap.
 */
public class GridMapUtil {

  /////////////////////////////////
  // NESTED CLASSES
  /////////////////////////////////

  /**
   * The uniform distributed spawn spec generator.
   */
  public static class UniformSpawnSpecGenerator implements SpawnSpecGenerator {
    /** The proportion of each spec */
    private List<Double> proportion;
    /** The destination selector */
    private DestinationSelector destinationSelector;
    /** probability of generating a vehicle in each spawn time step */
    private double prob;

    /**
     * Create an uniform spawn specification generator.
     *
     * @param trafficLevel         the traffic level
     * @param destinationSelector  the destination selector
     */
    public UniformSpawnSpecGenerator(double trafficLevel,
                                     DestinationSelector destinationSelector) {
      int n = VehicleSpecDatabase.getNumOfSpec();
      proportion = new ArrayList<Double>(n);
      double p = 1.0 / n;
      for(int i=0; i<n; i++) {
        proportion.add(p);
      }
      this.destinationSelector = destinationSelector;

      prob = trafficLevel * SimConfig.SPAWN_TIME_STEP;
      // Cannot generate more than one vehicle in each spawn time step
      assert prob <= 1.0;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public List<SpawnSpec> act(SpawnPoint spawnPoint, double timeStep) {
      List<SpawnSpec> result = new LinkedList<>();
      double initTime = spawnPoint.getCurrentTime();
      for(double time = initTime; time < initTime + timeStep; time += SimConfig.SPAWN_TIME_STEP) {
        if (Util.random.nextDouble() < prob) {
//          int i = Util.randomIndex(proportion);
          VehicleSpec vehicleSpec = VehicleSpecDatabase.getVehicleSpecById(0);
          Road fakeDestinationRoad = destinationSelector.selectDestination(spawnPoint.getLane());
          Road realDestinationRoad = destinationSelector.selectDestination(spawnPoint.getLane());
          while(fakeDestinationRoad == realDestinationRoad) {
            realDestinationRoad = destinationSelector.selectDestination(spawnPoint.getLane());
          }
          result.add(new SpawnSpec(spawnPoint.getCurrentTime(), vehicleSpec, fakeDestinationRoad, realDestinationRoad));
        }
      }

      return result;
    }
  }


  /////////////////////////////////
  // PUBLIC STATIC METHODS
  /////////////////////////////////


  /**
   * Set the FCFS managers at all intersections.
   *
   * @param layout       the map
   * @param currentTime  the current time
   * @param config       the reservation grid manager configuration
   */
  public static void setFCFSManagers(GridMap layout, double currentTime,
                                     ReservationGridManager.Config config) {
    layout.removeAllManagers();
    for(int column = 0; column < layout.getColumns(); column++) {
      for(int row = 0; row < layout.getRows(); row++) {
        List<Road> roads = layout.getRoads(column, row);
        RoadBasedIntersection intersection = new RoadBasedIntersection(roads);
        RoadBasedTrackModel trajectoryModel = new RoadBasedTrackModel(intersection);
        V2IManager im = new V2IManager(intersection, trajectoryModel, currentTime, config, layout.getImRegistry());
        im.setPolicy(new Policy(im, new RequestHandler()));
        layout.setManager(column, row, im);
      }
    }
  }

  /**
   * Set the uniform random spawn points.
   *
   * @param map           the map
   * @param trafficLevel  the traffic level
   */
  public static void setUniformRandomSpawnPoints(GridMap map, double trafficLevel) {
    for(SpawnPoint sp : map.getSpawnPoints()) {
      sp.setVehicleSpecChooser(new UniformSpawnSpecGenerator(trafficLevel, new DestinationSelector(map)));
    }
  }

}
