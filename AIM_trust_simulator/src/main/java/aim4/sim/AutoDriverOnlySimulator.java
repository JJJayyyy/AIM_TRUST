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
package aim4.sim;

import aim4.config.Debug;
import aim4.config.DebugPoint;
import aim4.config.Constants;
import aim4.driver.AutoDriver;
import aim4.driver.DriverSimView;
import aim4.im.IntersectionManager;
import aim4.im.v2i.V2IManager;
import aim4.map.BasicMap;
import aim4.map.DataCollectionLine;
import aim4.map.Road;
import aim4.map.SpawnPoint;
import aim4.map.SpawnPoint.SpawnSpec;
import aim4.map.lane.Lane;
import aim4.msg.i2v.I2VMessage;
import aim4.msg.v2i.V2IMessage;
import aim4.util.Util;
import aim4.vehicle.*;

import java.awt.*;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.*;
import java.text.DecimalFormat;
import java.util.List;
import java.util.Queue;
import java.util.*;

/**
 * The autonomous drivers only simulator.
 */
public class AutoDriverOnlySimulator implements Simulator {

  /////////////////////////////////
  // NESTED CLASSES
  /////////////////////////////////

  /** The result of a simulation step.*/
  public static class AutoDriverOnlySimStepResult implements SimStepResult {
    List<Integer> completedVINs;  // The VIN of the completed vehicles in this time step
    public AutoDriverOnlySimStepResult(List<Integer> completedVINs) { // Create a result of a simulation step
      this.completedVINs = completedVINs;
    }
    public List<Integer> getCompletedVINs() {  // Get the list of VINs of completed vehicles.
      return completedVINs;
    }
  }

  /////////////////////////////////
  // PRIVATE FIELDS
  /////////////////////////////////
  /** The map */
  private BasicMap basicMap;
  /** All active vehicles, in form of a map from VINs to vehicle objects. */
  public Map<Integer,VehicleSimView> vinToVehicles;
  public ArrayList<Integer> vehicleList;
  public Map<Integer, ArrayList<Double>> vehiclesTrustList;

  public Map<Integer, Integer> collidedCarInEpisode;
  public Map<Integer, Integer> completeCarInStep;
  public Map<Integer, Integer> addCollisionCarAtEndOfEpisode;

  public int scenario_count = 0;
  public int total_scenario = 10;

  public Map<Integer, Double> vehicleTrustValue;
  public Map<Integer, Double>vehiclesBufferList;
  public ArrayList<Double> belief;
  public ArrayList<Double> disbelief;
  public ArrayList<Double> suspend;
  public Boolean canSpawn = true;
  public Map<Integer, Integer> collisionVINs;

  /** The current time */
  private double currentTime;
  /** The number of completed vehicles */
  private int numOfCompletedVehicles;
  private int numOfCollisionVehicles;
  /** The total number of bits transmitted by the completed vehicles */
  private int totalBitsTransmittedByCompletedVehicles;
  /** The total number of bits received by the completed vehicles */
  private int totalBitsReceivedByCompletedVehicles;
  private int RL_episode_flag = 0;

  public Map<Integer, String> destinationMap;
  public Map<Integer, String> spawnRoadMap;
  public Map<Integer, Integer> collisionMap;
  public Map<Integer, String> velocityMap;
  public double startTime;



  /////////////////////////////////
  // CLASS CONSTRUCTORS
  /////////////////////////////////
  /**
   * Create an instance of the simulator.
   *
   * @param basicMap             the map of the simulation
   */
  public AutoDriverOnlySimulator(BasicMap basicMap) {
    this.basicMap = basicMap;
    this.vinToVehicles = new HashMap<>();
    this.vehiclesBufferList = new HashMap<>();
    this.vehiclesTrustList = new HashMap<>();
    this.vehicleTrustValue = new HashMap<>();
    this.vehicleList = new ArrayList<>(Constants.car_id_max);
    this.collidedCarInEpisode = new HashMap<>();
    this.completeCarInStep = new HashMap<>();
    this.addCollisionCarAtEndOfEpisode = new HashMap<>();
    this.destinationMap = new HashMap<>();
    this.spawnRoadMap = new HashMap<>();
    this.collisionMap = new HashMap<>();
    this.collisionVINs = new HashMap<>();
    this.velocityMap = new HashMap<>();

    for (int i=0; i<Constants.car_id_max;i++){
      vehicleList.add(1000+i);
    }

    currentTime = 0.0;
    numOfCompletedVehicles = 0;
    numOfCollisionVehicles = 0;
    totalBitsTransmittedByCompletedVehicles = 0;
    totalBitsReceivedByCompletedVehicles = 0;

    this.belief = new ArrayList<>();
    belief.add(1.0);
    belief.add(0.0);
    belief.add(0.0);
    belief.add(0.5);
    this.disbelief = new ArrayList<>();
    disbelief.add(0.0);
    disbelief.add(1.0);
    disbelief.add(0.0);
    disbelief.add(0.5);

    this.suspend = new ArrayList<>();
    suspend.add(0.5);
    suspend.add(0.0);
    suspend.add(0.0);
    suspend.add(0.5);

    Util.resetrandom();
    this.startTime=0;
  }

  /////////////////////////////////
  // PUBLIC METHODS
  /////////////////////////////////
  /** {@inheritDoc}*/
  @Override
  public synchronized AutoDriverOnlySimStepResult step(double timeStep) throws IOException, InterruptedException {
    Map<Integer, Integer> collisionlist = detectCollision();
    data_update();
    whetherCanSpawn();
    spawnVehicles(timeStep);
    RL_episode_flag = 0;
    setRLInforToVehicle();
    provideSensorInput();
    communication();
    letIntersectionManagersAct(timeStep);
    letDriversAct();
    moveVehicles(timeStep);
    List<Integer> completedVINs = cleanUpCompletedVehicles(collisionlist);
    currentTime += timeStep;
    return new AutoDriverOnlySimStepResult(completedVINs);
  }


  /////////////////////////////////
  // PUBLIC METHODS
  /////////////////////////////////

  // information retrieval

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized BasicMap getMap() {
    return basicMap;
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized double getSimulationTime() {
    return currentTime;
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized int getNumCompletedVehicles() {
    return numOfCompletedVehicles;
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized double getAvgBitsTransmittedByCompletedVehicles() {
    if (numOfCompletedVehicles > 0) {
      return ((double)totalBitsTransmittedByCompletedVehicles)
             / numOfCompletedVehicles;
    } else {
      return 0.0;
    }
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized double getAvgBitsReceivedByCompletedVehicles() {
    if (numOfCompletedVehicles > 0) {
      return ((double)totalBitsReceivedByCompletedVehicles)/ numOfCompletedVehicles;
    } else {
      return 0.0;
    }
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public synchronized Set<VehicleSimView> getActiveVehicles() {
    return new HashSet<>(vinToVehicles.values());
  }


  /////////////////////////////////
  // PRIVATE METHODS
  /////////////////////////////////


  /**
   * Spawn vehicles.
   *
   * @param timeStep  the time step
   */
  private void spawnVehicles(double timeStep) {
      for(SpawnPoint spawnPoint : basicMap.getSpawnPoints()) {
        List<SpawnSpec> spawnSpecs = spawnPoint.act(timeStep);
        if (!spawnSpecs.isEmpty()) {
          if (canSpawnVehicle(spawnPoint)) {
            for(SpawnSpec spawnSpec : spawnSpecs) {
              VehicleSimView vehicle = makeVehicle(spawnPoint, spawnSpec);
              VinRegistry.registerVehicle(vehicle, collidedCarInEpisode, completeCarInStep); // Get vehicle a VIN number

//              if(vehicle.getVIN() == 1003 || vehicle.getVIN() == 1008){ // 2 untrusted
//                if(vehicle.getVIN() == 1001 || vehicle.getVIN() == 1004 || vehicle.getVIN() == 1005
//              || vehicle.getVIN() == 1009 ){ // 4 untrusted
//              if((vehicle.getVIN()-1000)% 2 == 0 || vehicle.getVIN() == 1007){ // 6 untrusted  50^100
              if((vehicle.getVIN()-1000)% 2 == 1 || vehicle.getVIN() == 1008 ||
                      vehicle.getVIN() == 1000|| vehicle.getVIN() == 1006){ // 8 untrusted
//              if((vehicle.getVIN()-1000)% 2 == 1 ||( vehicle.getVIN()-1000)% 2 == 0){ // 10 untrusted
                final int i = Util.random.nextInt(100);
//                System.out.println(vehicle.getVIN());
                if (i>20){
                  vehicle.getDriver().setRealDestination(spawnSpec.getRealDestinationRoad());
                  vehicle.getDriver().setReliable(false);
                }
              }
              vinToVehicles.put(vehicle.getVIN(), vehicle);
              destinationMap.put(vehicle.getVIN(), vehicle.getDriver().getDestination().getName());
              spawnRoadMap.put(vehicle.getVIN(), basicMap.getRoad(vehicle.getDriver().getCurrentLane()).getName());

              if(!vehiclesTrustList.containsKey(vehicle.getVIN())){
                if(!vehicle.getDriver().getReliable()){
                  vehiclesTrustList.put(vehicle.getVIN(), suspend);
                  vehicleTrustValue.put(vehicle.getVIN(), suspend.get(0)+suspend.get(2)*suspend.get(3));
                }else {
                  vehiclesTrustList.put(vehicle.getVIN(), belief);
                  vehicleTrustValue.put(vehicle.getVIN(), belief.get(0)+belief.get(2)*belief.get(3));
                }
              }
//              System.out.println(vehicle.getVIN()+" real:"+vehicle.getDriver().getRealDestination()+", fake:"+vehicle.getDriver().getDestination());
              break; // only handle the first spawn vehicle
            }
          } // else ignore the spawnSpecs and do nothing
        }
      }
  }

 private void whetherCanSpawn() throws IOException, InterruptedException {
      if (vinToVehicles.size()==(Constants.car_id_max-collidedCarInEpisode.size()- completeCarInStep.size())){
      if (canSpawn){
        System.out.println("start");
        outputInfToRL();
      }
     canSpawn = false;
    }

   if (vinToVehicles.size()==0){
     if (!canSpawn){
       if (scenario_count%total_scenario==0&&scenario_count!=0){
         RL_episode_flag = 1;
       }
       outputInfToRL();
       for(int vin: collidedCarInEpisode.keySet()){
         addCollisionCarAtEndOfEpisode.put(vin, 1);
       }
       System.out.println("scenario_count: " + scenario_count);
       scenario_count++;
       if (scenario_count%total_scenario==0&&scenario_count!=0){
         System.out.println("collision number: " + numOfCollisionVehicles);
         File writename = new File("collision.txt");
         BufferedWriter out = new BufferedWriter(new FileWriter(writename, true));
         out.write(scenario_count+","+numOfCollisionVehicles+"\n");
         out.flush();

         numOfCollisionVehicles =  0;
         System.out.println("reset");
         double endTime= currentTime;
         System.out.println("running time ： "+(endTime-startTime)+"ms");
         startTime = currentTime;
         addCollisionCarAtEndOfEpisode = new HashMap<>();
         vehiclesTrustList = new HashMap<>();
         collidedCarInEpisode = new HashMap<>();
         velocityMap = new HashMap<>();
         collisionMap = new HashMap<>();
         destinationMap = new HashMap<>();
         spawnRoadMap = new HashMap<>();
         Util.resetrandom();
       }
       completeCarInStep = new HashMap<>();
       System.out.println("End\n");
       Thread.sleep(100);
     }
     canSpawn = true;
   }
 }


 private void data_update(){
   DecimalFormat fnum = new DecimalFormat("##0.00");
   for (Integer i : vehicleList){
     if(vinToVehicles.get(i)!=null){
       VehicleSimView vehicle = vinToVehicles.get(i);
       String velocity = fnum.format(vehicle.getVelocity());
       velocityMap.put(i, velocity);
       collisionMap.put(i, collisionVINs.get(i));
     }
   }
 }
  /**
   * Whether a spawn point can spawn any vehicle
   *
   * @param spawnPoint  the spawn point
   * @return Whether the spawn point can spawn any vehicle
   */
  private boolean canSpawnVehicle(SpawnPoint spawnPoint) {
    if (vinToVehicles.size()<(Constants.car_id_max-collidedCarInEpisode.size()- completeCarInStep.size()) && canSpawn){
      Rectangle2D noVehicleZone = spawnPoint.getNoVehicleZone();
      for(VehicleSimView vehicle : vinToVehicles.values()) {
        if (vehicle.getShape().intersects(noVehicleZone)) {
          return false;
        }
      }
      return true;
    }else {
      return false;
    }
  }

  /**
   * Create a vehicle at a spawn point.
   *
   * @param spawnPoint  the spawn point
   * @param spawnSpec   the spawn specification
   * @return the vehicle
   */
  private VehicleSimView makeVehicle(SpawnPoint spawnPoint, SpawnSpec spawnSpec) {
    VehicleSpec spec = spawnSpec.getVehicleSpec();
    Lane lane = spawnPoint.getLane();
    // Now just take the minimum of the max velocity of the vehicle, and
    // the speed limit in the lane
    double initVelocity = Math.min(spec.getMaxVelocity(), lane.getSpeedLimit());
    // Obtain a Vehicle
    AutoVehicleSimView vehicle = new BasicAutoVehicle(spec,
                           spawnPoint.getPosition(),
                           spawnPoint.getHeading(),
                           spawnPoint.getSteeringAngle(),
                           initVelocity, // velocity
                           initVelocity,  // target velocity
                           spawnPoint.getAcceleration(),
                           spawnSpec.getSpawnTime());
    // Set the driver
    AutoDriver driver = new AutoDriver(vehicle, basicMap);
    driver.setCurrentLane(lane);
    driver.setSpawnPoint(spawnPoint);
    driver.setDestination(spawnSpec.getDestinationRoad());
    driver.setRealDestination(spawnSpec.getDestinationRoad());
    driver.setReliable(true);
    driver.setBuffersize(1);
    driver.setWaitcycle(0);
    vehicle.setDriver(driver);
    return vehicle;
  }



  /////////////////////////////////
  // STEP 2
  /////////////////////////////////





  /**
   * Provide each vehicle with sensor information to allow it to make
   * decisions.  This works first by making an ordered list for each Lane of
   * all the vehicles in that Lane, in order from the start of the Lane to
   * the end of the Lane.  We must make sure to leave out all vehicles that
   * are in the intersection.  We must also concatenate the lists for lanes
   * that feed into one another.  Then, for each vehicle, depending on the
   * state of its sensors, we provide it with the appropriate sensor input.
   */
  private void provideSensorInput() {
    Map<Lane,SortedMap<Double,VehicleSimView>> vehicleLists = computeVehicleLists();
    Map<VehicleSimView, VehicleSimView> nextVehicle = computeNextVehicle(vehicleLists);
    provideIntervalInfo(nextVehicle);
    provideVehicleTrackingInfo(vehicleLists);

  }

  /**
   * Compute the lists of vehicles of all lanes.
   *
   * @return a mapping from lanes to lists of vehicles sorted by their
   *         distance on their lanes
   */
  private Map<Lane,SortedMap<Double,VehicleSimView>> computeVehicleLists() {
    // Set up the structure that will hold all the Vehicles as they are currently ordered in the Lanes
    Map<Lane,SortedMap<Double,VehicleSimView>> vehicleLists = new HashMap<>();
    for(Road road : basicMap.getRoads()) {
      for(Lane lane : road.getLanes()) {
        vehicleLists.put(lane, new TreeMap<>());
      }
    }
    // Now add each of the Vehicles, but make sure to exclude those that are
    // already inside (partially or entirely) the intersection
    for(VehicleSimView vehicle : vinToVehicles.values()) {// Find out what lanes it is in.
      Set<Lane> lanes = vehicle.getDriver().getCurrentlyOccupiedLanes();
      for(Lane lane : lanes) {// Find out what IntersectionManager is coming up for this vehicle
        IntersectionManager im = lane.getLaneIM().nextIntersectionManager(vehicle.getPosition());
        // Only include this Vehicle if it is not in the intersection.
        if(lane.getLaneIM().distanceToNextIntersection(vehicle.getPosition())>0
                || im == null || !im.intersects(vehicle.getShape().getBounds2D())) {
          // Now find how far along the lane it is.
          double dst = lane.distanceAlongLane(vehicle.getPosition());
          // Now add it to the map.
          vehicleLists.get(lane).put(dst, vehicle);
        }
      }
    }
    // Now consolidate the lists based on lanes
    for(Road road : basicMap.getRoads()) {
      for(Lane lane : road.getLanes()) {
        // We may have already removed this Lane from the map
        if(vehicleLists.containsKey(lane)) {
          Lane currLane = lane;
          // Now run through the lanes
          while(currLane.hasNextLane()) {
            currLane = currLane.getNextLane();
            // Put everything from the next lane into the original lane
            // and remove the mapping for the next lane
            vehicleLists.get(lane).putAll(vehicleLists.remove(currLane));
          }
        }
      }
    }

    return vehicleLists;
  }

  /**
   * Compute the next vehicles of all vehicles.
   *
   * @param vehicleLists  a mapping from lanes to lists of vehicles sorted by
   *                      their distance on their lanes
   * @return a mapping from vehicles to next vehicles
   */
  private Map<VehicleSimView, VehicleSimView> computeNextVehicle(
          Map<Lane,SortedMap<Double,VehicleSimView>> vehicleLists) {
    // At this point we should only have mappings for start Lanes, and they
    // should include all the Lanes they run into.  Now we need to turn this
    // into a hash map that maps Vehicles to the next vehicle in the Lane
    // or any Lane the Lane runs into
    Map<VehicleSimView, VehicleSimView> nextVehicle = new HashMap<>();
    // For each of the ordered lists of vehicles
    for(SortedMap<Double,VehicleSimView> vehicleList : vehicleLists.values()) {
      VehicleSimView lastVehicle = null;
      // Go through the Vehicles in order of their position in the Lane
      for(VehicleSimView currVehicle : vehicleList.values()) {
        if(lastVehicle != null) {
          // Create the mapping from the previous Vehicle to the current one
          nextVehicle.put(lastVehicle,currVehicle);
        }
        lastVehicle = currVehicle;
      }
    }
    return nextVehicle;
  }

  /**
   * Provide sensing information to the intervalometers of all vehicles.
   *
   * @param nextVehicle  a mapping from vehicles to next vehicles
   */
  private void provideIntervalInfo(Map<VehicleSimView, VehicleSimView> nextVehicle) {

    // Now that we have this list set up, let's provide input to all the
    // Vehicles.
    for(VehicleSimView vehicle: vinToVehicles.values()) {
      // If the vehicle is autonomous
      if (vehicle instanceof AutoVehicleSimView) {
        AutoVehicleSimView autoVehicle = (AutoVehicleSimView)vehicle;

        switch(autoVehicle.getLRFMode()) {
        case DISABLED:
          // Find the interval to the next vehicle
          double interval;
          // If there is a next vehicle, then calculate it
          if(nextVehicle.containsKey(autoVehicle)) {
            // It's the distance from the front of this Vehicle to the point
            // at the rear of the Vehicle in front of it
            interval = calcInterval(autoVehicle, nextVehicle.get(autoVehicle));
          } else { // Otherwise, just set it to the maximum possible value
            interval = Double.MAX_VALUE;
          }
          // Now actually record it in the vehicle
          autoVehicle.getIntervalometer().record(interval);
          autoVehicle.setLRFSensing(false); // Vehicle is not using
                                            // the LRF sensor
          break;
        case LIMITED:
          case ENABLED:
          autoVehicle.setLRFSensing(true); // Vehicle is using the LRF sensor
          break;
          default:
          throw new RuntimeException("Unknown LRF Mode: " + autoVehicle.getLRFMode().toString());
        }
      }
    }
  }

  /**
   * Calculate the distance between vehicle and the next vehicle on a lane.
   *
   * @param vehicle      the vehicle
   * @param nextVehicle  the next vehicle
   * @return the distance between vehicle and the next vehicle on a lane
   */
  private double calcInterval(VehicleSimView vehicle, VehicleSimView nextVehicle) {
    Point2D pos = vehicle.getPosition();
    if(nextVehicle.getShape().contains(pos)) {
      return 0.0;
    } else {
      double interval = Double.MAX_VALUE ;
      for(Line2D edge : nextVehicle.getEdges()) {
        double dst = edge.ptSegDist(pos);
        if(dst < interval){
          interval = dst;
        }
      }
      return interval;
    }
  }

  /**
   * Provide tracking information to vehicles.
   *
   * @param vehicleLists  a mapping from lanes to lists of vehicles sorted by
   *                      their distance on their lanes
   */
  private void provideVehicleTrackingInfo(
    Map<Lane, SortedMap<Double, VehicleSimView>> vehicleLists) {
    // Vehicle Tracking
    for(VehicleSimView vehicle: vinToVehicles.values()) {
      // If the vehicle is autonomous
      if (vehicle instanceof AutoVehicleSimView) {
        AutoVehicleSimView autoVehicle = (AutoVehicleSimView)vehicle;
        if (autoVehicle.isVehicleTracking()) {
          DriverSimView driver = autoVehicle.getDriver();
          Lane targetLane = autoVehicle.getTargetLaneForVehicleTracking();
          Point2D pos = autoVehicle.getPosition();
          double dst = targetLane.distanceAlongLane(pos);
          // initialize the distances to infinity
          double frontDst;
          double rearDst;
          VehicleSimView frontVehicle;
          VehicleSimView rearVehicle;
          // only consider the vehicles on the target lane
          SortedMap<Double,VehicleSimView> vehiclesOnTargetLane = vehicleLists.get(targetLane);
          // compute the distances and the corresponding vehicles
          try {
            double d = vehiclesOnTargetLane.tailMap(dst).firstKey();
            frontVehicle = vehiclesOnTargetLane.get(d);
            frontDst = (d-dst)-frontVehicle.getSpec().getLength();
          } catch(NoSuchElementException e) {
            frontDst = Double.MAX_VALUE;
            frontVehicle = null;
          }
          try {
            double d = vehiclesOnTargetLane.headMap(dst).lastKey();
            rearVehicle = vehiclesOnTargetLane.get(d);
            rearDst = dst-d;
          } catch(NoSuchElementException e) {
            rearDst = Double.MAX_VALUE;
            rearVehicle = null;
          }

          // assign the sensor readings
          autoVehicle.getFrontVehicleDistanceSensor().record(frontDst);
          autoVehicle.getRearVehicleDistanceSensor().record(rearDst);

          // assign the vehicles' velocities
          if(frontVehicle!=null) {
            autoVehicle.getFrontVehicleSpeedSensor().record(frontVehicle.getVelocity());
          } else {
            autoVehicle.getFrontVehicleSpeedSensor().record(Double.MAX_VALUE);
          }
          if(rearVehicle!=null) {
            autoVehicle.getRearVehicleSpeedSensor().record(rearVehicle.getVelocity());
          } else {
            autoVehicle.getRearVehicleSpeedSensor().record(Double.MAX_VALUE);
          }

          // show the section on the viewer
          if (Debug.isTargetVIN(driver.getVehicle().getVIN())) {
            Point2D p1 = targetLane.getPointAtNormalizedDistance(Math.max((dst-rearDst)/targetLane.getLength(),0.0));
            Point2D p2 = targetLane.getPointAtNormalizedDistance(Math.min((frontDst+dst)/targetLane.getLength(),1.0));
            Debug.addLongTermDebugPoint(new DebugPoint(p2, p1, "cl", Color.RED.brighter()));
          }
        }
      }
    }

  }




  /////////////////////////////////
  // STEP 3
  /////////////////////////////////


  private void setRLInforToVehicle() throws IOException {
    boolean flag = false;
    while (!flag){
      flag = readFromRLFile(currentTime);
    }
    for(VehicleSimView vehicle : vinToVehicles.values()) {
      vehicle.getDriver().setBuffersize(vehiclesBufferList.get(vehicle.getVIN())); /////
    }
  }

  private boolean readFromRLFile(double time) throws IOException {
    FileInputStream inputStream = new FileInputStream("BUFFER_UPDATE.csv");
    BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(inputStream));
    // read the file content
    String readTimeStamp = bufferedReader.readLine(); // first line is the timestamp
    try { // if the read content is not a double then return current step
      Double.valueOf(readTimeStamp);
    } catch (Exception ex) {
      return false;
    }
    double timeFromFile = Double.parseDouble(readTimeStamp);
    if(timeFromFile>time){
      String vinlist= bufferedReader.readLine();
      String bufferlist = bufferedReader.readLine();
      String[] vd = vinlist.split(",");
      String[] bd = bufferlist.split(",");
      for (int i=0; i<vd.length;i++){
        try {
          Integer.valueOf(vd[i]);
          Double.valueOf(bd[i]);
        }catch (Exception ex) {
          return false;
        }
      }
      for(int i=0; i<vd.length;i++){
        int vin = Integer.parseInt(vd[i]); ///////
        double bufferSize = Double.parseDouble(bd[i]);
        vehiclesBufferList.put(vin, bufferSize);
      }
      inputStream.close();
      return true;
    }else {
      inputStream.close();
      return false;
    }
  }


  private void outputInfToRL() throws IOException, InterruptedException {
    System.out.println("write to file");
    DecimalFormat fnum = new DecimalFormat("##0.00");
    File writename = new File("GUITORL.csv");
    BufferedWriter out = new BufferedWriter(new FileWriter(writename, false));
    String time = fnum.format(currentTime);
    out.write(time + "\n");
    out.write(RL_episode_flag + "\n");
    out.write("VIN" + "," + "Trust" + ',' + "Velocity" + ',' + "Spawnroad"+","+
            "Destination" + "," +"Buffersize" + ","+ "Collision" + ","+ "Step" + "\n");
    boolean collision_flag = false;
    for (Integer i : vehicleList){
        try {
          if(addCollisionCarAtEndOfEpisode.get(i)==null){
            out.write(i+","+ fnum.format(vehicleTrustValue.get(i)*10) + "," + velocityMap.get(i) + ',' + spawnRoadMap.get(i) + ',');
            out.write( destinationMap.get(i) + "," + vehiclesBufferList.get(i) + "," + collisionMap.get(i)+","+scenario_count%total_scenario+"\n");
          }
          else {
            out.write(i+","+ "-1" + "," + "-1" + ',' + "-1" + ',');
            out.write( "-1" + "," + vehiclesBufferList.get(i) + "," + "-1"+ ","+"-1"+ "\n");
          }
          if (collidedCarInEpisode.get(i)!=null){
            collision_flag = true;
          }
        } catch (Exception e) {
          e.printStackTrace();
        }
    }
    out.flush();
    if(collision_flag){
      System.out.println("there is a collision");

    }
    Thread.sleep(100);
  }


  /**
   * Deliver the V2I and I2V messages.
   */
  private void communication() {
    deliverV2IMessages();
    deliverI2VMessages();
  }
  /** Deliver the V2I messages.*/
  private void deliverV2IMessages() {
    // Go through each vehicle and deliver each of its messages
    for(VehicleSimView vehicle : vinToVehicles.values()) {
      // Start with V2I messages
      if (vehicle instanceof AutoVehicleSimView) {
        AutoVehicleSimView sender = (AutoVehicleSimView)vehicle;
        Queue<V2IMessage> v2iOutbox = sender.getV2IOutbox();
        Queue<V2IMessage> v2iRealOutbox = sender.getrealV2IOutbox();
        while(!v2iOutbox.isEmpty()&& (!v2iRealOutbox.isEmpty())) {
          V2IMessage msg = v2iOutbox.poll();
          V2IMessage realmsg = v2iRealOutbox.poll();
          V2IManager receiver = (V2IManager)basicMap.getImRegistry().get(msg.getImId());
          // Calculate the distance the message must travel
          double txDistance = sender.getPosition().distance(receiver.getIntersection().getCentroid());
          // Find out if the message will make it that far
          if(transmit(txDistance, sender.getTransmissionPower())) {
            // Actually deliver the message
            receiver.receive(msg, realmsg);
            // Add the delivery to the debugging information
          }
          // Either way, we increment the number of transmitted messages
        }
      }
    }
  }

  /** Deliver the I2V messages.*/
  private void deliverI2VMessages() {
    // Now deliver all the I2V messages
    for(IntersectionManager im : basicMap.getIntersectionManagers()) {
      V2IManager senderIM = (V2IManager)im;
      for(Iterator<I2VMessage> i2vIter = senderIM.outboxIterator(); i2vIter.hasNext();) {
        I2VMessage msg = i2vIter.next();
        AutoVehicleSimView vehicle = (AutoVehicleSimView)VinRegistry.getVehicleFromVIN(msg.getVin());
        // Calculate the distance the message must travel
        assert vehicle != null;
        double txDistance = senderIM.getIntersection().getCentroid().distance(vehicle.getPosition());
        // Find out if the message will make it that far
        if(transmit(txDistance, senderIM.getTransmissionPower())) {
          // Actually deliver the message
          vehicle.receive(msg);
        }
      }
      // Done delivering the IntersectionManager's messages, so clear the
      // outbox.
      senderIM.clearOutbox();
    }
  }

  /////////////////////////////////
  // STEP 5
  /////////////////////////////////


  /**
   * Allow each intersection manager to act.
   *
   * @param timeStep  the time step
   */
  private void letIntersectionManagersAct(double timeStep) {
    for(IntersectionManager im : basicMap.getIntersectionManagers()) {
      im.act(timeStep);
    }
  }


  /////////////////////////////////
  // STEP 6
  /////////////////////////////////

  /** Allow each driver to act. */
  private void letDriversAct() {
    for(VehicleSimView vehicle : vinToVehicles.values()) {
     vehicle.getDriver().act();
    }
  }




  /**
   * Whether the transmission of a message is successful
   *
   * @param distance  the distance of the transmission
   * @param power     the power of the transmission
   * @return whether the transmission of a messsage is successful
   */
  private boolean transmit(double distance, double power) {// Simple for now
    return distance <= power;
  }



  /**
   * Move all the vehicles.
   *
   * @param timeStep  the time step
   */
  private void moveVehicles(double timeStep) {
    for(VehicleSimView vehicle : vinToVehicles.values()) {
      Point2D p1 = vehicle.getPosition();
      vehicle.move(timeStep);
      Point2D p2 = vehicle.getPosition();
      for(DataCollectionLine line : basicMap.getDataCollectionLines()) {
        line.intersect(vehicle, currentTime, p1, p2);
      }
      if (Debug.isPrintVehicleStateOfVIN(vehicle.getVIN())) {
        vehicle.printState();
      }
    }
  }

  private Map<Integer, Integer>  detectCollision() {
    collisionVINs = new HashMap<>();
    for (VehicleSimView vehicle : vinToVehicles.values()) {
//      System.out.println(vehicle.getVIN()+ ":" + vehicle.getDriver().distanceToNextIntersection());
      collisionVINs.put(vehicle.getVIN(), 0);
      if (vehicle.getDriver().distanceToNextIntersection()==0 &&
              vehicle.getDriver().distanceFromPrevIntersection()>1000){ //in the intersection
        for (VehicleSimView vehicle2 : vinToVehicles.values()) {
          if (vehicle.getVIN() != vehicle2.getVIN()) {
            double dist = vehicle.getCenterPoint().distance(vehicle2.getCenterPoint());
//          System.out.println(vehicle.getVIN()+"/"+vehicle2.getVIN()+":"+dist);
            if(dist<3){
              collisionVINs.put(vehicle.getVIN(), 1);
              collisionVINs.put(vehicle2.getVIN(), 1);
            }
          }
        }
      }
//      System.out.println(vehicle.getVIN()+": "+vehicle.getDriver().distanceToNextIntersection());
    }
    return collisionVINs;
  }


  /////////////////////////////////
  // STEP 7
  /////////////////////////////////

  /**
   * Remove all completed vehicles.
   *
   * @return the VINs of the completed vehicles
   */
  private List<Integer> cleanUpCompletedVehicles(Map<Integer,Integer> collisionlist) {
    List<Integer> completedVINs = new LinkedList<>();
    Rectangle2D mapBoundary = basicMap.getDimensions();
    List<Integer> removedVINs = new ArrayList<>(vinToVehicles.size());

    for(VehicleSimView vehicle: vinToVehicles.values()) {
      // If the vehicle is no longer in the layout
      if(!vehicle.getShape().intersects(mapBoundary)) {
        // Process all the things we need to from this vehicle
        if (vehicle instanceof AutoVehicleSimView) {
          AutoVehicleSimView v2 = (AutoVehicleSimView)vehicle;
          totalBitsTransmittedByCompletedVehicles += v2.getBitsTransmitted();
          totalBitsReceivedByCompletedVehicles += v2.getBitsReceived();
        }
        try {
          numOfCompletedVehicles++;
        } catch (Exception e) {
          e.printStackTrace();
        }
        removedVINs.add(vehicle.getVIN());
        completeCarInStep.put(vehicle.getVIN(), 1);
      }
    }
    for(int vin: collisionlist.keySet()){
      if (vinToVehicles.get(vin)!= null){
        if (collisionlist.get(vin) == 1){
          numOfCollisionVehicles++;
          removedVINs.add(vin);
        }
      }
    }

    for(int vin : removedVINs) {
      collisionMap.put(vin, collisionVINs.get(vin));
      boolean trustflag;
      trustflag = vinToVehicles.get(vin).getDriver().getRealDestination() == vinToVehicles.get(vin).getDriver().getDestination();
//      System.out.println("trustflag of " + vin + " : "+ trustflag);
      trustcalculation(vin, trustflag);
      vinToVehicles.remove(vin);
      completedVINs.add(vin);
      VinRegistry.unregisterVehicle(vin);
    }
    return completedVINs;
  }




  private void trustcalculation(int VIN, boolean trust_flag){
    ArrayList<Double> newTrust;
    if (trust_flag){
      newTrust = fusion(belief, vehiclesTrustList.get(VIN));
    }else {
      newTrust = fusion(disbelief, vehiclesTrustList.get(VIN));
    }
    vehicleTrustValue.put(VIN, newTrust.get(0)+newTrust.get(2)*newTrust.get(3));
    vehiclesTrustList.put(VIN, newTrust);
//    System.out.println("update "+ VIN + " to "+ vehiclesTrustList.get(VIN));
  }

  private ArrayList<Double> fusion(ArrayList<Double> W_1, ArrayList<Double> W_2){
    double b, d, u, a;
    ArrayList<Double> newList = new ArrayList<>();
    if (W_1.get(2) == 0 && W_2.get(2) == 0){
      b = 0.5 * W_1.get(0) + 0.5 * W_2.get(0);
      d = 0.5 * W_1.get(1) + 0.5 * W_2.get(1);
      u = 0;
      a = 0.5 * W_1.get(3) + 0.5 * W_2.get(3);
    }else {
      b = (W_1.get(0) * W_2.get(2) + W_2.get(0) * W_1.get(2))/(W_1.get(2) + W_2.get(2) - W_1.get(2) * W_2.get(2));
      d = (W_1.get(1) * W_2.get(2) + W_2.get(1) * W_1.get(2))/(W_1.get(2) + W_2.get(2) - W_1.get(2) * W_2.get(2));
      u = (W_1.get(2) * W_2.get(2))/(W_1.get(2) + W_2.get(2) - W_1.get(2) * W_2.get(2));
      if (W_1.get(2) == 1 && W_2.get(2) == 1){
        a = (W_1.get(3) + W_2.get(3))/2;
      }
      else{
        a = (W_1.get(3) * W_2.get(2) + W_2.get(3) * W_1.get(2) -(W_1.get(3) + W_2.get(3))*
                W_1.get(2) * W_2.get(2))/(W_1.get(2) + W_2.get(2) -2* W_1.get(2) * W_2.get(2));
      }
    }
    newList.add(b);
    newList.add(d);
    newList.add(u);
    newList.add(a);
    return newList;
  }


}

