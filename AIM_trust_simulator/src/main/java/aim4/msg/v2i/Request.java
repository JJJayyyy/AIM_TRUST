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
package aim4.msg.v2i;

import java.util.Collections;
import java.util.List;

import aim4.config.Constants;
import aim4.vehicle.VehicleSpec;

/**
 * Message sent from a Vehicle to an Intersection Manager to request a
 * reservation.
 */
public class Request extends V2IMessage {

  // ///////////////////////////////
  // NESTED CLASSES
  // ///////////////////////////////

  /**
   * A proposal
   */
  public static class Proposal {

    // ///////////////////////////////
    // PRIVATE FIELDS
    // ///////////////////////////////

    /** The ID number of the lane in which the vehicle will be arriving. */
    private int arrivalLaneID;
    /** The ID number of the lane in which the vehicle will depart. */
    private int departureLaneID;
    /** When the Vehicle plans to arrive at the intersection. */
    private double arrivalTime;
    /**
     * The velocity at which the vehicle will be arriving, in meters per second.
     */
    private double arrivalVelocity;
    /**
     * The maximum velocity the vehicle can sustain given the arrival and departure lanes.
     */
    private double maximumTurnVelocity;


    /////////////////////////////////
    // CONSTRUCTORS
    /////////////////////////////////

    /**
     * Create a proposal.
     *
     * @param arrivalLaneID    the ID of the arrival lane
     * @param departureLaneID  the ID of the departure lane
     * @param arrivalTime      the arrival time
     * @param arrivalVelocity  the arrival velocity
     * @param maxTurnVelocity  the maximum turn velocity
     */
    public Proposal(int arrivalLaneID, int departureLaneID, double arrivalTime,
                    double arrivalVelocity, double maxTurnVelocity) {
      this.arrivalLaneID = arrivalLaneID;
      this.departureLaneID = departureLaneID;
      this.arrivalTime = arrivalTime;
      this.arrivalVelocity = arrivalVelocity;
      this.maximumTurnVelocity = maxTurnVelocity;
    }

    // ///////////////////////////////
    // PUBLIC METHODS
    // ///////////////////////////////

    /**
     * Get the lane ID number for the Lane in which the vehicle would like to
     * arrive at the intersection.
     *
     * @return the lane ID number for the Lane in which the vehicle would like
     *         to arrive at the intersection
     */
    public int getArrivalLaneID() {
      return arrivalLaneID;
    }

    /**
     * Get the ID number for the Lane in which the vehicle would like to depart
     * the intersection.
     *
     * @return the ID number for the Lane in which the vehicle would like to
     *         depart the intersection
     */
    public int getDepartureLaneID() {
      return departureLaneID;
    }

    /**
     * Get the time at which the vehicle wants to arrive at the intersection.
     *
     * @return the time at which the vehicle wants to arrive at the intersection
     */
    public double getArrivalTime() {
      return arrivalTime;
    }

    /**
     * Get the velocity at which the vehicle wants to arrive at the
     * intersection.
     *
     * @return the velocity at which the vehicle wants to arrive at the
     *         intersection
     */
    public double getArrivalVelocity() {
      return arrivalVelocity;
    }

    /**
     * Get the requesting vehicle's maximum velocity for this pair of arrival
     * and departure lanes.
     *
     * @return the requesting vehicle's maximum velocity for this pair of
     *         arrival and departure lanes
     */
    public double getMaximumTurnVelocity() {
      return maximumTurnVelocity;
    }

    // ///////////////////////////////
    // FOR DEBUG
    // ///////////////////////////////

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
      return "Proposal(Lane" + String.format("%2d", arrivalLaneID)
        + " -> Lane" + String.format("%2d", departureLaneID) + " arrive at "
        + String.format("%.2f", arrivalTime) + " at speed "
        + String.format("%.2f", arrivalVelocity) + " (maxTurnVelocity:"
        + String.format("%.2f", maximumTurnVelocity) + "))";
    }

  }

  /**
   * The specification of the vehicle.
   */
  public static class VehicleSpecForRequestMsg {

    /**
     * The highest acceleration of which the vehicle is capable, in meters per
     * second squared.
     */
    private double maxAcceleration;

    /**
     * The lowest acceleration of which the vehicle is capable (in other words
     * the maximum rate of deceleration), in meters per second squared.
     */
    private double maxDeceleration;

    /**
     * The lowest velocity of which the vehicle is capable, in meters per
     * second. In other words, how fast the vehicle can travel backward.
     */
    private double minVelocity;

    /**
     * The length of the vehicle, in meters.
     */
    private double length;

    /**
     * The width of the vehicle, in meters.
     */
    private double width;

    /**
     * The distance from the front of the vehicle to its front axle, in meters.
     */
    private double frontAxleDisplacement;

    /**
     * The distance from the front of the vehicle to its rear axle, in meters.
     */
    private double rearAxleDisplacement;

    /**
     * The maximum angle away from "straight ahead" that the front wheels can be
     * turned, in radians.
     */
    private double maxSteeringAngle;

    /**
     * The maximum rate at which the front wheels can be turned, in radians per
     * second.
     */
    private double maxTurnPerSecond;

    private double bufferSize;

    // ///////////////////////////////
    // CONSTRUCTORS
    // ///////////////////////////////

    /**
     * Create a copy of a given vehicle specification for request message
     *
     * @param vspec  the vehicle specification
     */
    public VehicleSpecForRequestMsg(VehicleSpec vspec) {
      this.maxAcceleration = vspec.getMaxAcceleration();
      this.maxDeceleration = vspec.getMaxDeceleration();
      this.minVelocity = vspec.getMinVelocity();
      this.length = vspec.getLength();
      this.width = vspec.getWidth();
      this.frontAxleDisplacement = vspec.getFrontAxleDisplacement();
      this.rearAxleDisplacement = vspec.getRearAxleDisplacement();
      this.maxSteeringAngle = vspec.getMaxSteeringAngle();
      this.maxTurnPerSecond = vspec.getMaxTurnPerSecond();
    }

    // ///////////////////////////////
    // PUBLIC METHODS
    // ///////////////////////////////

    /**
     * @return the maxAcceleration
     */
    public double getMaxAcceleration() {
      return maxAcceleration;
    }

    /**
     * @return the minAcceleration
     */
    public double getMaxDeceleration() {
      return maxDeceleration;
    }

    /**
     * @return the minVelocity
     */
    public double getMinVelocity() {
      return minVelocity;
    }

    /**
     * @return the length
     */
    public double getLength() {
      return length;
    }

    /**
     * @return the width
     */
    public double getWidth() {
      return width;
    }

    /**
     * @return the frontAxleDisplacement
     */
    public double getFrontAxleDisplacement() {
      return frontAxleDisplacement;
    }

    /**
     * @return the rearAxleDisplacement
     */
    public double getRearAxleDisplacement() {
      return rearAxleDisplacement;
    }

    /**
     * @return the maxSteeringAngle
     */
    public double getMaxSteeringAngle() {
      return maxSteeringAngle;
    }

    /**
     * @return the maxTurnPerSecond
     */
    public double getMaxTurnPerSecond() {
      return maxTurnPerSecond;
    }


  }

  // ///////////////////////////////
  // PRIVATE FIELDS
  // ///////////////////////////////

  /**
   * The unique request number of the request message sent the by vehicle to the
   * intersection manager.
   */
  private int requestId;

  /**
   * The specification of the vehicle
   */
  private VehicleSpecForRequestMsg spec;

  /**
   * The proposed traversals for the vehicle, including arrival lane, departure
   * lane, arrival time, arrival velocity, and maximum velocity for each
   * proposal. They are ordered by priority with the highest priority first.
   */
  private List<Proposal> proposals;


  /////////////////////////////////
  // CONSTRUCTORS
  /////////////////////////////////

  /**
   * Basic class constructor with all required fields.
   *
   * @param sourceID       the ID number of the Vehicle sending this message
   * @param destinationID  the ID number of the IntersectionManager to which
   *                       this message is being sent
   * @param requestId      the request id
   * @param spec           the specification of the vehicle
   * @param proposals      the proposals
   */
  public Request(int sourceID, int destinationID, int requestId,
                 VehicleSpecForRequestMsg spec,
                 List<Proposal> proposals, double buffersize, boolean reliability, int waitcycle) {
    super(sourceID, destinationID, buffersize, reliability, waitcycle);
    this.requestId = requestId;
    this.spec = spec;
    this.proposals = proposals;
    // Make sure our parameters are well-formed
    if (proposals.isEmpty()) {
      throw new IllegalArgumentException("Trajectory parameters can't be "
        + "empty!");
    }

    // Everything else is just an assignment
    messageType = Type.REQUEST;
    size +=
      9 * Constants.DOUBLE_SIZE
      + Constants.INTEGER_SIZE
      + (2 * Constants.INTEGER_SIZE + 3 * Constants.DOUBLE_SIZE)
        * proposals.size();
  }


  /////////////////////////////////
  // PUBLIC METHODS
  /////////////////////////////////

  // Getters

  /**
   * Get the request ID.
   */
  public int getRequestId() {
    return requestId;
  }

  /**
   * Get the specification of the vehicle
   *
   * @return the specification of the vehicle.
   */
  public VehicleSpecForRequestMsg getSpec() {
    return spec;
  }

  /**
   * Get the List of Proposals in this request.
   *
   * @return the List of Proposals in this Request
   */
  public List<Proposal> getProposals() {
    return Collections.unmodifiableList(proposals);
  }

}
