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
package aim4.im.v2i.RequestHandler;

import aim4.im.v2i.policy.Policy;
import aim4.im.v2i.policy.Policy.ProposalFilterResult;
import aim4.im.v2i.policy.Policy.ReserveParam;
import aim4.msg.i2v.Reject;
import aim4.msg.v2i.Request;

import java.awt.*;

/**
 * The "First Come, First Served" request handler.
 */
public class RequestHandler {

  /////////////////////////////////
  // PRIVATE FIELDS
  /////////////////////////////////

  /** The base policy */
  private Policy basePolicy = null;


  /////////////////////////////////
  // PUBLIC METHODS
  /////////////////////////////////

  /**
   * Set the base policy call-back.
   *
   * @param basePolicy  the base policy's call-back
   */
  public void setBasePolicyCallback(Policy basePolicy) {
    this.basePolicy = basePolicy;
  }

  /**
   * Process the request message.
   *
   * @param msg the request message
   */

  public void processRequestMsg(Request msg, Request realmsg) {
    int vin = msg.getVin();
    // If the vehicle has got a reservation already, reject it.
//    if (basePolicy.hasReservation(vin)) {
//      basePolicy.sendRejectMsg(vin, msg.getRequestId(), Reject.Reason.CONFIRMED_ANOTHER_REQUEST);
//      return;
//    }
    // filter the proposals
    ProposalFilterResult filterResult = Policy.standardProposalsFilter(msg.getProposals(),
                                         basePolicy.getCurrentTime());
    if (filterResult.isNoProposalLeft()) {
      basePolicy.sendRejectMsg(vin, msg.getRequestId(), filterResult.getReason());
    }
    ProposalFilterResult realfilterResult = Policy.standardProposalsFilter(realmsg.getProposals(),
            basePolicy.getCurrentTime());
    if (realfilterResult.isNoProposalLeft()) {
      basePolicy.sendRejectMsg(vin, realmsg.getRequestId(), realfilterResult.getReason());
    }
    // try to see if reservation is possible for the remaining proposals.
    ReserveParam reserveParam = basePolicy.findReserveParam(msg, filterResult.getProposals(), realfilterResult.getProposals());
    if (reserveParam != null) {
      basePolicy.sendComfirmMsg(msg.getRequestId(), reserveParam);
    } else {
      basePolicy.sendRejectMsg(vin, msg.getRequestId(), Reject.Reason.NO_CLEAR_PATH);
    }
  }

}
