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
package aim4.vehicle;

import aim4.util.Util;

/**
 * A gauge that holds doubles.  This gauge also can apply noise
 * functions to incoming data to simulate a noisy gauge.
 */
public class DoubleGauge {
  /**
   * The actual value being measured by the gauge.
   */
  private double value = 0;
  /**
   * The maximum value the gauge can read.
   */
  private double maxValue = Double.MAX_VALUE;
  /**
   * The minimum value the gauge can read
   */
  private double minValue = Double.MIN_VALUE;


  // Constructors
  /**
   * Class constructor for unlimited, uninitialized, noiseless gauge.
   */
  public DoubleGauge() {}


  // Get and Set

  /**
   * Read the value of the gauge.
   *
   * @return the value of the gauge.
   */
  public double read() {
    return value;
  }

  /**
   * Records a value to the gauge
   *
   * @param recValue the value to be written to the gauge
   */
  public void record(double recValue) {
    value = Util.constrain(recValue, minValue, maxValue);
  }
}
