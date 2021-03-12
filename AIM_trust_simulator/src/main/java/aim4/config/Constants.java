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
package aim4.config;

import java.text.*;

/**
 * A class to hold constants for the simulator.
 */
public final class Constants {


  public static final int car_id_max = 10;
  /**
   * A NumberFormat for one place after the decimal.  Used a lot, so might as
   * well just define it once.
   */
  public static final NumberFormat ONE_DEC = new DecimalFormat("#.0");

  /**
   * A NumberFormat for two places after the decimal.  Used a lot, so might as
   * well just define it once.
   */
  public static final NumberFormat TWO_DEC = new DecimalFormat("#.00");

  /**
   * The size, in bits, of an integer on our theoretical platform.
   * {@value} bits.
   */
  public static final int INTEGER_SIZE = 32; // 32 bits

  /**
   * The size, in bits, of a double-precision floating point number on our
   * theoretical platform. {@value} bits.
   */
  public static final int DOUBLE_SIZE = 64; // 64 bits

  /**
   * The size, in bits, of an enumerated data type on our theoretical
   * platform. {@value} bits.
   */
  public static final int ENUM_SIZE = 8; // 8 bits

  /** The number of bits per byte ({@value}). */
  public static final int BITS_PER_BYTE = 8;

  /** The number of bytes per kilobyte ({@value}). */
  public static final int BYTES_PER_KB = 1024;

  /** The number of bits per kilobyte ({@value}). */
  public static final int BITS_PER_KB = BYTES_PER_KB * BITS_PER_BYTE;

  /** The number of bytes per megabyte ({@value}). */
  public static final int BYTES_PER_MB = BYTES_PER_KB * 1024;

  /**
   * The precision with which two double values are considered equal.
   * The equality of two double values a and b should be tested by
   * using Math.abs(a-b) < Constants.DOUBLE_EQUAL_PRECISION. {@value}
   */
  public static final double DOUBLE_EQUAL_PRECISION = 0.0000000001;

  /**
   * The precision with which two double values are considered equal.
   * The equality of two double values a and b should be tested by
   * using Math.abs(a-b) < Constants.DOUBLE_EQUAL_PRECISION. {@value}
   */
  public static final double DOUBLE_EQUAL_WEAK_PRECISION = 0.000001;


  /**
   * A direction to turn at an intersection.
   */
  public enum TurnDirection {
    LEFT,
    RIGHT,
    STRAIGHT,
    /** 180 degree turn back to the same road (currently not allowed).*/
    U_TURN,
  }


}
