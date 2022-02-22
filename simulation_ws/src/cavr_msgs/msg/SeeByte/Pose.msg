##############################################################################
# Copyright (c) 2007-2018, SeeByte Ltd.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
# OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
#
# THIS IS COMMERCIAL OFF THE SHELF SOFTWARE SOLD UNDER
# A SEEBYTE STANDARD MULTI SEAT LICENSE
#
##############################################################################

# Message holding full position and orientation information for a vehicle or other body.

Header header                    # Standard header
Metadata metadata                # Message metadata
GlobalPosition global_position   # Position in latitude, longitude, altitude and depth
LatLon origin                    # Local frame reference point (latitude, longitude)
geometry_msgs/Point attitude     # Orientation relative to local frame (radians)
                                 # Axes
                                 #  x = +front/-back
                                 #  y = +right/-left
                                 #  z = +down/-up
                                 # Rotations
                                 #  All rotations are cw around axis
                                 #  x = roll
                                 #  y = pitch
                                 #  z = yaw
geometry_msgs/Twist velocity     # Velocity for each axis (meters/second, radians/second)
                                 # Axes
                                 # x = +front/-back
                                 # y = +right/-left
                                 # z = +down/-up

