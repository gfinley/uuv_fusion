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

# Message to hold environmental readings.

Header header                       # Standard header
Metadata metadata                   # Message metadata
seebyte_vehicle_interface/CTD ctd   # Conductivity, temperature & depth
float32 current_direction_n         # Water current speed north (metres/second)
float32 current_direction_e         # Water current speed east (metres/second)
float32 current_direction_v         # Water current speed vertically in vehicle body frame (metres/second)
float32 current_heading             # Water current heading in north-east plane (degrees)
float32 current_speed               # Water current speed (meters/second)
float32 fluorescence                # Measure of chlorophyll fluorescence (micrograms/liter)
float32 turbidity                   # Measure of particulates in water (Nephelometric Turbity Units)
float32 concentration               # Measure of dissolved oxygen (microMoles/liter)
float32 saturation                  # Measure of dissolved oxygen (percent of saturation)
