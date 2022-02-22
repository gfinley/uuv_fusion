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

# Message to hold conductivity, temperature & depth readings.

Header header           # Standard header
Metadata metadata       # Message metadata
float32 conductivity    # Electrolytic conductivity (milli-Siemens/cm)
float32 salinity        # Water salinity (parts per thousand)
float32 temperature     # Water temperature (degrees Celsius)
float32 sound_speed     # Sound speed (meters/second)
float32 pressure        # Pressure (Pascal) - Deprecated field
float32 depth           # Depth (meters)
