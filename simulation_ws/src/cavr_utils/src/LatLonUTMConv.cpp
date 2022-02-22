// LLUTMConverstion.cpp : Defines the entry point for the console application.
//
/************ source of code**************************
 http://www.gpsy.com/gpsinfo/geotoutm/
 Date: Fri, 05 Jun 1998 14:32:23 -0700
 From: "Chuck Gantz" chuck.gantz@globalstar.com
 Enclosures: LatLong-UTMconversion.cpp (view online as text file)
 LatLong-UTMconversion.h (view online as text file)
 UTMConversions.cpp (view online as text file)
 SwissGrid.cpp (view online as text file)
 constants.h (view online as text file)
 ******************************************************/

#include <cavr_utils/LatLonUTMConv.h>
#ifndef INCLUDED_PRECOMPILED_HEADERS
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <math.h>
#include <stdio.h>
#endif // INCLUDED_PRECOMPILED_HEADERS
/*   Start  for coordinate conversion Test routine
 void LLUTMConversionTest(void);

 void main()
 { LLUTMConversionTest();}

 void LLUTMConversionTest()
 {
 double Lat = 31.23149; // enter latitude and longitude by hand and recompile
 double Long = -97.68751;
 double UTMNorthing;
 double UTMEasting;
 double SwissNorthing;
 double SwissEasting;
 char UTMZone[4];
 int RefEllipsoid = 23;//WGS-84. See list with file "LatLong- UTM conversion.cpp" for id numbers

 cout << "Starting position(Lat, Long):  " << Lat << "   " << Long <<endl;

 LLtoUTM(RefEllipsoid, Lat, Long, UTMNorthing, UTMEasting, UTMZone);
 cout << setiosflags(ios::showpoint | ios::fixed) << setprecision(5);
 cout << "Calculated UTM position(Northing, Easting, Zone):  ";
 cout << UTMNorthing << "   " << UTMEasting;
 cout << "   " << UTMZone <<endl;

 UTMtoLL(RefEllipsoid, UTMNorthing, UTMEasting, UTMZone, Lat, Long);
 cout << "Calculated Lat, Long position(Lat, Long):  " << Lat << "   " << Long << endl <<endl;

 LLtoSwissGrid(Lat, Long, SwissNorthing, SwissEasting);
 cout << setiosflags(ios::showpoint | ios::fixed) << setprecision(5);
 cout << "Calculated Swiss Grid position(Northing, Easting):  ";
 cout << SwissNorthing << "   " << SwissEasting << endl;
 }

 //   End  for coordinate conversion Test routine */

/* N 47.38195� E 8.54879�  (Swiss Grid: 683.748 248.342)
 N 47�12.625' / E 7� 27.103'= N 47.21041667 E 7.45171667(Swiss Grid = 600920/228685)
 N 47�22.690' / E 8� 13.950'= N 47.37816667 E 8.23250000 (Swiss Grid = 659879/247637)
 */

/*Reference ellipsoids derived from Peter H. Dana's website-
 http://www.utexas.edu/depts/grg/gcraft/notes/datum/elist.html
 Department of Geography, University of Texas at Austin
 Internet: pdana@mail.utexas.edu
 3/22/95

 Source
 Defense Mapping Agency. 1987b. DMA Technical Report: Supplement to Department of Defense World Geodetic System
 1984 Technical Report. Part I and II. Washington, DC: Defense Mapping Agency
 */

void LLtoUTM(int ReferenceEllipsoid, const double Lat, const double Long,
             double &UTMNorthing, double &UTMEasting, char* UTMZone)
{
  //converts lat/long to UTM coords.  Equations from USGS Bulletin 1532
  //East Longitudes are positive, West longitudes are negative.
  //North latitudes are positive, South latitudes are negative
  //Lat and Long are in decimal degrees
  //Written by Chuck Gantz- chuck.gantz@globalstar.com

  double a = ellipsoid[ReferenceEllipsoid].EquatorialRadius;
  double eccSquared = ellipsoid[ReferenceEllipsoid].eccentricitySquared;
  double k0 = 0.9996;

  double LongOrigin;
  double eccPrimeSquared;
  double N, T, C, A, M;

  //Make sure the longitude is between -180.00 .. 179.9
  double LongTemp = (Long + 180) - int((Long + 180) / 360) * 360 - 180; // -180.00 .. 179.9;

  double LatRad = Lat * deg2rad;
  double LongRad = LongTemp * deg2rad;
  double LongOriginRad;
  int ZoneNumber;

  ZoneNumber = int((LongTemp + 180) / 6) + 1;

  if (Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0)
    ZoneNumber = 32;

  // Special zones for Svalbard
  if (Lat >= 72.0 && Lat < 84.0)
  {
    if (LongTemp >= 0.0 && LongTemp < 9.0) ZoneNumber = 31;
    else if (LongTemp >= 9.0 && LongTemp < 21.0) ZoneNumber = 33;
    else if (LongTemp >= 21.0 && LongTemp < 33.0) ZoneNumber = 35;
    else if (LongTemp >= 33.0 && LongTemp < 42.0) ZoneNumber = 37;
  }
  LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3; //+3 puts origin in middle of zone
  LongOriginRad = LongOrigin * deg2rad;

  //compute the UTM Zone from the latitude and longitude
  sprintf(UTMZone, "%d%c", ZoneNumber, UTMLetterDesignator(Lat));

  eccPrimeSquared = (eccSquared) / (1 - eccSquared);

  N = a / sqrt(1 - eccSquared * sin(LatRad) * sin(LatRad));
  T = tan(LatRad) * tan(LatRad);
  C = eccPrimeSquared * cos(LatRad) * cos(LatRad);
  A = cos(LatRad) * (LongRad - LongOriginRad);

  M =
      a
          * ((1 - eccSquared / 4 - 3 * eccSquared * eccSquared / 64
              - 5 * eccSquared * eccSquared * eccSquared / 256) * LatRad
              - (3 * eccSquared / 8 + 3 * eccSquared * eccSquared / 32
                  + 45 * eccSquared * eccSquared * eccSquared / 1024)
                  * sin(2 * LatRad)
              + (15 * eccSquared * eccSquared / 256
                  + 45 * eccSquared * eccSquared * eccSquared / 1024)
                  * sin(4 * LatRad)
              - (35 * eccSquared * eccSquared * eccSquared / 3072)
                  * sin(6 * LatRad));

  UTMEasting = (double) (k0 * N
      * (A + (1 - T + C) * A * A * A / 6
          + (5 - 18 * T + T * T + 72 * C - 58 * eccPrimeSquared) * A * A * A * A
              * A / 120) + 500000.0);

  UTMNorthing = (double) (k0
      * (M
          + N * tan(LatRad)
              * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24
                  + (61 - 58 * T + T * T + 600 * C - 330 * eccPrimeSquared) * A
                      * A * A * A * A * A / 720)));
  if (Lat < 0) UTMNorthing += 10000000.0; //10000000 meter offset for southern hemisphere
}

char UTMLetterDesignator(double Lat)
{
  //This routine determines the correct UTM letter designator for the given latitude
  //returns 'Z' if latitude is outside the UTM limits of 84N to 80S
  //Written by Chuck Gantz- chuck.gantz@globalstar.com
  char LetterDesignator;

  if ((84 >= Lat) && (Lat >= 72)) LetterDesignator = 'X';
  else if ((72 > Lat) && (Lat >= 64)) LetterDesignator = 'W';
  else if ((64 > Lat) && (Lat >= 56)) LetterDesignator = 'V';
  else if ((56 > Lat) && (Lat >= 48)) LetterDesignator = 'U';
  else if ((48 > Lat) && (Lat >= 40)) LetterDesignator = 'T';
  else if ((40 > Lat) && (Lat >= 32)) LetterDesignator = 'S';
  else if ((32 > Lat) && (Lat >= 24)) LetterDesignator = 'R';
  else if ((24 > Lat) && (Lat >= 16)) LetterDesignator = 'Q';
  else if ((16 > Lat) && (Lat >= 8)) LetterDesignator = 'P';
  else if ((8 > Lat) && (Lat >= 0)) LetterDesignator = 'N';
  else if ((0 > Lat) && (Lat >= -8)) LetterDesignator = 'M';
  else if ((-8 > Lat) && (Lat >= -16)) LetterDesignator = 'L';
  else if ((-16 > Lat) && (Lat >= -24)) LetterDesignator = 'K';
  else if ((-24 > Lat) && (Lat >= -32)) LetterDesignator = 'J';
  else if ((-32 > Lat) && (Lat >= -40)) LetterDesignator = 'H';
  else if ((-40 > Lat) && (Lat >= -48)) LetterDesignator = 'G';
  else if ((-48 > Lat) && (Lat >= -56)) LetterDesignator = 'F';
  else if ((-56 > Lat) && (Lat >= -64)) LetterDesignator = 'E';
  else if ((-64 > Lat) && (Lat >= -72)) LetterDesignator = 'D';
  else if ((-72 > Lat) && (Lat >= -80)) LetterDesignator = 'C';
  else LetterDesignator = 'Z'; //This is here as an error flag to show that the Latitude is outside the UTM limits

  return LetterDesignator;
}

void UTMtoLL(int ReferenceEllipsoid, const double UTMNorthing,
             const double UTMEasting, const char* UTMZone, double& Lat,
             double& Long)
{
  //converts UTM coords to lat/long.  Equations from USGS Bulletin 1532
  //East Longitudes are positive, West longitudes are negative.
  //North latitudes are positive, South latitudes are negative
  //Lat and Long are in decimal degrees.
  //Written by Chuck Gantz- chuck.gantz@globalstar.com

  double k0 = 0.9996;
  double a = ellipsoid[ReferenceEllipsoid].EquatorialRadius;
  double eccSquared = ellipsoid[ReferenceEllipsoid].eccentricitySquared;
  double eccPrimeSquared;
  double e1 = (1 - sqrt(1 - eccSquared)) / (1 + sqrt(1 - eccSquared));
  double N1, T1, C1, R1, D, M;
  double LongOrigin;
  double mu, phi1, phi1Rad;
  double x, y;
  int ZoneNumber;
  char* ZoneLetter;
  int NorthernHemisphere; //1 for northern hemispher, 0 for southern

  x = UTMEasting - 500000.0; //remove 500,000 meter offset for longitude
  y = UTMNorthing;

  ZoneNumber = strtoul(UTMZone, &ZoneLetter, 10);
  if ((*ZoneLetter - 'N') >= 0) NorthernHemisphere = 1; //point is in northern hemisphere
  else
  {
    NorthernHemisphere = 0; //point is in southern hemisphere
    y -= 10000000.0; //remove 10,000,000 meter offset used for southern hemisphere
  }

  LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3; //+3 puts origin in middle of zone

  eccPrimeSquared = (eccSquared) / (1 - eccSquared);

  M = y / k0;
  mu = M
      / (a
          * (1 - eccSquared / 4 - 3 * eccSquared * eccSquared / 64
              - 5 * eccSquared * eccSquared * eccSquared / 256));

  phi1Rad = mu + (3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * sin(2 * mu)
      + (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * sin(4 * mu)
      + (151 * e1 * e1 * e1 / 96) * sin(6 * mu);
  phi1 = phi1Rad * rad2deg;

  N1 = a / sqrt(1 - eccSquared * sin(phi1Rad) * sin(phi1Rad));
  T1 = tan(phi1Rad) * tan(phi1Rad);
  C1 = eccPrimeSquared * cos(phi1Rad) * cos(phi1Rad);
  R1 = a * (1 - eccSquared)
      / pow(1 - eccSquared * sin(phi1Rad) * sin(phi1Rad), 1.5);
  D = x / (N1 * k0);

  Lat = phi1Rad
      - (N1 * tan(phi1Rad) / R1)
          * (D * D / 2
              - (5 + 3 * T1 + 10 * C1 - 4 * C1 * C1 - 9 * eccPrimeSquared) * D
                  * D * D * D / 24
              + (61 + 90 * T1 + 298 * C1 + 45 * T1 * T1 - 252 * eccPrimeSquared
                  - 3 * C1 * C1) * D * D * D * D * D * D / 720);
  Lat = Lat * rad2deg;

  Long =
      (D - (1 + 2 * T1 + C1) * D * D * D / 6
          + (5 - 2 * C1 + 28 * T1 - 3 * C1 * C1 + 8 * eccPrimeSquared
              + 24 * T1 * T1) * D * D * D * D * D / 120) / cos(phi1Rad);
  Long = LongOrigin + Long * rad2deg;

}

void LLtoSwissGrid(const double Lat, const double Long, double &SwissNorthing,
                   double &SwissEasting)
{
  //converts lat/long to Swiss Grid coords.  Equations from "Supplementary PROJ.4 Notes-
  //Swiss Oblique Mercator Projection", August 5, 1995, Release 4.3.3, by Gerald I. Evenden
  //Lat and Long are in decimal degrees
  //This transformation is, of course, only valid in Switzerland
  //Written by Chuck Gantz- chuck.gantz@globalstar.com
  double a = ellipsoid[3].EquatorialRadius; //Bessel ellipsoid
  double eccSquared = ellipsoid[3].eccentricitySquared;
  double ecc = sqrt(eccSquared);

  double LongOrigin = 7.43958333; //E7d26'22.500"
  double LatOrigin = 46.95240556; //N46d57'8.660"

  double LatRad = Lat * deg2rad;
  double LongRad = Long * deg2rad;
  double LatOriginRad = LatOrigin * deg2rad;
  double LongOriginRad = LongOrigin * deg2rad;

  double c = sqrt(
      1 + ((eccSquared * pow(cos(LatOriginRad), 4)) / (1 - eccSquared)));

  double equivLatOrgRadPrime = asin(sin(LatOriginRad) / c);

  //eqn. 1
  double K = log(tan(FOURTHPI + equivLatOrgRadPrime / 2))
      - c
          * (log(tan(FOURTHPI + LatOriginRad / 2))
              - ecc / 2
                  * log(
                      (1 + ecc * sin(LatOriginRad))
                          / (1 - ecc * sin(LatOriginRad))));

  double LongRadPrime = c * (LongRad - LongOriginRad); //eqn 2
  double w = c
      * (log(tan(FOURTHPI + LatRad / 2))
          - ecc / 2 * log((1 + ecc * sin(LatRad)) / (1 - ecc * sin(LatRad))))
      + K; //eqn 1
  double LatRadPrime = 2 * (atan(exp(w)) - FOURTHPI); //eqn 1

  //eqn 3
  double sinLatDoublePrime = cos(equivLatOrgRadPrime) * sin(LatRadPrime)
      - sin(equivLatOrgRadPrime) * cos(LatRadPrime) * cos(LongRadPrime);
  double LatRadDoublePrime = asin(sinLatDoublePrime);

  //eqn 4
  double sinLongDoublePrime = cos(LatRadPrime) * sin(LongRadPrime)
      / cos(LatRadDoublePrime);
  double LongRadDoublePrime = asin(sinLongDoublePrime);

  double R = a * sqrt(1 - eccSquared)
      / (1 - eccSquared * sin(LatOriginRad) * sin(LatOriginRad));

  SwissNorthing = R * log(tan(FOURTHPI + LatRadDoublePrime / 2)) + 200000.0; //eqn 5
  SwissEasting = R * LongRadDoublePrime + 600000.0; //eqn 6

}

void SwissGridtoLL(const double SwissNorthing, const double SwissEasting,
                   double& Lat, double& Long)
{
  double a = ellipsoid[3].EquatorialRadius; //Bessel ellipsoid
  double eccSquared = ellipsoid[3].eccentricitySquared;
  // double ecc = sqrt(eccSquared);

  double LongOrigin = 7.43958333; //E7d26'22.500"
  double LatOrigin = 46.95240556; //N46d57'8.660"

  double LatOriginRad = LatOrigin * deg2rad;
  double LongOriginRad = LongOrigin * deg2rad;

  double R = a * sqrt(1 - eccSquared)
      / (1 - eccSquared * sin(LatOriginRad) * sin(LatOriginRad));

  double LatRadDoublePrime = 2
      * (atan(exp((SwissNorthing - 200000.0) / R)) - FOURTHPI); //eqn. 7
  double LongRadDoublePrime = (SwissEasting - 600000.0) / R; //eqn. 8 with equation corrected

  double c = sqrt(
      1 + ((eccSquared * pow(cos(LatOriginRad), 4)) / (1 - eccSquared)));
  double equivLatOrgRadPrime = asin(sin(LatOriginRad) / c);

  double sinLatRadPrime = cos(equivLatOrgRadPrime) * sin(LatRadDoublePrime)
      + sin(equivLatOrgRadPrime) * cos(LatRadDoublePrime)
          * cos(LongRadDoublePrime);
  double LatRadPrime = asin(sinLatRadPrime);

  double sinLongRadPrime = cos(LatRadDoublePrime) * sin(LongRadDoublePrime)
      / cos(LatRadPrime);
  double LongRadPrime = asin(sinLongRadPrime);

  Long = (LongRadPrime / c + LongOriginRad) * rad2deg;

  Lat = NewtonRaphson(LatRadPrime) * rad2deg;

}

double NewtonRaphson(const double initEstimate)
{
  double Estimate = initEstimate;
  double tol = 0.00001;
  double corr;

  double eccSquared = ellipsoid[3].eccentricitySquared;
  double ecc = sqrt(eccSquared);

  double LatOrigin = 46.95240556; //N46d57'8.660"
  double LatOriginRad = LatOrigin * deg2rad;

  double c = sqrt(
      1 + ((eccSquared * pow(cos(LatOriginRad), 4)) / (1 - eccSquared)));

  double equivLatOrgRadPrime = asin(sin(LatOriginRad) / c);

  //eqn. 1
  double K = log(tan(FOURTHPI + equivLatOrgRadPrime / 2))
      - c
          * (log(tan(FOURTHPI + LatOriginRad / 2))
              - ecc / 2
                  * log(
                      (1 + ecc * sin(LatOriginRad))
                          / (1 - ecc * sin(LatOriginRad))));
  double C = (K - log(tan(FOURTHPI + initEstimate / 2))) / c;

  do
  {
    corr = CorrRatio(Estimate, C);
    Estimate = Estimate - corr;
  } while (fabs(corr) > tol);

  return Estimate;
}

double CorrRatio(double LatRad, const double C)
{
  double eccSquared = ellipsoid[3].eccentricitySquared;
  double ecc = sqrt(eccSquared);
  double corr = (C + log(tan(FOURTHPI + LatRad / 2))
      - ecc / 2 * log((1 + ecc * sin(LatRad)) / (1 - ecc * sin(LatRad))))
      * (((1 - eccSquared * sin(LatRad) * sin(LatRad)) * cos(LatRad))
          / (1 - eccSquared));

  return corr;
}

bool LatLongDeltaMeters(double lat1, double lon1, double lat2, double lon2,
                        double &MetersNorth, double &MetersEast)
{

  //(semimajor axis)
  double dfa = 6378137;
  // (semiminor axis)
  double dfb = 6356752;

  double dftanlat2 = pow(tan(lat1 * deg2rad), 2);
  double dfRadius = dfb * sqrt(1 + dftanlat2)
      / sqrt((pow(dfb, 2) / pow(dfa, 2)) + dftanlat2);

  //the decimal degrees conversion should take place elsewhere
  double dXArcDeg = (lon1 - lon2) * deg2rad;
  double dX = dfRadius * sin(dXArcDeg) * cos(lat1 * deg2rad);

  double dYArcDeg = (lat1 - lat2) * deg2rad;
  double dY = dfRadius * sin(dYArcDeg);

  //This is the total distance traveled thus far, either North or East
  MetersNorth = dX;
  MetersEast = dY;

  return true;
}

bool LatLongTranslateMeters(double lat1, double lon1, double &lat2, double &lon2,
                          double MetersNorth, double MetersEast)
{

//  //(semimajor axis)
//  double dfa = 6278137;
//  // (semiminor axis)
//  double dfb = 7058600;

  //(semimajor axis)
  double dfa = 6378137;
  // (semiminor axis)
  double dfb = 6356752;

  double dftanlat2 = pow(tan(lat1 * deg2rad), 2);
  double dfRadius = dfb * sqrt(1 + dftanlat2)
      / sqrt((pow(dfb, 2) / pow(dfa, 2)) + dftanlat2);

  //the decimal degrees conversion should take place elsewhere

  lon2 =
      (-1
          * (asin(MetersEast / (dfRadius * acos(lat1 * deg2rad)))
              - (lon1 * deg2rad))) * rad2deg;

  lat2 = (-1 * (asin(MetersNorth / dfRadius) - (lat1 * deg2rad))) * rad2deg;

  return true;
}
