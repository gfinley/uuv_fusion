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

#ifndef CONVERTLLUTM_H_INCLUDED
#define CONVERTLLUTM_H_INCLUDED



/*Reference ellipsoids derived from Peter H. Dana's website-
http://www.utexas.edu/depts/grg/gcraft/notes/datum/elist.html
Department of Geography, University of Texas at Austin
Internet: pdana@mail.utexas.edu
3/22/95

Source
Defense Mapping Agency. 1987b. DMA Technical Report: Supplement to Department of Defense World Geodetic System
1984 Technical Report. Part I and II. Washington, DC: Defense Mapping Agency
*/

#ifndef INCLUDED_PRECOMPILED_HEADERS
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <math.h>
#endif // INCLUDED_PRECOMPILED_HEADERS

#define REFELLIPSOID_WGS84 23

// Foreward declarations of routines defined in this file
void LLtoUTM(int ReferenceEllipsoid, const double Lat, const double Long,
       double &UTMNorthing, double &UTMEasting, char* UTMZone);

void UTMtoLL(int ReferenceEllipsoid, const double UTMNorthing, const double UTMEasting, const char* UTMZone,
        double& Lat,  double& Long );

bool LatLongDeltaMeters(double lat1, double lon1, double lat2, double lon2,
                        double &MetersNorth, double &MetersEast);

bool LatLongTranslateMeters(double lat1, double lon1, double &lat2, double &lon2,
                          double MetersNorth, double MetersEast);

char UTMLetterDesignator(double Lat);

void LLtoSwissGrid(const double Lat, const double Long,
       double &SwissNorthing, double &SwissEasting);

void SwissGridtoLL(const double SwissNorthing, const double SwissEasting,
          double& Lat, double& Long);

double CorrRatio(double LatRad, const double C);

double NewtonRaphson(const double initEstimate);

//constants.h

#ifndef CONSTANTS_H
#define CONSTANTS_H

const double PI = 3.14159265;
const double FOURTHPI = PI / 4;
const double deg2rad = PI / 180;
const double rad2deg = 180.0 / PI;

class Ellipsoid
{
public:
  Ellipsoid(){};
  Ellipsoid(int Id, std::string name, double radius, double ecc)
  {
    id = Id; ellipsoidName = name;
    EquatorialRadius = radius; eccentricitySquared = ecc;
  }

  int id;
  std::string ellipsoidName;
  double EquatorialRadius;
  double eccentricitySquared;

};

static Ellipsoid ellipsoid[] =
{//  id, Ellipsoid name, Equatorial Radius, square of eccentricity
  Ellipsoid( -1, "Placeholder", 0, 0),//placeholder only, To allow array indices to match id numbers
  Ellipsoid( 1, "Airy", 6377563, 0.00667054),
  Ellipsoid( 2, "Australian National", 6378160, 0.006694542),
  Ellipsoid( 3, "Bessel 1841", 6377397, 0.006674372),
  Ellipsoid( 4, "Bessel 1841 (Nambia) ", 6377484, 0.006674372),
  Ellipsoid( 5, "Clarke 1866", 6378206, 0.006768658),
  Ellipsoid( 6, "Clarke 1880", 6378249, 0.006803511),
  Ellipsoid( 7, "Everest", 6377276, 0.006637847),
  Ellipsoid( 8, "Fischer 1960 (Mercury) ", 6378166, 0.006693422),
  Ellipsoid( 9, "Fischer 1968", 6378150, 0.006693422),
  Ellipsoid( 10, "GRS 1967", 6378160, 0.006694605),
  Ellipsoid( 11, "GRS 1980", 6378137, 0.00669438),
  Ellipsoid( 12, "Helmert 1906", 6378200, 0.006693422),
  Ellipsoid( 13, "Hough", 6378270, 0.00672267),
  Ellipsoid( 14, "International", 6378388, 0.00672267),
  Ellipsoid( 15, "Krassovsky", 6378245, 0.006693422),
  Ellipsoid( 16, "Modified Airy", 6377340, 0.00667054),
  Ellipsoid( 17, "Modified Everest", 6377304, 0.006637847),
  Ellipsoid( 18, "Modified Fischer 1960", 6378155, 0.006693422),
  Ellipsoid( 19, "South American 1969", 6378160, 0.006694542),
  Ellipsoid( 20, "WGS 60", 6378165, 0.006693422),
  Ellipsoid( 21, "WGS 66", 6378145, 0.006694542),
  Ellipsoid( 22, "WGS-72", 6378135, 0.006694318),
  Ellipsoid( 23, "WGS-84", 6378137, 0.00669438)
};

#endif






#endif
