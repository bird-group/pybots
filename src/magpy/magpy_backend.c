#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "GeomagnetismHeader.h"
#include "EGM9615.h"

static PyObject * magpy_backend(PyObject * self, PyObject * args)
{
    // Input:
    //  lat: rad
    //  lon: rad
    //  alt: m, WGS84
    //  year
    //  month
    //  day
    const double lat;
    const double lon;
    const double alt;
    const int year;
    const int month;
    const int day;
    MAGtype_MagneticModel * MagneticModels[1], *TimedMagneticModel;
    MAGtype_Ellipsoid Ellip;
    MAGtype_CoordSpherical CoordSpherical;
    MAGtype_CoordGeodetic CoordGeodetic;
    MAGtype_Date UserDate;
    MAGtype_GeoMagneticElements GeoMagneticElements, Errors;
    MAGtype_Geoid Geoid;
    //char filename[] = "WMM.COF";
    const char *cof_path;
    char VersionDate[12];
    int NumTerms, Flag = 1, nMax = 0;
    int epochs = 1;
    char Error_Message[255];

    if (!PyArg_ParseTuple(args, "sdddiii", &cof_path, &lat, &lon, &alt, &year, &month, &day))
        return NULL;

    strncpy(VersionDate, VERSIONDATE_LARGE + 39, 11);
    VersionDate[11] = '\0';
    if (!MAG_robustReadMagModels(cof_path, &MagneticModels, epochs)) {
        printf("\n WMM.COF not found.");
        return NULL;
    }
    if(nMax < MagneticModels[0]->nMax) nMax = MagneticModels[0]->nMax;
    NumTerms = ((nMax + 1) * (nMax + 2) / 2);
    TimedMagneticModel = MAG_AllocateModelMemory(NumTerms); /* For storing the time modified WMM Model parameters */
    MAG_SetDefaults(&Ellip, &Geoid); /* Set default values and constants */

    CoordGeodetic.phi = lat;
    CoordGeodetic.lambda = lon;
    Geoid.UseGeoid = 0;
    CoordGeodetic.HeightAboveEllipsoid = alt;
    CoordGeodetic.HeightAboveGeoid = CoordGeodetic.HeightAboveEllipsoid;

    UserDate.Year = year;
    UserDate.Month = month;
    UserDate.Day = day;
    if (!MAG_DateToYear(&UserDate, Error_Message)) {
        MAG_FreeMagneticModelMemory(MagneticModels[0]);
        MAG_FreeMagneticModelMemory(TimedMagneticModel);
        return NULL;
    }

    MAG_GeodeticToSpherical(Ellip, CoordGeodetic, &CoordSpherical);
    MAG_TimelyModifyMagneticModel(
        UserDate, MagneticModels[0], TimedMagneticModel);
    MAG_Geomag(
        Ellip, 
        CoordSpherical, 
        CoordGeodetic, 
        TimedMagneticModel,
        &GeoMagneticElements
        );
    MAG_FreeMagneticModelMemory(MagneticModels[0]);
    MAG_FreeMagneticModelMemory(TimedMagneticModel);
    
    return Py_BuildValue(
        "(ddd)", 
        GeoMagneticElements.X,
        GeoMagneticElements.Y,
        GeoMagneticElements.Z
        );
}

static PyMethodDef MagPyBackendMethods[] = {
    { "magpy_backend", magpy_backend, METH_VARARGS, "magpy_backend"},
    { NULL, NULL, 0, NULL }
};

#if PY_MAJOR_VERSION >= 3
static struct PyModuleDef magpy_backend_module = {
    PyModuleDef_HEAD_INIT,
    "magpy_backend",
    NULL,
    -1,
    MagPyBackendMethods
};

PyMODINIT_FUNC
PyInit_magpy_backend(void)
{
    return PyModule_Create(&magpy_backend_module);
}
#endif //PY_MAJOR_VERSION >= 3

#if PY_MAJOR_VERSION < 3
DL_EXPORT(void) initmagpy_backend(void)
{
    Py_InitModule("magpy_backend", MagPyBackendMethods);
}
#endif //PY_MAJOR_VERSION < 3

